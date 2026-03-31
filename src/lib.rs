use log::{debug, trace, warn};
use pathfinding::prelude::{kuhn_munkres_min, Matrix};
use petgraph::prelude::StableGraph;
use petgraph::stable_graph::{NodeIndex, StableDiGraph};
use petgraph::visit::EdgeRef;
use petgraph::{Directed, Graph, Undirected};
use pyo3::prelude::*;
use rayon::prelude::*;
use rustc_hash::FxHashMap as HashMap;
use serde::{Deserialize, Serialize};
use std::hash::{Hash, Hasher};
pub type TripId = u32;
pub type DeltaSoC = f32; // 0 - 1
pub type Duration = u32; // seconds
/// Sentinel value for non-existent edges in the Hungarian algorithm's cost matrix.
const NO_EDGE_WEIGHT: i64 = i32::MAX as i64;
#[derive(Serialize, Deserialize, Clone, Debug, Copy, Default)]
pub struct NodeWeight {
    delta_soc: DeltaSoC,
    duration: Duration,
}
pub type EdgeWeight = Duration;

/// Solve the vehicle scheduling problem
/// This is the entry point (and only exposed function) for the Python module
/// It takes a JSON string as input and returns a list of (TripId, TripId) pairs
#[pyfunction]
pub fn solve(
    input_json: String,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> PyResult<Vec<(TripId, TripId)>> {
    let input_json = input_json.to_string();
    let json_file = read_graph_from_string(input_json);
    let result = soc_aware_rotation_plan_vec(&json_file, max_delta_soc, max_duration);

    Ok(result)
}

/// A Python module implemented in Rust.
#[pymodule]
pub fn eflips_schedule_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(solve, m)?)?;
    Ok(())
}

/// The list of nodes is an inner level structure of the graph
/// The IDs are TripIds, the weights are a delta_soc_effective, duration tuple.
#[derive(Serialize, Deserialize)]
struct JsonNode {
    id: TripId,
    weight: (Option<DeltaSoC>, Option<Duration>),
}

/// The list of edges is an inner level structure of the graph
/// An edge connects two nodes and has a weight
/// The source and target are the TripIds of the nodes
#[derive(Serialize, Deserialize)]
struct JsonEdge {
    source: TripId,
    target: TripId,
    weight: EdgeWeight,
}

/// The Json file is composed of the graph already split into a set of connected
/// subgraphs
#[derive(Serialize, Deserialize)]
struct JsonGraph {
    nodes: Vec<JsonNode>,
    edges: Vec<JsonEdge>,
}

/// The BusGraph is the internal representation of the graph
/// It contains the graph itself, as well as maps to convert between TripIds and NodeIndices
/// Also, the topological order of the nodes is stored, so we don't need to recalculate it
/// every time.
#[derive(Clone, Debug)]
pub struct BusGraph {
    graph: StableGraph<NodeWeight, EdgeWeight, Directed>,
    node_id_trip_id: HashMap<NodeIndex, TripId>,
    trip_id_node_id: HashMap<TripId, NodeIndex>,
    topo_order: HashMap<NodeIndex, usize>,
}

/// Reads in the graph from a JSON input string
/// The JSON string should contain first a list of (node_id, weight(s)) pairs
/// Then a list of (source, target, weight) triples
fn read_graph_from_string(input: String) -> Vec<BusGraph> {
    let json_file: Vec<JsonGraph> =
        serde_json::from_str(&input).expect("Error while reading JSON file");

    json_graph_to_bus_graph(json_file)
}

/// Reads in the graph from a JSON input file
/// The JSON string should contain first a list of (node_id, weight (=delta_soc_effective)) pairs
/// Then a list of (source, target, weight) triples
pub fn read_graph_from_file(input: &str) -> Vec<BusGraph> {
    let file = std::fs::read_to_string(input).expect("Could not read file");
    let json_file: Vec<JsonGraph> =
        serde_json::from_str(&file).expect("Error while reading JSON file");

    json_graph_to_bus_graph(json_file)
}

/// Turns a Vec of JsonGraph into a Vec of BusGraph
fn json_graph_to_bus_graph(json_graphs: Vec<JsonGraph>) -> Vec<BusGraph> {
    let mut all_graphs = Vec::new();
    for json_graph in json_graphs {
        // Set up nodes and corresponding ID maps
        let mut trip_id_node_id: HashMap<TripId, NodeIndex> = HashMap::with_capacity_and_hasher(json_graph.nodes.len(), Default::default());
        let mut graph = StableDiGraph::new();
        for node in json_graph.nodes {
            // Turn the None values into 0.0
            let weight = NodeWeight {
                delta_soc: node.weight.0.unwrap_or(0.0),
                duration: node.weight.1.unwrap_or(0),
            };

            let node_id = graph.add_node(weight);
            trip_id_node_id.insert(node.id, node_id);
        }
        let node_id_trip_id: HashMap<NodeIndex, TripId> =
            trip_id_node_id.iter().map(|(k, v)| (*v, *k)).collect();

        // Add edges
        for edge in json_graph.edges {
            let source = trip_id_node_id
                .get(&edge.source)
                .expect("Source node not found");
            let target = trip_id_node_id
                .get(&edge.target)
                .expect("Target node not found");
            graph.add_edge(*source, *target, edge.weight);
        }

        // Calculate the topological order of the nodes
        let toposort = petgraph::algo::toposort(&graph, None).expect("Graph is not a DAG");
        let mut topo_order: HashMap<NodeIndex, usize> = HashMap::with_capacity_and_hasher(toposort.len(), Default::default());
        for (i, node) in toposort.iter().enumerate() {
            topo_order.insert(*node, i);
        }

        all_graphs.push(BusGraph {
            graph,
            node_id_trip_id,
            trip_id_node_id,
            topo_order,
        });
    }

    all_graphs
}

/// Calculate the total number of rotations
/// This is the number of unbroken sequences of two trips
/// that are matched to each other
/// (domino style)
pub fn total_rotation_count(
    rotation_connections: &Vec<(TripId, TripId)>,
    bus_graph: &BusGraph,
) -> usize {
    let working_graph = assemble_working_graph(rotation_connections, bus_graph);
    let connected_sets = petgraph::algo::kosaraju_scc(&working_graph);
    connected_sets.len()
}

/// Create a new working graph that has edges between all nodes from the rotation connections input
/// from the original graph.
/// This is a precondition to calculating the sum of weights of the nodes in a connected set
fn assemble_working_graph(
    rotation_connections: &Vec<(TripId, TripId)>,
    bus_graph: &BusGraph,
) -> StableGraph<NodeWeight, EdgeWeight> {
    let mut working_graph = bus_graph.graph.clone();
    working_graph.retain_edges(|_, _edge| false);
    for rotation_connection in rotation_connections {
        let node1 = *bus_graph
            .trip_id_node_id
            .get(&rotation_connection.0)
            .expect("Trip not found!");
        let node2 = *bus_graph
            .trip_id_node_id
            .get(&rotation_connection.1)
            .expect("Trip not found!");
        working_graph.add_edge(node1, node2, 0);
        working_graph.add_edge(node2, node1, 0);
    }

    working_graph
}

fn sort_rotation_topologically(nodes: &[NodeIndex], bus_graph: &BusGraph) -> Vec<NodeIndex> {
    let mut sorted = nodes.to_vec();
    sorted.sort_unstable_by_key(|n| {
        *bus_graph
            .topo_order
            .get(n)
            .expect("Node not found in topo map")
    });
    sorted
}

fn exceeds_limits(
    weight: &NodeWeight,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> bool {
    max_delta_soc.is_some_and(|max| weight.delta_soc > max)
        || max_duration.is_some_and(|max| weight.duration > max)
}

/// Returns the index of the first node (in the given sorted slice) where the cumulative
/// weight (node weights + inter-trip edge weights) first exceeds the given limits.
/// Returns None if no limit is exceeded.
fn first_excess_index(
    sorted_nodes: &[NodeIndex],
    bus_graph: &BusGraph,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> Option<usize> {
    let mut weight = NodeWeight::default();
    for (i, node) in sorted_nodes.iter().enumerate() {
        if i > 0 {
            let prev = sorted_nodes[i - 1];
            let edge = bus_graph.graph.find_edge_undirected(prev, *node);
            weight.duration = weight.duration.checked_add(
                *bus_graph.graph.edge_weight(edge.unwrap().0).unwrap()
            ).expect("Duration overflow in first_excess_index");
        }
        let nw = *bus_graph
            .graph
            .node_weight(*node)
            .expect("Node has no weight!");
        weight.delta_soc += nw.delta_soc;
        weight.duration = weight.duration.checked_add(nw.duration)
            .expect("Duration overflow in first_excess_index");
        if exceeds_limits(&weight, max_delta_soc, max_duration) {
            return Some(i);
        }
    }
    None
}

/// Returns true if the cumulative weight of the rotation exceeds the given limits.
fn is_rotation_excessive(
    connected_set: &[NodeIndex],
    bus_graph: &BusGraph,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> bool {
    let sorted = sort_rotation_topologically(connected_set, bus_graph);
    first_excess_index(&sorted, bus_graph, max_delta_soc, max_duration).is_some()
}

/// Find the node lists of the rotations with delta_soc_effective > 1
/// The node lists will be returned in topological order
fn excessive_rotations(
    rotation_connections: &Vec<(TripId, TripId)>,
    bus_graph: &BusGraph,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> Vec<Vec<NodeIndex>> {
    let working_graph = assemble_working_graph(rotation_connections, bus_graph);

    let connected_sets = petgraph::algo::kosaraju_scc(&working_graph);
    let mut excessive_soc_rotations: Vec<Vec<NodeIndex>> = Vec::new();
    for connected_set in connected_sets {
        if is_rotation_excessive(&connected_set, bus_graph, max_delta_soc, max_duration) {
            excessive_soc_rotations.push(connected_set);
        }
    }
    excessive_soc_rotations
}

/// Take a list of nodes of a rotation that has excessive SOC
/// and return the amount of it from the start up to the last one that
fn nodes_to_remove(
    rotation: &Vec<NodeIndex>,
    bus_graph: &BusGraph,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
    forward: bool,
) -> Vec<NodeIndex> {
    let mut sorted = sort_rotation_topologically(rotation, bus_graph);
    if !forward {
        sorted.reverse();
    }
    let cutoff =
        first_excess_index(&sorted, bus_graph, max_delta_soc, max_duration).unwrap_or(sorted.len());
    sorted[..cutoff].to_vec()
}

/// Calculate the cost of removing a set of nodes from the graph. THis is evaluated for all
/// candidate rotations that have excessive SOC. Then the one with the smallest cost is removed.
fn cost_of_removal(
    rotation: &Vec<NodeIndex>,
    working_graph: &BusGraph,
    graph: &BusGraph,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> (Vec<NodeIndex>, usize) {
    /// The weight we give to the total number of rotations
    const TOTAL_ROTATIONS_WEIGHT: usize = 1e9 as usize;

    /// The weight we give to the number of excessive rotations
    /// This is a tiebreaker in case the total number of rotations is the same
    const EXCESSIVE_ROTATIONS_WEIGHT: usize = 1;

    let nodes_to_remove_front = nodes_to_remove(rotation, graph, max_delta_soc, max_duration, true);
    let weight_front = {
        // Remove the nodes from a copy of the graph
        let mut bus_graph_copy = working_graph.clone();
        for node in &nodes_to_remove_front {
            bus_graph_copy.graph.remove_node(*node);
        }
        let bipartite_graph = to_bipartite(&bus_graph_copy);

        let rotation_plan_after_removal_front = maximum_matching(bipartite_graph);

        // Calculate some quality metrics
        let total_rotations_after_removal_front =
            total_rotation_count(&rotation_plan_after_removal_front, &bus_graph_copy);
        let excessive_rotations_after_removal_front = excessive_rotations(
            &rotation_plan_after_removal_front,
            &bus_graph_copy,
            max_delta_soc,
            max_duration,
        )
        .len();
        total_rotations_after_removal_front * TOTAL_ROTATIONS_WEIGHT
            + excessive_rotations_after_removal_front * EXCESSIVE_ROTATIONS_WEIGHT
    };

    let nodes_to_remove_back = nodes_to_remove(rotation, graph, max_delta_soc, max_duration, false);
    let weight_back = {
        // Remove the nodes from a copy of the graph
        let mut bus_graph_copy = working_graph.clone();
        for node in &nodes_to_remove_back {
            bus_graph_copy.graph.remove_node(*node);
        }
        let bipartite_graph = to_bipartite(&bus_graph_copy);
        let rotation_plan_after_removal_back = maximum_matching(bipartite_graph);
        let total_rotations_after_removal_back =
            total_rotation_count(&rotation_plan_after_removal_back, &bus_graph_copy);
        let excessive_rotations_after_removal_back = excessive_rotations(
            &rotation_plan_after_removal_back,
            &bus_graph_copy,
            max_delta_soc,
            max_duration,
        )
        .len();
        total_rotations_after_removal_back * TOTAL_ROTATIONS_WEIGHT
            + excessive_rotations_after_removal_back * EXCESSIVE_ROTATIONS_WEIGHT
    };

    if weight_front < weight_back {
        trace!("Weight front: {}", weight_front);
        (nodes_to_remove_front, weight_front)
    } else {
        trace!("Weight back: {}", weight_front);
        let reversed_nodes_to_remove_back = nodes_to_remove_back.iter().rev().cloned().collect();
        (reversed_nodes_to_remove_back, weight_back)
    }
}

/// Solve the rotation plan problem in a SOC-aware way
pub fn soc_aware_rotation_plan(
    graph: &BusGraph,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> Vec<(TripId, TripId)> {
    ///Whether to use rayon for parallel processing
    const PARALLEL: bool = true;

    // Construct a non-soc-aware rotation plan
    let bipartite_graph = to_bipartite(graph);
    let mut rotation_plan = maximum_matching(bipartite_graph);

    let mut rotations_removed: Vec<Vec<NodeIndex>> = Vec::new();
    let mut working_graph = graph.clone();
    loop {
        let the_excessive_soc_rotations =
            excessive_rotations(&rotation_plan, &working_graph, max_delta_soc, max_duration);
        if the_excessive_soc_rotations.is_empty() {
            break;
        }

        let effects_of_removal = {
            if PARALLEL {
                debug!(
                    "Checking {} rotations in parallel",
                    the_excessive_soc_rotations.len()
                );
                the_excessive_soc_rotations
                    .into_par_iter()
                    .map(|rotation| {
                        cost_of_removal(
                            &rotation,
                            &working_graph,
                            graph,
                            max_delta_soc,
                            max_duration,
                        )
                    })
                    .collect::<Vec<(Vec<NodeIndex>, usize)>>()
            } else {
                warn!("Not using parallel processing!");
                let mut effects_of_removal: Vec<(Vec<NodeIndex>, usize)> = Vec::new();
                for rotation in &the_excessive_soc_rotations {
                    let (nodes_to_remove, effect) = cost_of_removal(
                        rotation,
                        &working_graph,
                        graph,
                        max_delta_soc,
                        max_duration,
                    );
                    effects_of_removal.push((nodes_to_remove, effect));
                }
                effects_of_removal
            }
        };

        // Find the rotation that has the most positive effect on the total number of rotations
        let mut best_rotation: Option<Vec<NodeIndex>> = None;
        let mut best_effect: usize = usize::MAX;
        for (rotation, effect) in &effects_of_removal {
            if *effect < best_effect {
                best_rotation = Some(rotation.clone());
                best_effect = *effect;
            }
        }
        debug!("Best effect: {}", best_effect);
        // Remove the best rotation from the working graph
        let best_rotation = best_rotation.expect("No best rotation found!");

        rotations_removed.push(best_rotation.clone());
        for node in best_rotation {
            let _result = working_graph
                .graph
                .remove_node(node)
                .expect("Node not found!");
        }
        rotation_plan = maximum_matching(to_bipartite(&working_graph));
    }

    // Now, we solve the maximum matching problem on the working graph one last time
    let bipartite_graph = to_bipartite(&working_graph);
    let mut rotation_plan = maximum_matching(bipartite_graph);

    // Convert the rotations that were removed to (TripId, TripId) pairs and add them to the
    // rotation plan
    for rotation in rotations_removed {
        for i in 0..rotation.len() - 1 {
            let trip1 = *working_graph
                .node_id_trip_id
                .get(&rotation[i])
                .expect("Node not found!");
            let trip2 = *working_graph
                .node_id_trip_id
                .get(&rotation[i + 1])
                .expect("Node not found!");
            rotation_plan.push((trip1, trip2));
        }
    }

    rotation_plan
}

/// Convenience method to calculate the SoC-aware rotation plan for a vector of graphs
fn soc_aware_rotation_plan_vec(
    graphs: &Vec<BusGraph>,
    max_delta_soc: Option<DeltaSoC>,
    max_duration: Option<Duration>,
) -> Vec<(TripId, TripId)> {
    let mut results: Vec<(TripId, TripId)> = Vec::new();
    for graph in graphs {
        let edged = soc_aware_rotation_plan(graph, max_delta_soc, max_duration);
        for edge in edged {
            results.push(edge);
        }
    }
    results
}

pub struct BipartiteGraph {
    graph: Graph<(), EdgeWeight, Undirected>,
    top_node_id_trip_id: HashMap<NodeIndex, TripId>,
    bottom_node_id_trip_id: HashMap<NodeIndex, TripId>,
}

/// Take the core graph for the bus graph and convert it to a bipartite graph
/// Where one set of  nodes only has outgoing edges and the other only incoming edges
/// On this graph we can then solve the maximum matching problem
fn to_bipartite(graph: &BusGraph) -> BipartiteGraph {
    let node_count = graph.graph.node_count();
    let mut bipartite_graph: Graph<(), EdgeWeight, Undirected> = Graph::with_capacity(node_count * 2, graph.graph.edge_count());

    // Add nodes to the bipartite graph
    let mut top_node_id_bipartite_node_id: HashMap<NodeIndex, NodeIndex> = HashMap::with_capacity_and_hasher(node_count, Default::default());
    let mut bottom_bipartite_node_id_node_id: HashMap<NodeIndex, NodeIndex> = HashMap::with_capacity_and_hasher(node_count, Default::default());

    for node in graph.graph.node_indices() {
        let top_node_idx = bipartite_graph.add_node(());
        top_node_id_bipartite_node_id.insert(node, top_node_idx);

        let bottom_node_idx = bipartite_graph.add_node(());
        bottom_bipartite_node_id_node_id.insert(node, bottom_node_idx);
    }

    // Add the edges with their weight
    for node in graph.graph.node_indices() {
        for edge in graph.graph.edges(node) {
            let bipartite_top_idx = top_node_id_bipartite_node_id
                .get(&edge.source())
                .expect("Node not found!");
            let bipartite_bottom_idx = bottom_bipartite_node_id_node_id
                .get(&edge.target())
                .expect("Node not found!");
            bipartite_graph.add_edge(*bipartite_top_idx, *bipartite_bottom_idx, *edge.weight());
        }
    }

    // Assemble a map from the bipartite graph node ids to the original trip ids
    let mut top_node_id_trip_id: HashMap<NodeIndex, TripId> = HashMap::with_capacity_and_hasher(node_count, Default::default());
    for (node_id, bipartite_node_id) in top_node_id_bipartite_node_id {
        top_node_id_trip_id.insert(
            bipartite_node_id,
            *graph
                .node_id_trip_id
                .get(&node_id)
                .expect("Node not found!"),
        );
    }
    let mut bottom_node_id_trip_id: HashMap<NodeIndex, TripId> = HashMap::with_capacity_and_hasher(node_count, Default::default());
    for (node_id, bipartite_node_id) in bottom_bipartite_node_id_node_id {
        bottom_node_id_trip_id.insert(
            bipartite_node_id,
            *graph
                .node_id_trip_id
                .get(&node_id)
                .expect("Node not found!"),
        );
    }

    BipartiteGraph {
        graph: bipartite_graph,
        top_node_id_trip_id,
        bottom_node_id_trip_id,
    }
}

#[allow(dead_code)]
fn write_debug_dotfile(graph: &BipartiteGraph, connected_set: &Vec<NodeIndex>) {
    // Remove all nodes from the trip graph that are not in the connected set
    let mut dump_graph = graph.graph.clone();
    let mut nodes_to_remove: Vec<NodeIndex> = Vec::new();
    for node in dump_graph.node_indices() {
        if !connected_set.contains(&node) {
            nodes_to_remove.push(node);
        }
    }

    // Order the list highest to lowest
    nodes_to_remove.sort_by_key(|b| std::cmp::Reverse(b.index()));

    for node in nodes_to_remove {
        dump_graph.remove_node(node);
    }

    // Dump a dot representation of the graph to a file
    let dot = petgraph::dot::Dot::new(&dump_graph);
    let dot_string = format!("{:?}", dot);

    // The filename should be a hash of all the nodes
    let hash = {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        for node in connected_set {
            node.hash(&mut hasher);
        }
        hasher.finish()
    };
    let filename = format!("debug-{}.dot", hash);
    std::fs::write(filename, dot_string).expect("Could not write dot file");
}

/// Solve the maximum matching problem on the bipartite graph
/// Try using the Hungarian algorithm where possible
/// If the Hungarian algorithm fails, use the Hopcroft-Karp algorithm
/// **In the furture, this could be replaced with the blossom algorithm**
fn maximum_matching(graph: BipartiteGraph) -> Vec<(TripId, TripId)> {
    // First, we divide the graph into its connected sets
    let connected_sets = petgraph::algo::kosaraju_scc(&graph.graph);

    let mut matching: Vec<(TripId, TripId)> = Vec::new();

    // For each of these sets, solve the maximum matching problem
    for connected_set in connected_sets {
        // In the special case of our set only having one node, no matching is possible
        if connected_set.len() == 1 {
            continue;
        } else if connected_set.len() == 2 {
            // If we have two nodes, we can just match them
            // After figuring out which one is top and which one is bottom
            let (top_node, bottom_node) = {
                if graph.top_node_id_trip_id.contains_key(&connected_set[0]) {
                    (connected_set[0], connected_set[1])
                } else {
                    (connected_set[1], connected_set[0])
                }
            };

            let trip1 = *graph
                .top_node_id_trip_id
                .get(&top_node)
                .expect("Node not found!");
            let trip2 = *graph
                .bottom_node_id_trip_id
                .get(&bottom_node)
                .expect("Node not found!");
            matching.push((trip1, trip2));
        } else {
            // Recover the "top" and "bottom" nodes from the connected set
            // Their identity was lost during the SCC calculation
            let mut top_nodes: Vec<NodeIndex> = Vec::new();
            let mut bottom_nodes: Vec<NodeIndex> = Vec::new();
            for node in &connected_set {
                if graph.top_node_id_trip_id.contains_key(node) {
                    top_nodes.push(*node);
                } else {
                    bottom_nodes.push(*node);
                }
            }

            // We will be looking at the graph so that the "broader" set of nodes is at the
            // bottom, and the "narrower" set of nodes is at the top
            // This means we may need to swap the top and bottom nodes
            // We call them row and column nodes, for their usage in the Hungarian algorithm
            let half = connected_set.len() / 2 + 1;
            let mut row_trip_id: HashMap<NodeIndex, TripId> = HashMap::with_capacity_and_hasher(half, Default::default());
            let mut column_trip_id: HashMap<NodeIndex, TripId> = HashMap::with_capacity_and_hasher(half, Default::default());

            let mut inverted = false;
            let (column_nodes, row_nodes) = {
                if top_nodes.len() > bottom_nodes.len() {
                    // If we have more top than bottom nodes we assemble the matrix one way
                    // We create a column for each top node, and a row for each bottom node
                    for node in &top_nodes {
                        column_trip_id.insert(
                            *node,
                            *graph
                                .top_node_id_trip_id
                                .get(node)
                                .expect("Node not found!"),
                        );
                    }
                    for node in &bottom_nodes {
                        row_trip_id.insert(
                            *node,
                            *graph
                                .bottom_node_id_trip_id
                                .get(node)
                                .expect("Node not found!"),
                        );
                    }

                    (top_nodes, bottom_nodes)
                } else {
                    // If we have more bottom than top nodes we assemble the matrix the other way
                    // We create a row for each top node, and a column for each bottom node
                    inverted = true;
                    for node in &top_nodes {
                        row_trip_id.insert(
                            *node,
                            *graph
                                .top_node_id_trip_id
                                .get(node)
                                .expect("Node not found!"),
                        );
                    }
                    for node in &bottom_nodes {
                        column_trip_id.insert(
                            *node,
                            *graph
                                .bottom_node_id_trip_id
                                .get(node)
                                .expect("Node not found!"),
                        );
                    }

                    (bottom_nodes, top_nodes)
                }
            };

            let mut matrix: Vec<Vec<i64>> = Vec::new();
            for row in &row_nodes {
                let mut row_vec: Vec<i64> = Vec::new();
                for column in &column_nodes {
                    let edge_option = graph.graph.find_edge(*column, *row);
                    match edge_option {
                        None => row_vec.push(NO_EDGE_WEIGHT),
                        Some(edge) => {
                            row_vec
                                .push(*graph.graph.edge_weight(edge).expect("Edge has no weight!")
                                    as i64)
                        }
                    };
                }
                matrix.push(row_vec);
            }

            let weights = Matrix::from_rows(matrix).unwrap();
            let (total_wait_time, matching_indices) = kuhn_munkres_min(&weights);

            // The Hungarian algorithm *may* have created a matching that includes a node that
            // doesn't actually have an edge in the graph. If that is the case, we'll have to
            // use hopcroft-karp instead.
            // Invalid matchings are marked by a total wait time greater than NO_EDGE_WEIGHT
            if total_wait_time >= NO_EDGE_WEIGHT {
                // We need to find out if a matching that covers all of the column_nodes nodes is possible
                // If yes, we can use the Hungarian Algorithm (Kuhn-Munkres) to find the best
                // (lowest wait time) matching
                // If not, we use the Hopcroft-Karp algorithm to find *a* matching, which is not
                // guaranteed to be the best one

                // In order to check if a matching that covers all top nodes is possible, we run the
                // Hopcroft-Karp algorithm once to identify the maximum matching size

                // The hopcroft_karp package takes an edge list as input, so we need to convert our
                // graph to this format
                let mut wait_times = HashMap::default();
                let mut edge_list: Vec<(NodeIndex, NodeIndex)> = Vec::new();
                for node in &row_nodes {
                    for edge in graph.graph.edges(*node) {
                        let target = edge.target();
                        if column_nodes.contains(&edge.target()) {
                            edge_list.push((*node, target));
                        }
                        wait_times.insert((*node, target), edge.weight());
                    }
                }

                // The hopcroft-karp algorithm finds *one* maximum matching, not necessarily the best
                // one. However, by sorting the edges by their weight, we can get a good approximation
                // of the best matching
                edge_list
                    .sort_unstable_by_key(|edge| *wait_times.get(edge).expect("Edge not found!"));

                // We were not able to find a matching that covers all top nodes
                // We need to use the Hopcroft-Karp algorithm
                let maximum_matching = hopcroft_karp::matching(&edge_list);
                for (top, bottom) in maximum_matching {
                    let (trip1, trip2) = {
                        if inverted {
                            (
                                *row_trip_id.get(&top).expect("Node not found!"),
                                *column_trip_id.get(&bottom).expect("Node not found!"),
                            )
                        } else {
                            (
                                *column_trip_id.get(&bottom).expect("Node not found!"),
                                *row_trip_id.get(&top).expect("Node not found!"),
                            )
                        }
                    };
                    matching.push((trip1, trip2));
                }
            } else {
                // We have a valid matching
                for i in 0..matching_indices.len() {
                    let row = row_nodes[i];
                    let column = column_nodes[matching_indices[i]];

                    // Depending on how we assigned top and bottom, the order of the matching might be
                    // different. Since we don't want to get an edge in the wrong direction, we need to
                    // check whether we inverted our bipartite graph
                    let (trip1, trip2) = {
                        if inverted {
                            (
                                *row_trip_id.get(&row).expect("Node not found!"),
                                *column_trip_id.get(&column).expect("Node not found!"),
                            )
                        } else {
                            (
                                *column_trip_id.get(&column).expect("Node not found!"),
                                *row_trip_id.get(&row).expect("Node not found!"),
                            )
                        }
                    };
                    matching.push((trip1, trip2));
                }
            }
        }
    }
    matching
}

/// Write the matching to a JSON file. This is useful for debugging
#[allow(dead_code)]
fn edges_to_json(edges: Vec<(TripId, TripId)>, filename: String) {
    let json = serde_json::to_string(&edges).expect("Could not serialize matching to JSON");
    std::fs::write(filename, json).expect("Could not write JSON to file");
}

#[cfg(test)]
mod tests {
    use super::*;

    const NO_NODE_WEIGHT_PATH: &str = "test/no_node_weights.json";
    const ONE_NODE_WEIGHT_PATH: &str = "test/one_node_weight.json";
    const BOTH_NODE_WEIGHTS_PATH: &str = "test/both_node_weights.json";

    /// Takes a solution (as a list of (TripId, TripId) pairs) as swell as the graph this is
    /// placed on, and checks if the sum of the node weights of the nodes in the connected set
    /// is below the limit
    /// Panics if the sum of the node weights is above the limit
    fn validate_sum_of_node_weights(
        solution: &Vec<(TripId, TripId)>,
        graph: &BusGraph,
        max_delta_soc: Option<DeltaSoC>,
        max_duration: Option<Duration>,
    ) {
        let working_graph = assemble_working_graph(solution, graph);
        let connected_sets = petgraph::algo::kosaraju_scc(&working_graph);
        for connected_set in connected_sets {
            let weight_sum = node_weight_sum(&connected_set, graph);
            if max_delta_soc.is_some() && weight_sum.delta_soc > max_delta_soc.unwrap() {
                panic!(
                    "max_delta_soc is {}, actual {}",
                    max_delta_soc.unwrap(),
                    weight_sum.delta_soc
                );
            }
            if max_duration.is_some() && weight_sum.duration > max_duration.unwrap() {
                panic!(
                    "max_duration is {}, actual {}",
                    max_duration.unwrap(),
                    weight_sum.duration
                );
            }
        }
    }

    /// Calculate the sum of weights of the nodes in a connected set
    fn node_weight_sum(connected_set: &Vec<NodeIndex>, bus_graph: &BusGraph) -> NodeWeight {
        let sorted_connected_set = sort_rotation_topologically(connected_set, bus_graph);
        let mut weight_sums = NodeWeight::default();
        for (i, node) in sorted_connected_set.iter().enumerate() {
            let cur_trip_weight = *bus_graph
                .graph
                .node_weight(*node)
                .expect("Node has no weight!");
            weight_sums.delta_soc += cur_trip_weight.delta_soc;
            weight_sums.duration += cur_trip_weight.duration;

            // For the duration, we also have to count the edge weight (which is a waiting duration) into the total.
            if i < sorted_connected_set.len() - 1 {
                let next_node = sorted_connected_set[i + 1];
                let edge = bus_graph.graph.find_edge_undirected(*node, next_node);
                let edge_weight = bus_graph.graph.edge_weight(edge.unwrap().0).unwrap();
                weight_sums.duration += *edge_weight;
            }
        }
        weight_sums
    }

    #[test]
    fn test_read_graph_from_string() {
        // We read the file into a string, then test the function
        let input_str = std::fs::read_to_string(NO_NODE_WEIGHT_PATH).expect("Could not read file");
        let graphs: Vec<BusGraph> = read_graph_from_string(input_str);
        assert_eq!(graphs.len(), 12);
    }

    #[test]
    fn test_read_graph_from_file() {
        let graphs: Vec<BusGraph> = read_graph_from_file(NO_NODE_WEIGHT_PATH);
        assert_eq!(graphs.len(), 12);
    }

    #[test]
    fn test_total_rotation_count() {
        let graphs: Vec<BusGraph> = read_graph_from_file(NO_NODE_WEIGHT_PATH);
        let trip_ids = soc_aware_rotation_plan(&graphs[0], 1.0.into(), None);
        let count = total_rotation_count(&trip_ids, &graphs[0]);
        assert_eq!(count, 62);
    }

    #[test]
    fn test_schedule_no_node_weight() {
        let graphs: Vec<BusGraph> = read_graph_from_file(NO_NODE_WEIGHT_PATH);

        let mut number_of_schedules = Vec::new();

        for graph in &graphs {
            let trip_ids = soc_aware_rotation_plan(graph, None, None);

            // The "None" node weights in the input file should be converted to zeroes. Therefore, all values should be zeroes, that's what we're checking for here.
            validate_sum_of_node_weights(&trip_ids, graph, 0.0.into(), None);

            let count = total_rotation_count(&trip_ids, graph);
            number_of_schedules.push(count);
        }

        let number_of_edges = [62, 20, 5, 3, 3, 2, 2, 1, 1, 1, 1, 1];
        assert_eq!(number_of_schedules, Vec::from(&number_of_edges[..]));
    }

    #[test]
    fn test_schedule_one_node_weight() {
        let graphs: Vec<BusGraph> = read_graph_from_file(ONE_NODE_WEIGHT_PATH);
        let trip_ids = soc_aware_rotation_plan(&graphs[0], 1.0.into(), None);

        validate_sum_of_node_weights(&trip_ids, &graphs[0], 1.0.into(), None);

        let count = total_rotation_count(&trip_ids, &graphs[0]);
        assert_eq!(count, 270);
    }

    #[test]
    fn test_schedule_both_node_weights() {
        let graphs: Vec<BusGraph> = read_graph_from_file(BOTH_NODE_WEIGHTS_PATH);

        let mut number_of_schedules = Vec::new();

        for graph in &graphs {
            let trip_ids = soc_aware_rotation_plan(graph, 1.0.into(), 86400.into());

            validate_sum_of_node_weights(&trip_ids, graph, 1.0.into(), 86400.into());

            let count = total_rotation_count(&trip_ids, graph);
            number_of_schedules.push(count);
        }

        let number_of_edges = [280, 103, 21, 12, 12, 14, 5, 4, 1, 1, 1, 1];
        assert_eq!(number_of_schedules, number_of_edges);
    }
}
