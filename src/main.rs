use crate::lib::read_graph_from_file;
use crate::lib::soc_aware_rotation_plan;

mod lib;

const INPUT_FILE: &str = "graph_red.json";
fn main() {
    env_logger::init();

    let bus_graph = read_graph_from_file(INPUT_FILE);
    if bus_graph.len() != 1 {
        panic!("Expected exactly one graph in the input file");
    }
    let single_graph = &bus_graph[0];

    let edges = soc_aware_rotation_plan(single_graph);
    // Write to a JSON
    let output_file = "graph_out.json";
    let output = std::fs::File::create(output_file).unwrap();
    serde_json::to_writer(&output, &edges).unwrap();
}
