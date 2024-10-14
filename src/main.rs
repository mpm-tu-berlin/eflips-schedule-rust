use crate::lib::soc_aware_rotation_plan;
use crate::lib::{read_graph_from_file, total_rotation_count, BusGraph};

mod lib;

fn main() {
    env_logger::init();

    const NO_NODE_WEIGHT_PATH: &str = "test/no_node_weights.json";
    const ONE_NODE_WEIGHT_PATH: &str = "test/one_node_weight.json";
    const BOTH_NODE_WEIGHTS_PATH: &str = "test/both_node_weights.json";

    let graphs: Vec<BusGraph> = read_graph_from_file(NO_NODE_WEIGHT_PATH);
    let trip_ids = soc_aware_rotation_plan(&graphs[0]);
    let count = total_rotation_count(&trip_ids, &graphs[0]);
    assert_eq!(count, 62);
}
