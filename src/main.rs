use crate::lib::soc_aware_rotation_plan;
use crate::lib::{read_graph_from_file, BusGraph};

mod lib;

fn main() {
    env_logger::init();

    const INPUT_FILE: &str = "/tmp/graph.json";

    let graphs: Vec<BusGraph> = read_graph_from_file(INPUT_FILE);
    let trip_ids = soc_aware_rotation_plan(&graphs[0], 1.0.into(), (4 * 3600).into());
    for trip_id in trip_ids {
        println!("{} -> {}", trip_id.0, trip_id.1);
    }
}
