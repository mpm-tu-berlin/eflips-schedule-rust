# eflips-schedule-rust

---

Part of the [eFLIPS/simBA](https://github.com/stars/ludgerheide/lists/ebus2030) list of projects.

---


eflips-schedule-rust provides speedups (100x-1000x) to solving the vehicle scheduling problem for electric buses. It is 
a Rust library that can be used as a Python extension.

## Installation

1. The project is available on [PyPI](https://pypi.org/project/eflips-schedule-rust/). You can install it via pip:

```bash
pip install eflips-schedule-rust
```

## Usage

### General

The vehicle scheduling problem needs to be formulated as a Directed Acyclic Graph (DAG) with the following properties:

- Nodes are trips. Each trip may have a Tuple of two weights. The weights indicate various cost properties of 
the trip and range from 0 to 1 (float). One example would be to have each first weight of the tuple represent the energy
consumption of the trip, as fraction of the battery capacity. The second weight could represent the time of the trip as
a fraction of 24 hours. The solver can then create schedules that exceed neither the battery capacity nor the time.
- Edges are connections between trips. Each edge has a single weight that represents the cost of the connection. This
should normally be the waiting time between the trips. However, things such as "try to connect on the same line" can 
also be expressed by scaling the weights. The allowed weights are between 0 and 1e6 (integer).
- Connections that cannot be made should be represented as not existing, not by a very high weight.
- The result is a list of edges, each representing a connection between two trips. By connecting all the nodes using *only* the edges in the result, a valid schedule is created.

### Function call and JSON data format

The function call is as follows:

```python
import eflips_schedule_rust
edgelist = eflips_schedule_rust.solve(data)
```

The `data` parameter is a JSON string that contains the graph. It has the following structure:

1. Outermost is a list of subgraphs. If the main graph is composed of disconnected components, each component should be a subgraph.
2. Each subgraph is a dict of two keys: "nodes" and "edges".
    a. "nodes" is a list of nodes. Each node is a dict with the following keys:
        - "id": a unique identifier for the node. This is used to reference the node in the edges.
        - "weights": a tuple of two floats. The first weight is the energy consumption of the trip, the second weight is the time of the trip.
    b. "edges" is a list of edges. Each edge is a dict with the following keys:
        - "source": the id of the source node.
        - "target": the id of the target node.
        - "weight": a float that represents the cost of the connection.

Here is an example of a JSON string that represents a graph with two nodes and one edge:

```json
[
    {
        "nodes": [
            {
                "id": 0,
                "weights": [0.5, 0.5]
            },
            {
                "id": 1,
                "weights": [0.5, 0.5]
            }
        ],
        "edges": [
            {
                "source": 0,
                "target": 1,
                "weight": 1
            }
        ]
    }
]
```

## Testing

*Unit tests* use rust's `cargo test` and can be run using `cargo test`. The tests are located in the `src/lib.rs` file.

*Integration tests* use Python's `unittest` and can be run using `python -m unittest`. The tests are located in the `tests/test.py` file.

## Documentation

Documentation is inline accordint to Python (Docstring) and Rust (//!) conventions.


## Development

We utilize the [GitHub Flow](https://docs.github.com/get-started/quickstart/github-flow) branching structure. This means
that the `main` branch is always deployable and that all development happens in feature branches. The feature branches
are merged into `main` via pull requests.

We use [black](https://black.readthedocs.io/en/stable/) for python code formatting and `rustfmt` for rust. ng.

## License

This project is licensed under the AGPLv3 license - see the [LICENSE](LICENSE.md) file for details.

## Funding Notice

This code was developed as part of the project [eBus2030+]([https://www.eflip.de/](https://www.now-gmbh.de/projektfinder/e-bus-2030/)) funded by the Federal German Ministry for Digital and Transport (BMDV) under grant number 03EMF0402.
