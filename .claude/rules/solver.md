---
paths:
  - "src/**/*.rs"
---

# Solver internals (src/lib.rs)

- `BusGraph` wraps a petgraph `StableDiGraph`. It uses `StableGraph` because nodes are removed during limit enforcement, and indices must stay valid. Null node weights become 0. The input must be a DAG, because `toposort` panics otherwise.
- Matching (`to_bipartite` + `maximum_matching`) splits each trip into top and bottom nodes. Each component is solved with Kuhn-Munkres to minimize waiting time, with missing edges costing `NO_EDGE_WEIGHT`. If the optimum needs a missing edge, the code falls back to Hopcroft-Karp on weight-sorted edges, which gives maximum cardinality but not optimal waiting time.
- Limit enforcement (`soc_aware_rotation_plan`) is a greedy loop. While a rotation exceeds a limit, it scores cutting that rotation's longest feasible prefix or suffix. The score is `total_rotations * 1e9 + excessive_rotations` after re-matching, and candidates are scored in parallel with rayon. The best cut is removed from the graph permanently and appended to the output as a fixed rotation.
- A rotation's duration is the sum of its trip durations plus the waiting times on the edges between consecutive trips. Its `delta_soc` is the sum of its node weights.
