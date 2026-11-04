import json
import os.path
import unittest
from eflips_schedule_rust import solve
import networkx as nx


class TheTestCase(unittest.TestCase):
    def test_non_soc_aware(self):
        path_to_this_file = __file__
        graph_file = os.path.join(
            os.path.dirname(path_to_this_file), "no_node_weights.json"
        )
        with open(graph_file, "r") as f:
            graphs = json.load(f)

        number_of_schedules = []

        # graphs is a list of dictionaries, each dictionary has "nodes" and "edges" keys
        for graph in graphs:
            result = solve(json.dumps([graph]), max_delta_soc=None, max_duration=None)

            # The result is a list of edges, each edge is a list of two nodes
            # Assemble a networkx graph from the result
            G = nx.Graph()
            for node in graph["nodes"]:
                G.add_node(node["id"])
            for edge in result:
                G.add_edge(edge[0], edge[1])
            number_of_schedules.append(nx.number_connected_components(G))

        # Print the number of connected components
        self.assertEqual(number_of_schedules, [62, 20, 5, 3, 3, 2, 2, 1, 1, 1, 1, 1])

    def test_soc_aware(self):
        path_to_this_file = __file__
        graph_file = os.path.join(
            os.path.dirname(path_to_this_file), "both_node_weights.json"
        )
        with open(graph_file, "r") as f:
            graphs = json.load(f)
        result = solve(json.dumps(graphs), max_delta_soc=1.0, max_duration=None)

        number_of_schedules = []

        # graphs is a list of dictionaries, each dictionary has "nodes" and "edges" keys
        for graph in graphs:
            result = solve(json.dumps([graph]), max_delta_soc=1.0, max_duration=None)

            # The result is a list of edges, each edge is a list of two nodes
            # Assemble a networkx graph from the result
            G = nx.Graph()
            for node in graph["nodes"]:
                G.add_node(node["id"])
            for edge in result:
                G.add_edge(edge[0], edge[1])
            number_of_schedules.append(nx.number_connected_components(G))

        # Print the number of connected components
        self.assertEqual(number_of_schedules, [142, 64, 21, 11, 9, 5, 5, 2, 1, 1, 1, 1])

    def test_edge_delta_soc(self):
        # Three trips of 0.3 each fit into one rotation, unless a connection consumes 0.5 itself
        nodes = [{"id": i, "weight": [0.3, None]} for i in (1, 2, 3)]
        edges = [
            {"source": 1, "target": 2, "weight": 60},
            {"source": 2, "target": 3, "weight": 60},
        ]
        result = solve(json.dumps([{"nodes": nodes, "edges": edges}]), max_delta_soc=1.0, max_duration=None)
        self.assertEqual(sorted(result), [(1, 2), (2, 3)])

        edges[1]["delta_soc"] = 0.5
        result = solve(json.dumps([{"nodes": nodes, "edges": edges}]), max_delta_soc=1.0, max_duration=None)
        self.assertNotIn((2, 3), result)
        self.assertEqual(len(result), 1)


if __name__ == "__main__":
    unittest.main()
