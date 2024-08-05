import json
import os.path
import unittest
from eflips_schedule_rust import rotation_plan
import networkx as nx


class MyTestCase(unittest.TestCase):
    def test_non_soc_aware(self):
        path_to_this_file = __file__
        graph_file = os.path.join(os.path.dirname(path_to_this_file), 'graph.json')
        with open(graph_file, 'r') as f:
            graph = json.load(f)
        result = rotation_plan(json.dumps(graph), soc_aware = False)

        # The result is a list of edges, each edge is a list of two nodes
        # Assemble a networkx graph from the result
        G = nx.Graph()
        for edge in result:
            G.add_edge(edge[0], edge[1])

        # Print the number of connected components
        assert nx.number_connected_components(G) == 280


    def test_soc_aware(self):
        path_to_this_file = __file__
        graph_file = os.path.join(os.path.dirname(path_to_this_file), 'graph.json')
        with open(graph_file, 'r') as f:
            graph = json.load(f)
        result = rotation_plan(json.dumps(graph), soc_aware = True)

        # The result is a list of edges, each edge is a list of two nodes
        # Assemble a networkx graph from the result
        G = nx.Graph()
        for edge in result:
            G.add_edge(edge[0], edge[1])

        # Print the number of connected components
        assert nx.number_connected_components(G) == 330

if __name__ == '__main__':
    unittest.main()
