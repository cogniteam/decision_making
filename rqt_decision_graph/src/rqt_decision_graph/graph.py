from __future__ import division

import pydot


class Graph(object):

    @staticmethod
    def __read_dot_data_from_file(path):
        try:
            with open(path, 'r') as file_handler:
                return file_handler.read()
        except IOError:
            return None

    def __init__(self, dot_to_qt, source, parent=None):
        super(Graph, self).__init__()
        self.dot_to_qt = dot_to_qt
        self.parent = parent
        self.name = None
        self.nodes = None
        self.edges = None
        self.source = source

    def load(self):
        self._create_dot_from_file(self.source)

    def _create_dot_from_file(self, dot_file_path):
        dot_data = self.__read_dot_data_from_file(dot_file_path)
        if dot_data is None:
            raise GraphParseException("Could not read dot file")

        graph = pydot.graph_from_dot_data(dot_data)
        if graph is None:
            raise GraphParseException("Could not create graph based on loaded dot data")

        dot_graph = graph.create_dot()
        if dot_graph is None:
            raise GraphParseException("Could not create DOT graph based on graph")

        self.nodes, self.edges = self.dot_to_qt.dotcode_to_qt_items(dot_graph)


class GraphParseException(Exception):
    pass