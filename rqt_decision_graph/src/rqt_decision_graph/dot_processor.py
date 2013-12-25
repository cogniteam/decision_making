from __future__ import division

import pydot

POINTS_PER_INCH = 72


class DotProcessor(object):
    def __init__(self, dot_to_qt):
        super(DotProcessor, self).__init__()

        self._dot_to_qt = dot_to_qt

    def process(self, dot_data):
        graphs = []
        graphs_nodes_and_edges = {}
        cluster_nodes = []

        self._process(dot_data, graphs, graphs_nodes_and_edges, cluster_nodes)

        return self._get_qt_elements(graphs, graphs_nodes_and_edges, cluster_nodes)

    def _is_cluster(self, node_name, cluster_nodes):
        for node in cluster_nodes:
            if node.get_name() == node_name:
                return True

        return False

    def _find_parent_graph_node(self, graph_name, graphs_nodes_and_edges):

        for graph in graphs_nodes_and_edges:
            for graph_node in graphs_nodes_and_edges[graph][0]:
                if graph_node.get_name() == graph_name:
                    return graph_node

        return None

    def _extract_positon(self, graph_object):
        position = graph_object.get_pos().strip('"').split(',')

        return float(position[0]), float(position[1])

    def _extract_node_size(self, node):
        width = node.get_width().strip('"')
        height = node.get_height().strip('"')

        return float(width), float(height)

    def _align_edge_positions(self, edge, horizontal_offset, vertical_offset):
        position = edge.get_pos()[3:].strip('"').split(' ')

        result = '"e,'

        for pair in position:
            x, y = pair.replace('\\', '').split(',')
            result += '{0},{1} '.format(horizontal_offset + float(x),
                                        vertical_offset + float(y))
        result += '"'

        edge.set_pos(result)

    def _align_edge_label(self, edge, horizontal_offset, vertical_offset):
        label_pos = edge.get_lp().strip('"').split(',')

        edge.set_lp('\"{0},{1}\"'.format(horizontal_offset + float(label_pos[0]),
                                         vertical_offset + float(label_pos[1])))

    def _align_edge_to_parent(self, edge, parent_x, parent_y, parent_width, parent_height):
        horizontal_offset = parent_x - parent_width * POINTS_PER_INCH / 2
        vertical_offset = parent_y - parent_height * POINTS_PER_INCH / 2

        self._align_edge_positions(edge, horizontal_offset, vertical_offset)

        if edge.get_label() is not None:
            self._align_edge_label(edge, horizontal_offset, vertical_offset)

    def _align_node_to_parent(self, node, parent_x, parent_y, parent_width, parent_height):
        horizontal_offset = parent_x - parent_width * POINTS_PER_INCH / 2
        vertical_offset = parent_y - parent_height * POINTS_PER_INCH / 2

        x, y = self._extract_positon(node)
        node.set_pos('\"{0},{1}\"'.format(horizontal_offset + x, vertical_offset + y))

    def _get_qt_elements(self, graphs, graphs_nodes_and_edges, cluster_nodes):
        nodes, edges = {}, {}

        for graph in graphs:
            if self._is_cluster(graph, cluster_nodes):
                parent_node = self._find_parent_graph_node(graph, graphs_nodes_and_edges)

                parent_x, parent_y = self._extract_positon(parent_node)
                parent_width, parent_height = self._extract_node_size(parent_node)

                cluster_node_nodes = graphs_nodes_and_edges[graph][0]

                for node in cluster_node_nodes:
                    self._align_node_to_parent(node, parent_x, parent_y, parent_width, parent_height)

                    if self._is_cluster(node.get_name(), cluster_nodes):
                        nodes[node.get_name()] = self._dot_to_qt.get_cluster_node(node)
                    else:
                        nodes[node.get_name()] = self._dot_to_qt.get_node_item_for_node(node)
            else:
                graph_nodes = graphs_nodes_and_edges[graph][0]

                for node in graph_nodes:
                    nodes[node.get_name()] = self._dot_to_qt.get_cluster_node(node)

        for graph in graphs:
            if self._is_cluster(graph, cluster_nodes):
                parent_node = self._find_parent_graph_node(graph, graphs_nodes_and_edges)
                cluster_node_edges = graphs_nodes_and_edges[graph][1]

                parent_x, parent_y = self._extract_positon(parent_node)
                parent_width, parent_height = self._extract_node_size(parent_node)

                for edge in cluster_node_edges:
                    self._align_edge_to_parent(edge, parent_x, parent_y, parent_width, parent_height)
                    self._dot_to_qt.add_edge_item_for_edge(edge, nodes, edges)
            else:
                cluster_node_edges = graphs_nodes_and_edges[graph][1]
                for edge in cluster_node_edges:
                    self._dot_to_qt.add_edge_item_for_edge(edge, nodes, edges)

        return nodes, edges

    def _process(self, dot_data, graphs, graphs_nodes_and_edges, cluster_nodes):

        graph = pydot.graph_from_dot_data(dot_data)
        graphs.append(graph.get_name())

        for sub_graph in graph.get_subgraph_list():
            digraph, label = self._create_digraph(sub_graph)
            digraph_dot = digraph.create_dot()

            bounding_box = self._process(digraph_dot, graphs, graphs_nodes_and_edges, cluster_nodes)
            width, height = self._calculate_bounding_box_size(bounding_box)

            node = self._create_node(digraph.get_name(), label, str(width), str(height))

            cluster_nodes.append(node)
            graph.add_node(node)

        graph, _ = self._create_digraph(graph, include_sub_graphs=False, translate_edge=True)
        graph_dot = graph.create_dot()
        graph = pydot.graph_from_dot_data(graph_dot)

        self._map_graph_to_nodes_and_edges(graphs_nodes_and_edges,
                                           graph,
                                           graph.get_node_list(),
                                           graph.get_edge_list())

        return graph.get_bb()

    def _map_graph_to_nodes_and_edges(self, graphs_nodes_and_edges, graph, nodes, edges):
        graphs_nodes_and_edges[graph.get_name()] = ([], [])

        for n in nodes:
            if n.get_name() not in ('graph', 'node', 'empty'):
                graphs_nodes_and_edges[graph.get_name()][0].append(n)

        for e in edges:
            graphs_nodes_and_edges[graph.get_name()][1].append(e)

    def _calculate_bounding_box_size(self, bounding_box):
        bb = bounding_box.strip('"').split(',')

        width = float(bb[2]) / POINTS_PER_INCH
        height = float(bb[3]) / POINTS_PER_INCH

        return width, height

    def _create_node(self, name, label, width, height):
        return pydot.Node(name=name, label=label, shape='box', width=width, height=height, fixedsize='true')

    def _create_digraph(self, graph, include_sub_graphs=True, translate_edge=False):
        digraph = pydot.Dot(graph_type='digraph',
                            graph_name=graph.get_name())
        label = None
        for node in graph.get_node_list():
            if node.get_name() in ('node', 'empty'):
                continue
            elif node.get_name() == 'graph':
                label = node.obj_dict['attributes'].get('label', ' ')
            else:
                digraph.add_node(node)

        for edge in graph.get_edge_list():
            if translate_edge:
                source = edge.get_ltail() if edge.get_ltail() is not None else edge.get_source()
                destination = edge.get_lhead() if edge.get_lhead() is not None else edge.get_destination()
                edge_label = edge.get_label() if edge.get_label() is not None else ''

                digraph.add_edge(pydot.Edge(source, destination, label=edge_label))
            else:
                digraph.add_edge(edge)

        if include_sub_graphs:
            for sub_graph in graph.get_subgraph_list():
                digraph.add_subgraph(sub_graph)

        if label is not None:
            digraph.set_label(label)
            digraph.set_labelloc("t")

        digraph_dot = digraph.create_dot()
        digraph = pydot.graph_from_dot_data(digraph_dot)

        return digraph, label

