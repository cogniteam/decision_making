"""
Copyright (c) 2013, Cogniteam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

*   Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

*   Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

*   Neither the name of the Cogniteam nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from __future__ import division
from python_qt_binding.QtGui import QGraphicsTextItem
from pydot import *

POINTS_PER_INCH = 72


class DotProcessor(object):
    def __init__(self, dot_to_qt):
        super(DotProcessor, self).__init__()

        self._dot_to_qt = dot_to_qt

    def process(self, dot_data):
        graphs = []
        graphs_nodes_and_edges = {}
        cluster_nodes = []

        self._map_dot_graph(dot_data, graphs, graphs_nodes_and_edges, cluster_nodes)

        return self._get_qt_elements(graphs, graphs_nodes_and_edges, cluster_nodes)

    def _is_cluster(self, node_name, cluster_nodes):
        return next((True for node in cluster_nodes if node.get_name() == node_name), False)

    def _find_parent_graph_node(self, graph_name, graphs_nodes_and_edges):
        for graph in graphs_nodes_and_edges:
            graph_nodes = graphs_nodes_and_edges[graph][0]

            graph_node = next((node for node in graph_nodes if node.get_name() == graph_name), None)

            if graph_node is not None:
                return graph_node

        return None

    def _extract_positon(self, graph_object):
        return [float(value) for value in graph_object.get_pos().strip('\"').split(',')]

    def _extract_node_size(self, node):
        width = node.get_width().strip('\"')
        height = node.get_height().strip('\"')

        return float(width), float(height)

    def _align_edge_positions(self, edge, horizontal_offset, vertical_offset):
        position = edge.get_pos()[3:].strip('\"').split(' ')

        result = '\"e,'
        for pair in position:
            x, y = [float(value) for value in pair.replace('\\', '').split(',')]
            result += '{0},{1} '.format(horizontal_offset + x, vertical_offset + y)
        result += '\"'

        edge.set_pos(result)

    def _align_edge_label(self, edge, horizontal_offset, vertical_offset):
        label_pos_x, label_pos_y = [float(value) for value in edge.get_lp().strip('\"').split(',')]

        edge.set_lp('\"{0},{1}\"'.format(horizontal_offset + label_pos_x, vertical_offset + label_pos_y))

    def _align_edge_to_parent(self, edge, horizontal_offset, vertical_offset):
        self._align_edge_positions(edge, horizontal_offset, vertical_offset)

        if edge.get_label() is not None:
            self._align_edge_label(edge, horizontal_offset, vertical_offset)

    def _align_node_to_parent(self, node, horizontal_offset, vertical_offset):

        x, y = self._extract_positon(node)
        node.set_pos('\"{0},{1}\"'.format(horizontal_offset + x, vertical_offset + y))

    def _get_offsets(self, target_node):
        parent_x, parent_y = self._extract_positon(target_node)
        parent_width, parent_height = self._extract_node_size(target_node)

        # magic number detected
        horizontal_offset = parent_x - parent_width * POINTS_PER_INCH / 2 + 8
        vertical_offset = parent_y - parent_height * POINTS_PER_INCH / 2 + 8

        return horizontal_offset, vertical_offset

    def _get_qt_nodes(self, graphs, graphs_nodes_and_edges, cluster_nodes, nodes):
        for graph in graphs:
            if self._is_cluster(graph, cluster_nodes):
                parent_node = self._find_parent_graph_node(graph, graphs_nodes_and_edges)
                horizontal_offset, vertical_offset = self._get_offsets(parent_node)

                cluster_node_nodes = graphs_nodes_and_edges[graph][0]

                for node in cluster_node_nodes:
                    self._align_node_to_parent(node, horizontal_offset, vertical_offset)

                    if self._is_cluster(node.get_name(), cluster_nodes):
                        nodes[node.get_name()] = self._dot_to_qt.get_cluster_node(node)
                    else:
                        nodes[node.get_name()] = self._dot_to_qt.get_node_item_for_node(node)
            else:
                graph_nodes = graphs_nodes_and_edges[graph][0]

                for node in graph_nodes:
                    if self._is_cluster(node.get_name(), cluster_nodes):
                        nodes[node.get_name()] = self._dot_to_qt.get_cluster_node(node)
                    else:
                        nodes[node.get_name()] = self._dot_to_qt.get_node_item_for_node(node)

    def _get_qt_edges(self, graphs, graphs_nodes_and_edges, cluster_nodes, nodes, edges):
        for graph in graphs:
            if self._is_cluster(graph, cluster_nodes):
                parent_node = self._find_parent_graph_node(graph, graphs_nodes_and_edges)
                horizontal_offset, vertical_offset = self._get_offsets(parent_node)

                cluster_node_edges = graphs_nodes_and_edges[graph][1]

                for edge in cluster_node_edges:
                    self._align_edge_to_parent(edge, horizontal_offset, vertical_offset)
                    self._dot_to_qt.add_edge_item_for_edge(edge, nodes, edges)
            else:
                cluster_node_edges = graphs_nodes_and_edges[graph][1]
                for edge in cluster_node_edges:
                    self._dot_to_qt.add_edge_item_for_edge(edge, nodes, edges)

    def _get_qt_elements(self, graphs, graphs_nodes_and_edges, cluster_nodes):
        nodes, edges = {}, {}

        self._get_qt_nodes(graphs, graphs_nodes_and_edges, cluster_nodes, nodes)
        self._get_qt_edges(graphs, graphs_nodes_and_edges, cluster_nodes, nodes, edges)

        return nodes, edges

    def _map_dot_graph(self, dot_data, graphs, graphs_nodes_and_edges, cluster_nodes):

        graph = graph_from_dot_data(dot_data)
        graphs.append(graph.get_name())

        for sub_graph in graph.get_subgraph_list():
            if not sub_graph.get_name():
                continue

            digraph, digraph_dot, self_edge = self._create_digraph(sub_graph)

            if self_edge:
                graph.add_edge(self_edge)

            bounding_box = self._map_dot_graph(digraph_dot, graphs, graphs_nodes_and_edges, cluster_nodes)
            width, height = self._calculate_size(bounding_box)

            width += 0.2
            height += 0.2

            node = self._create_fixed_size_node(digraph.get_name(), digraph.get_label(), str(width), str(height),
                                                digraph.get_URL())

            cluster_nodes.append(node)
            graph.add_node(node)

        if self._has_html_nodes(graph):
            graph = self._adjust_html_nodes_size(graph)

        processed_graph, processed_graph_dot, _ = self._create_digraph(graph,
                                                                       sub_graphs=False,
                                                                       translate_edge=True)

        self._map_nodes_and_edges_to_graph(graphs_nodes_and_edges,
                                           processed_graph,
                                           processed_graph.get_node_list(),
                                           processed_graph.get_edge_list())

        return processed_graph.get_bb()

    def _map_nodes_and_edges_to_graph(self, graphs_nodes_and_edges, graph, nodes, edges, include_nameless=True):
        if graph.get_name() not in graphs_nodes_and_edges:
            graphs_nodes_and_edges[graph.get_name()] = ([], [])

        graph_name = graph.get_name()
        graph_nodes = graphs_nodes_and_edges[graph_name][0]
        graph_edges = graphs_nodes_and_edges[graph_name][1]

        for node in nodes:
            if node.get_name() in ('graph', 'node', 'empty'):
                continue

            graph_nodes.append(node)

        for edge in edges:
            graph_edges.append(edge)

        if not include_nameless:
            return

        for sub_graph in graph.get_subgraph_list():
            if sub_graph.get_name():
                continue

            self._map_nodes_and_edges_to_graph(graphs_nodes_and_edges,
                                               graph,
                                               sub_graph.get_node_list(),
                                               sub_graph.get_edge_list(),
                                               include_nameless=False)

    def _calculate_size(self, bounding_box):
        return [float(value) / POINTS_PER_INCH for value in bounding_box.strip('\"').split(',')[-2:]]

    def _create_fixed_size_node(self, name, label, width, height, url):
        return Node(name=name, label=label, shape='box', width=width, height=height, fixedsize='true', URL=url)

    def _is_html_node(self, node):
        if node.get_name() in ('graph', 'node', 'empty'):
            return False

        label = node.get_label()

        if not label:
            return False

        label = label.strip('\"')

        if label.startswith('<') and label.endswith('>'):
            return True

        return True

    def _has_html_nodes(self, graph):
        for node in graph.get_node_list():
            if self._is_html_node(node):
                return True

        for sub_graph in graph.get_subgraph_list():
            if self._is_html_node(sub_graph):
                return True

        return False

    def _adjust_html_node_size(self, node):
        if node.get_name() in ('graph', 'node', 'empty'):
            return node

        label = node.get_label()

        if not label:
            return node

        label = label.strip('\"')

        if not label.startswith('<') and not label.endswith('>'):
            return node

        text = QGraphicsTextItem()
        text.setHtml(label)

        bounding_box = text.boundingRect()
        width = bounding_box.width() / POINTS_PER_INCH
        height = bounding_box.height() / POINTS_PER_INCH

        url = node.get_attributes().get('URL', None)

        return self._create_fixed_size_node(node.get_name(), label, str(width), str(height), url)

    def _adjust_html_nodes_size(self, graph):
        digraph = Dot(graph_name=graph.get_name(), graph_type='digraph')

        for node in graph.get_node_list():
            digraph.add_node(self._adjust_html_node_size(node))

        for edge in graph.get_edge_list():
            digraph.add_edge(edge)

        for sub_graph in graph.get_subgraph_list():
            if sub_graph.get_name() == '':
                fixed_sub_graph = Subgraph()
                for node in sub_graph.get_node_list():
                    fixed_sub_graph.add_node(self._adjust_html_node_size(node))

                digraph.add_subgraph(fixed_sub_graph)
            else:
                digraph.add_subgraph(sub_graph)

        return graph_from_dot_data(digraph.create_dot())

    def _translate_edge_source(self, edge):
        source = edge.get_ltail()

        if not source:
            source = edge.get_source()

        return source

    def _translate_edge_destination(self, edge):
        destination = edge.get_lhead()

        if not destination:
            destination = edge.get_destination()

        return destination

    def _is_parent_graph_node(self, source, destination, parent_name):
        return source == destination and destination == parent_name

    def _create_digraph(self, graph, sub_graphs=True, translate_edge=False):
        parent_edge = None
        digraph = Dot(graph_name=graph.get_name(), graph_type='digraph')
        digraph.set_node_defaults(shape='box')
        label = None
        url = None
        for node in graph.get_node_list():
            if node.get_name() in ('node', 'empty'):
                continue
            elif node.get_name() == 'graph':
                label = node.get_attributes().get('label', ' ')
                url = node.get_attributes().get('URL', None)
            else:
                digraph.add_node(node)

        for edge in graph.get_edge_list():
            source = self._translate_edge_source(edge)
            destination = self._translate_edge_destination(edge)
            edge_label = edge.get_label()

            if translate_edge:
                if self._is_parent_graph_node(source, destination, graph.get_name()):
                    continue

                translated_edge = Edge(source, destination)
                if edge_label:
                    translated_edge.set_label(edge_label)

                digraph.add_edge(translated_edge)

            else:
                if self._is_parent_graph_node(source, destination, graph.get_name()):
                    parent_edge = Edge(source, destination)
                    if edge_label:
                        parent_edge.set_label(edge_label)
                    continue
                digraph.add_edge(edge)

        for sub_graph in graph.get_subgraph_list():
            if sub_graphs or sub_graph.get_name() == '':
                digraph.add_subgraph(sub_graph)

        if label:
            digraph.set_label(label)
            digraph.set_labelloc('t')

        if url:
            digraph.set_URL(url)

        digraph_dot = digraph.create_dot()

        return graph_from_dot_data(digraph_dot), digraph_dot, parent_edge

