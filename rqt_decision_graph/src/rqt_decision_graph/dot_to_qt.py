"""
Software License Agreement (BSD License)

Copyright (c) 2008, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

*  Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
*  Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
*  Neither the name of Willow Garage, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE

Note: This is modified version by Cogniteam
"""

from python_qt_binding.QtCore import QPointF, QRectF
from python_qt_binding.QtGui import QColor
from pydot import *

POINTS_PER_INCH = 72


# hack required by pydot
def get_unquoted(item, name):
    value = item.get(name)
    if value is None:
        return None
    try:
        return value.strip('"\n"')
    except AttributeError:
        # not part of the string family
        return value


# hack required to show properly long labels of custom shapes
def clean_line_separator(attribute):
    return attribute.replace('\\', '')

# approximately, for workarounds (TODO: get this from dotfile somehow)
LABEL_HEIGHT = 30


# Class generating Qt Elements from doctcode
class DotToQtGenerator():

    def __init__(self, factory):
        self._factory = factory

    def get_node_item_for_node(self, node):
        """
        returns a pyqt NodeItem object, or None in case of error or invisible style
        """
        # let pydot imitate pygraphviz api
        attr = {}
        for name in node.get_attributes().iterkeys():
            value = get_unquoted(node, name)
            attr[name] = value
        obj_dic = node.__getattribute__("obj_dict")
        for name in obj_dic:
            if name not in ['attributes', 'parent_graph'] and obj_dic[name] is not None:
                attr[name] = get_unquoted(obj_dic, name)
        node.attr = attr

        if 'style' in node.attr:
            if node.attr['style'] == 'invis':
                return None

        color = QColor(node.attr['color']) if 'color' in node.attr else None
        name = None
        if 'label' in node.attr:
            name = node.attr['label']
        elif 'name' in node.attr:
            name = node.attr['name']
        else:
            print("Error, no label defined for node with attr: %s" % node.attr)
            return None

        if name is None:
            # happens on Lucid pygraphviz version
            print("Error, label is None for node %s, pygraphviz version may be too old." % node)
        else:
            name = name.decode('string_escape')

        # decrease rect by one so that edges do not reach inside
        bb_width = len(name) / 5
        if 'width' in node.attr:
            bb_width = node.attr['width']

        bb_height = 1.0
        if 'width' in node.attr:
            bb_height = node.attr['height']

        pos = (0, 0)
        if 'pos' in node.attr:
            pos = node.attr['pos'].split(',')

        bounding_box = QRectF(0, 0, POINTS_PER_INCH * float(bb_width) - 1.0, POINTS_PER_INCH * float(bb_height) - 1.0)

        bounding_box.moveCenter(QPointF(float(pos[0]), -float(pos[1])))

        pen_width = self._factory.pen_width()
        if 'penwidth' in node.attr:
            pen_width = float(node.attr['penwidth'])

        url = node.attr['URL'] if 'URL' in node.attr else 'N/A'
        node_item = self._factory.create_node(bounding_box=bounding_box,
                                              shape=node.attr.get('shape', 'box'),
                                              label=name,
                                              url=url,
                                              penwidth=pen_width)
        return node_item

    def add_edge_item_for_edge(self, edge, nodes, edges):
        """
        adds EdgeItem by data in edge to edges
        :param same_label_siblings: if true, edges with same label will be considered siblings (collective highlighting)
        """
        # let pydot imitate pygraphviz api
        attr = {}
        for name in edge.get_attributes().iterkeys():
            value = get_unquoted(edge, name)
            attr[name] = value
        edge.attr = attr

        # cleans '\\\n' from positions, hack
        edge.attr['pos'] = clean_line_separator(edge.attr['pos'])

        if 'style' in edge.attr:
            if edge.attr['style'] == 'invis':
                return

        label = edge.attr.get('label', None)
        label_pos = edge.attr.get('lp', None)
        label_center = None
        if label_pos is not None:
            label_pos = label_pos.split(',')
            label_center = QPointF(float(label_pos[0]), -float(label_pos[1]))

        # try pydot, fallback for pygraphviz
        source_node = edge.get_source() if hasattr(edge, 'get_source') else edge[0]

        destination_node = str(edge.get_destination() if hasattr(edge, 'get_destination') else edge[1])

        # create edge with from-node and to-node
        edge_pos = (0, 0)
        if 'pos' in edge.attr:
            edge_pos = edge.attr['pos']
        if label is not None:
            label = label.decode('string_escape')

        edge_item = self._factory.create_edge(spline=edge_pos,
                                              label=label,
                                              label_center=label_center,
                                              from_node=nodes[source_node],
                                              to_node=nodes[destination_node])

        if self._factory.same_label_siblings():
            if label is None:
                # for sibling detection
                label = "%s_%s" % (source_node, destination_node)
            # symmetrically add all sibling edges with same label
            if label in edges:
                for sibling in edges[label]:
                    edge_item.add_sibling_edge(sibling)
                    sibling.add_sibling_edge(edge_item)

        if label not in edges:
            edges[label] = []
        edges[label].append(edge_item)

    def get_graph_edges(self, graph, nodes, edges):
        for subgraph in graph.get_subgraph_list():
            self.get_graph_edges(subgraph, nodes, edges)
            for edge in subgraph.get_edge_list():
                self.add_edge_item_for_edge(edge, nodes, edges)

        for edge in graph.get_edge_list():
            self.add_edge_item_for_edge(edge, nodes, edges)

    def get_cluster_node(self, node):
        # let pydot imitate pygraphviz api
        attr = {}
        for name in node.get_attributes().iterkeys():
            value = get_unquoted(node, name)
            attr[name] = value
        obj_dic = node.__getattribute__("obj_dict")
        for name in obj_dic:
            if name not in ['nodes', 'attributes', 'parent_graph'] and obj_dic[name] is not None:
                attr[name] = get_unquoted(obj_dic, name)
            elif name == 'nodes':
                for key in obj_dic['nodes']['graph'][0]['attributes']:
                    attr[key] = get_unquoted(obj_dic['nodes']['graph'][0]['attributes'], key)
        node.attr = attr

        name = None
        if 'label' in node.attr:
            name = node.attr['label']
        elif 'name' in node.attr:
            name = node.attr['name']
        else:
            print("Error, no label defined for node with attr: %s" % node.attr)
            return None
        if name is None:
            # happens on Lucid pygraphviz version
            print("Error, label is None for node %s, pygraphviz version may be too old." % node)

        bb_width = node.attr['width']
        bb_height = node.attr['height']

        bounding_box = QRectF(0, 0, POINTS_PER_INCH * float(bb_width) - 1.0, POINTS_PER_INCH * float(bb_height) - 1.0)
        # print bounding_box
        pos = (0, 0)
        if 'pos' in node.attr:
            pos = node.attr['pos'].split(',')
        bounding_box.moveCenter(QPointF(float(pos[0]), -float(pos[1])))

        label_pos = QPointF(bounding_box.center().x(), bounding_box.top() + LABEL_HEIGHT / 2)

        color = QColor(node.attr['color']) if 'color' in node.attr else None

        url = node.attr['URL'] if 'URL' in node.attr else 'N/A'
        label = node.attr['label'] if 'label' in node.attr else name

        graph_node_item = self._factory.create_node(bounding_box=bounding_box,
                                                    shape='box',
                                                    label=label,
                                                    label_pos=label_pos,
                                                    url=url,
                                                    cluster=True)

        return graph_node_item

    def get_subgraph_nodes(self, graph, nodes):
        # let pydot imitate pygraphviz api
        attr = {}
        for name in graph.get_attributes().iterkeys():
            value = get_unquoted(graph, name)
            attr[name] = value
        obj_dic = graph.__getattribute__("obj_dict")
        for name in obj_dic:
            if name not in ['nodes', 'attributes', 'parent_graph'] and obj_dic[name] is not None:
                attr[name] = get_unquoted(obj_dic, name)
            elif name == 'nodes':
                for key in obj_dic['nodes']['graph'][0]['attributes']:
                    attr[key] = get_unquoted(obj_dic['nodes']['graph'][0]['attributes'], key)
        graph.attr = attr

        bb = graph.attr['bb'].strip('"').split(',')
        if len(bb) < 4:
            # bounding box is empty
            return None
        bounding_box = QRectF(0, 0, float(bb[2]) - float(bb[0]), float(bb[3]) - float(bb[1]))
        if 'lp' in graph.attr:
            label_pos = graph.attr['lp'].strip('"').split(',')
        else:
            label_pos = (float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2,
                         float(bb[1]) + (float(bb[3]) - float(bb[1])) - LABEL_HEIGHT / 2)
        bounding_box.moveCenter(QPointF(float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2,
                                        -float(bb[1]) - (float(bb[3]) - float(bb[1])) / 2))
        name = graph.attr.get('label', '')

        color = QColor(graph.attr['color']) if 'color' in graph.attr else None

        url = graph.attr['URL'] if 'URL' in graph.attr else 'N/A'

        graph_node_item = self._factory.create_node(bounding_box=bounding_box,
                                                    shape='box',
                                                    label=name,
                                                    label_pos=QPointF(float(label_pos[0]), -float(label_pos[1])),
                                                    url=url,
                                                    cluster=True)

        for subgraph in graph.get_subgraph_list():
            subgraph_node_item = self.get_subgraph_nodes(subgraph, nodes)
            # skip subgraphs with empty bounding boxes
            if subgraph_node_item is None:
                continue

            nodes[subgraph.get_name()] = subgraph_node_item
            for node in subgraph.get_node_list():
                # hack required by pydot
                if node.get_name() in ('graph', 'node', 'empty'):
                    continue
                nodes[node.get_name()] = self.get_node_item_for_node(node)
        for node in graph.get_node_list():
            # hack required by pydot
            if node.get_name() in ('graph', 'node',  'empty'):
                continue
            nodes[node.get_name()] = self.get_node_item_for_node(node)

        return graph_node_item

    def get_graph_nodes(self, graph, nodes):

        for subgraph in graph.get_subgraph_list():
            subgraph_node_item = self.get_subgraph_nodes(subgraph, nodes)
            # skip subgraphs with empty bounding boxes
            if subgraph_node_item is None:
                continue

            nodes[subgraph.get_name()] = subgraph_node_item
            for node in subgraph.get_node_list():
                # hack required by pydot
                if node.get_name() in ('graph', 'node', 'empty'):
                    continue
                nodes[node.get_name()] = self.get_node_item_for_node(node)
        for node in graph.get_node_list():
            # hack required by pydot
            if node.get_name() in ('graph', 'node', 'empty'):
                continue
            nodes[node.get_name()] = self.get_node_item_for_node(node)

    def dotcode_to_qt_items(self, dotcode):
        """
        takes dotcode, runs layout, and creates qt items based on the dot layout.
        returns two dicts, one mapping node names to Node_Item, one mapping edge names to lists of Edge_Item
        :param same_label_siblings: if true, edges with same label will be considered siblings (collective highlighting)
        """
        # layout graph
        if dotcode is None:
            return {}, {}

        graph = graph_from_dot_data(dotcode.encode("ascii", "ignore"))

        nodes = {}
        edges = {}

        self.get_graph_nodes(graph, nodes)
        self.get_graph_edges(graph, nodes, edges)

        return nodes, edges