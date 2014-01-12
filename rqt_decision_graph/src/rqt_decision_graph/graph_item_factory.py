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

from .edge_item import EdgeItem
from .node_item import NodeItem
from .graph_item import GraphItem


class GraphItemFactory(object):

    def __init__(self):
        super(GraphItemFactory, self).__init__()
        self._color = GraphItem.COLOR
        self._pen_width = GraphItem.PEN_WIDTH
        self._edge_pen_width = EdgeItem.EDGE_PEN_WIDTH
        self._label_pen_width = GraphItem.LABEL_PEN_WIDTH
        self._same_label_siblings = False

    def set_color(self, color):
        self._color = color

    def color(self):
        return self._color()

    def set_pen_width(self, pen_width):
        self._pen_width = pen_width

    def pen_width(self):
        return self._pen_width

    def set_edge_pen_width(self, edge_pen_width):
        self._edge_pen_width = edge_pen_width

    def edge_pen_width(self):
        return self._edge_pen_width

    def set_label_pen_width(self, label_pen_width):
        self._label_pen_width = label_pen_width

    def label_pen_width(self):
        return self._label_pen_width

    def set_same_label_siblings(self, same_label_siblings):
        self._same_label_siblings = same_label_siblings

    def same_label_siblings(self):
        return self._same_label_siblings

    def create_edge(self, spline, label, label_center, from_node, to_node, parent=None, **kwargs):
        self._check_constraints(kwargs, False)
        return EdgeItem(spline, label, label_center, from_node, to_node, parent, **kwargs)

    def create_node(self, bounding_box, shape, label, label_pos=None, url=None, parent=None, cluster=False, **kwargs):
        self._check_constraints(kwargs)
        return NodeItem(bounding_box, shape, label, label_pos, url, parent, **kwargs)

    def _check_constraints(self, dictionary, node=True):
        if 'penwidth' not in dictionary:
            dictionary['penwidth'] = self._pen_width

        if not node:
            if 'edge_pen_width' not in dictionary:
                dictionary['edge_pen_width'] = self._edge_pen_width

        if 'label_pen_width' not in dictionary:
            dictionary['label_pen_width'] = self._label_pen_width

        if 'color' not in dictionary:
            dictionary['color'] = self._color
