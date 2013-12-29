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

from python_qt_binding.QtCore import QRectF
from python_qt_binding.QtGui import QColor
from .graph_item_factory import GraphItemFactory
from .hovered_edge_item import HoveredEdgeItem
from .hovered_node_item import HoveredNodeItem


class HoveredGraphItemFactory(GraphItemFactory):

    def __init__(self):
        super(HoveredGraphItemFactory, self).__init__()

        self._highlight_level = 1
        self._hovered_color = QColor(250, 0, 0)

    def set_highlight_level(self, level):
        self._highlight_level = level

    def highlight_level(self):
        return self._highlight_level

    def set_hovered_color(self, hovered_color):
        self._hovered_color = hovered_color

    def hovered_color(self):
        return self._hovered_color

    def create_edge(self, spline, label, label_center, from_node, to_node, parent=None, **kwargs):
        self._check_constraints(kwargs, False)
        return HoveredEdgeItem(spline, label, label_center, from_node, to_node, parent, **kwargs)

    def create_node(self, bounding_box, shape, label, label_pos=None, url=None, parent=None, cluster=False, **kwargs):
        self._check_constraints(kwargs)
        node_item = HoveredNodeItem(bounding_box, shape, label, label_pos, url, parent, **kwargs)

        if cluster:
            bounding_box = QRectF(bounding_box)
            bounding_box.setHeight(30)
            node_item.set_hover_shape(bounding_box)

        return node_item

    def _check_constraints(self, dictionary, node=True):
        super(HoveredGraphItemFactory, self)._check_constraints(dictionary, node)

        if 'highlight_level' not in dictionary:
            dictionary['highlight_level'] = self._highlight_level

        if 'hovered_color' not in dictionary:
            dictionary['hovered_color'] = self._hovered_color
