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

from python_qt_binding.QtGui import QColor
from python_qt_binding.QtCore import QRectF
from .dmg_node_item import DmgNodeItem
from .dmg_html_node_item import DmgHtmlNodeItem
from .dmg_edge_item import DmgEdgeItem
from .hovered_item_factory import HoveredGraphItemFactory


class DmgItemFactory(HoveredGraphItemFactory):
    def __init__(self):
        super(DmgItemFactory, self).__init__()
        self._highlighted_pen_width = 3.0
        self._highlighted_label_pen_width = 1.0
        self._highlighted_color = QColor(0, 150, 0)

    def set_highlighted_pen_width(self, highlighted_pen_width):
        self._highlighted_pen_width = highlighted_pen_width

    def highlighted_pen_width(self):
        return self._highlighted_pen_width

    def set_highlighted_label_pen_width(self, highlighted_label_pen_width):
        self._highlighted_label_pen_width = highlighted_label_pen_width

    def highlighted_label_pen_width(self):
        return self._highlighted_label_pen_width

    def set_highlighted_color(self, highlighted_color):
        self._highlighted_color = highlighted_color

    def highlighted_color(self):
        return self._highlighted_color

    def set_highlighted_color(self, highlighted_color):
        self._highlighted_color = highlighted_color

    def highlighted_color(self):
        return self._highlighted_color

    def create_edge(self, spline, label, label_center, from_node, to_node, parent=None, **kwargs):
        self._check_constraints(kwargs, False)
        return DmgEdgeItem(spline, label, label_center, from_node, to_node, parent, **kwargs)

    def create_node(self, bounding_box, shape, label, label_pos=None, url=None, parent=None, cluster=False, **kwargs):
        self._check_constraints(kwargs)

        if label.startswith('<') and label.endswith('>'):
            node_item = DmgHtmlNodeItem(bounding_box, shape, label[1:-1], label_pos, url, parent, **kwargs)
        else:
            node_item = DmgNodeItem(bounding_box, shape, label, label_pos, url, parent, **kwargs)

            if cluster:
                bounding_box = QRectF(bounding_box)
                bounding_box.setHeight(30)
                node_item.set_hover_shape(bounding_box)

        return node_item

    def _check_constraints(self, dictionary, node=True):
        super(DmgItemFactory, self)._check_constraints(dictionary, node)

        if 'highlighted_color' not in dictionary:
            dictionary['highlighted_color'] = self._highlighted_color

        if 'highlighted_pen_width' not in dictionary:
            dictionary['highlighted_pen_width'] = self._highlighted_pen_width

        if 'highlighted_label_pen_width' not in dictionary:
            dictionary['highlighted_label_pen_width'] = self._highlighted_label_pen_width