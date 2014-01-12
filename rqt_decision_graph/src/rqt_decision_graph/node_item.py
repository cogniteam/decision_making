"""
Copyright (c) 2011, Dirk Thomas, TU Darmstadt
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the TU Darmstadt nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Note: This is modified version by Cogniteam
"""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QGraphicsSimpleTextItem, QPen

from .graph_item import GraphItem
from .shape_factory import ShapeFactory


class NodeItem(GraphItem):

    def __init__(self, bounding_box, shape, label, label_pos=None, url=None, parent=None, **kwargs):
        super(NodeItem, self).__init__(parent, **kwargs)

        self.url = url
        self._incoming_edges = set()
        self._outgoing_edges = set()
        self._brush = QBrush(self._color)

        self._label_pen = QPen()
        self._label_pen.setColor(self._color)
        self._label_pen.setJoinStyle(Qt.RoundJoin)
        self._label_pen.setWidthF(self._label_pen_width)

        if self._pen_width == 0.0:
            self._graphics_item_pen = QPen(Qt.NoPen)
        else:
            self._graphics_item_pen = QPen(self._label_pen)
            self._graphics_item_pen.setWidthF(self._pen_width)

        self._graphics_item = ShapeFactory.create(shape, bounding_box)
        if ShapeFactory.message is not None:
            print ShapeFactory.message

        self.addToGroup(self._graphics_item)

        if not shape == 'point':
            self._label = QGraphicsSimpleTextItem(label)

            label_rectangle = self._label.boundingRect()
            if label_pos is None:
                label_rectangle.moveCenter(bounding_box.center())
            else:
                label_rectangle.moveCenter(label_pos)
            self._label.setPos(label_rectangle.x(), label_rectangle.y())

            self.addToGroup(self._label)
        else:
            self._graphics_item.setBrush(self._color)
            self._label = None

        self._brush.setColor(self._color)
        self._graphics_item_pen.setColor(self._color)
        self._label_pen.setColor(self._color)

        self._graphics_item.setPen(self._graphics_item_pen)

        if self._label is not None:
            self._label.setBrush(self._brush)
            self._label.setPen(self._label_pen)

    def add_incoming_edge(self, edge):
        self._incoming_edges.add(edge)

    def add_outgoing_edge(self, edge):
        self._outgoing_edges.add(edge)

    def set_color(self, color=None):
        if color is None:
            color = self._color

        self._brush.setColor(color)
        self._graphics_item_pen.setColor(color)
        self._label_pen.setColor(color)

        self._graphics_item.setPen(self._graphics_item_pen)

        if self._label is not None:
            self._label.setBrush(self._brush)
            self._label.setPen(self._label_pen)