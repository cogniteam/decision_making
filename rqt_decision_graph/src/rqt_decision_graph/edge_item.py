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

from python_qt_binding.QtCore import Qt, QPointF
from python_qt_binding.QtGui import QBrush, QGraphicsSimpleTextItem, QPen, QColor, QPainterPath, QGraphicsPolygonItem, \
                                    QPolygonF, QPainterPath, QGraphicsPathItem

from .graph_item import GraphItem


class EdgeItem(GraphItem):

    EDGE_PEN_WIDTH = 0.5

    def __init__(self, spline, label, label_center, from_node, to_node, parent=None, **kwargs):
        super(EdgeItem, self).__init__(parent, **kwargs)

        self._edge_pen_width = kwargs.get('edge_pen_width', self.EDGE_PEN_WIDTH)

        self.from_node = from_node
        self.from_node.add_outgoing_edge(self)

        self.to_node = to_node
        self.to_node.add_incoming_edge(self)

        self._brush = QBrush(self._color)

        self._label_pen = QPen()
        self._label_pen.setColor(self._color)
        self._label_pen.setJoinStyle(Qt.RoundJoin)
        self._label_pen.setWidthF(self._label_pen_width)

        self._edge_pen = QPen()
        self._edge_pen.setColor(self._color)
        self._edge_pen.setWidthF(self._edge_pen_width)

        self._sibling_edges = set()

        self._label = None
        if label is not None:
            self._label = QGraphicsSimpleTextItem(label)

            font = self._label.font()
            font.setPointSize(8)
            self._label.setFont(font)

            label_rect = self._label.boundingRect()
            label_rect.moveCenter(label_center)
            self._label.setPos(label_rect.x(), label_rect.y())

        # spline specification according to http://www.graphviz.org/doc/info/attrs.html#k:splineType
        coordinates = spline.split(' ')
        # extract optional end_point
        end_point = None
        if coordinates[0].startswith('e,'):
            parts = coordinates.pop(0)[2:].split(',')
            end_point = QPointF(float(parts[0]), -float(parts[1]))
        # extract optional start_point
        if coordinates[0].startswith('s,'):
            parts = coordinates.pop(0).split(',')

        # first point
        parts = coordinates.pop(0).split(',')
        point = QPointF(float(parts[0]), -float(parts[1]))
        path = QPainterPath(point)

        while len(coordinates) > 2:
            # extract triple of points for a cubic spline
            parts = coordinates.pop(0).split(',')
            point1 = QPointF(float(parts[0]), -float(parts[1]))
            parts = coordinates.pop(0).split(',')
            point2 = QPointF(float(parts[0]), -float(parts[1]))
            parts = coordinates.pop(0).split(',')
            point3 = QPointF(float(parts[0]), -float(parts[1]))
            path.cubicTo(point1, point2, point3)

        self._arrow = None
        if end_point is not None:
            # draw arrow
            self._arrow = QGraphicsPolygonItem()
            polygon = QPolygonF()
            polygon.append(point3)
            offset = QPointF(end_point - point3)
            corner1 = QPointF(-offset.y(), offset.x()) * 0.35
            corner2 = QPointF(offset.y(), -offset.x()) * 0.35
            polygon.append(point3 + corner1)
            polygon.append(end_point)
            polygon.append(point3 + corner2)
            self._arrow.setPolygon(polygon)

        self._path = QGraphicsPathItem()
        self._path.setPath(path)
        self.addToGroup(self._path)

        self._brush.setColor(self._color)
        self._edge_pen.setColor(self._color)
        self._label_pen.setColor(self._color)

        self._path.setPen(self._edge_pen)
        if self._arrow is not None:
            self._arrow.setBrush(self._brush)
            self._arrow.setPen(self._edge_pen)
        if self._label is not None:
            self._label.setBrush(self._brush)
            self._label.setPen(self._label_pen)

    def add_to_scene(self, scene):
        scene.addItem(self)
        if self._label is not None:
            scene.addItem(self._label)
        if self._arrow is not None:
            scene.addItem(self._arrow)

    def setToolTip(self, tool_tip):
        super(EdgeItem, self).setToolTip(tool_tip)
        if self._label is not None:
            self._label.setToolTip(tool_tip)
        if self._arrow is not None:
            self._arrow.setToolTip(tool_tip)

    def add_sibling_edge(self, edge):
        self._sibling_edges.add(edge)

    def set_color(self, color=None):
        if color is None:
            color = self._color

        self._brush.setColor(color)
        self._edge_pen.setColor(color)
        self._label_pen.setColor(color)

        self._path.setPen(self._edge_pen)
        if self._arrow is not None:
            self._arrow.setBrush(self._brush)
            self._arrow.setPen(self._edge_pen)
        if self._label is not None:
            self._label.setBrush(self._brush)
            self._label.setPen(self._label_pen)
