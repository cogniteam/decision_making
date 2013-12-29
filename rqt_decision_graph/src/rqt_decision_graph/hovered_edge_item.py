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
from .edge_item import EdgeItem


class HoveredEdgeItem(EdgeItem):

    HIGHLIGHT_LEVEL = 1
    HOVERED_COLOR = QColor(250, 0, 0)

    def __init__(self, spline, label, label_center, from_node, to_node,
                 parent=None, **kwargs):
        super(HoveredEdgeItem, self).__init__(spline, label, label_center, from_node, to_node, parent, **kwargs)

        self._highlight_level = kwargs.get('highlight_level', self.HIGHLIGHT_LEVEL)
        self._hovered_color = kwargs.get('hovered_color', self.HOVERED_COLOR)

        if self._label is not None:
            self._label.hoverEnterEvent = self.hoverEnterEvent
            self._label.hoverLeaveEvent = self.hoverLeaveEvent
            self._label.setAcceptHoverEvents(True)

        if self._arrow is not None:
            self._arrow.hoverEnterEvent = self.hoverEnterEvent
            self._arrow.hoverLeaveEvent = self.hoverLeaveEvent
            self._arrow.setAcceptHoverEvents(True)

    def hoverEnterEvent(self, event):
         # hovered edge item in red
        self.set_color(self._hovered_color)
        self._highlight_connections()

    def hoverLeaveEvent(self, event):
        self.set_color()
        self._highlight_connections(False)

    def _highlight_connections(self, highlighted=True):
        if highlighted:
            if self._highlight_level > 1:
                if self.from_node != self.to_node:
                    # from-node in blue
                    self.from_node.set_color(self.COLOR_BLUE)
                    # to-node in green
                    self.to_node.set_color(self.COLOR_GREEN)
                else:
                    # from-node/in-node in teal
                    self.from_node.set_color(self.COLOR_TEAL)
                    self.to_node.set_color(self.COLOR_TEAL)
            if self._highlight_level > 2:
                # sibling edges in orange
                for sibling_edge in self._sibling_edges:
                    sibling_edge.set_color(self.COLOR_ORANGE)
        else:
            if self._highlight_level > 1:
                self.from_node.set_color()
                self.to_node.set_color()
            if self._highlight_level > 2:
                for sibling_edge in self._sibling_edges:
                    sibling_edge.set_color()
