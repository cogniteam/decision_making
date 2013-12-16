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
