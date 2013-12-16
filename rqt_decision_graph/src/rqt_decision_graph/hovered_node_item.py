from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QGraphicsSimpleTextItem, QPen, QPainterPath, QColor

from .node_item import NodeItem


class HoveredNodeItem(NodeItem):

    HIGHLIGHT_LEVEL = 1
    HOVERED_COLOR = QColor(250, 0, 0)

    def __init__(self, bounding_box, shape, label, label_pos=None, url=None,
                 parent=None, **kwargs):

        super(HoveredNodeItem, self).__init__(bounding_box, shape, label, label_pos, url, parent, **kwargs)

        self._highlight_level = kwargs.get('highlight_level', self.HIGHLIGHT_LEVEL)
        self._hovered_color = kwargs.get('hovered_color', self.HOVERED_COLOR)

        self.hover_shape = None
        self.setAcceptHoverEvents(True)

    def set_hover_shape(self, shape):
        self.hover_shape = shape

    def shape(self):
        if self.hover_shape is not None:
            path = QPainterPath()
            path.addRect(self.hover_shape)
            return path
        else:
            return super(HoveredNodeItem, self).shape()

    def hoverEnterEvent(self, event):
        self.set_color(self.COLOR_RED)
        self._highlight_connections()

    def hoverLeaveEvent(self, event):
        self.set_color()
        self._highlight_connections(False)

    def _highlight_connections(self, highlighted=True):
        if highlighted:
            if self._highlight_level > 1:
                cyclic_edges = self._incoming_edges.intersection(self._outgoing_edges)
                # incoming edges in blue
                incoming_nodes = set()
                for incoming_edge in self._incoming_edges.difference(cyclic_edges):
                    incoming_edge.set_color(self.COLOR_BLUE)
                    if incoming_edge.from_node != self:
                        incoming_nodes.add(incoming_edge.from_node)
                # outgoing edges in green
                outgoing_nodes = set()
                for outgoing_edge in self._outgoing_edges.difference(cyclic_edges):
                    outgoing_edge.set_color(self.COLOR_GREEN)
                    if outgoing_edge.to_node != self:
                        outgoing_nodes.add(outgoing_edge.to_node)
                # incoming/outgoing edges in teal
                for edge in cyclic_edges:
                    edge.set_color(self.COLOR_TEAL)

                if self._highlight_level > 2:
                    cyclic_nodes = incoming_nodes.intersection(outgoing_nodes)
                    # incoming nodes in blue
                    for incoming_node in incoming_nodes.difference(cyclic_nodes):
                        incoming_node.set_color(self.COLOR_BLUE)
                    # outgoing nodes in green
                    for outgoing_node in outgoing_nodes.difference(cyclic_nodes):
                        outgoing_node.set_color(self.COLOR_GREEN)
                    # incoming/outgoing nodes in teal
                    for node in cyclic_nodes:
                        node.set_color(self.COLOR_TEAL)
            else:
                if self._highlight_level > 1:
                    for incoming_edge in self._incoming_edges:
                        incoming_edge.set_color()
                        if self.highlight_level > 2 and incoming_edge.from_node != self:
                            incoming_edge.from_node.set_color()
                    for outgoing_edge in self._outgoing_edges:
                        outgoing_edge.set_color()
                        if self.highlight_level > 2 and outgoing_edge.to_node != self:
                            outgoing_edge.to_node.set_color()
