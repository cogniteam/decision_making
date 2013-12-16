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