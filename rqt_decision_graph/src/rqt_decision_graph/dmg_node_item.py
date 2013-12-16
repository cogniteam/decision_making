from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QGraphicsSimpleTextItem, QPen, QColor

from .hovered_node_item import HoveredNodeItem


class DmgNodeItem(HoveredNodeItem):

    HIGHLIGHTED_COLOR = QColor(100, 100, 100)
    HIGHLIGHTED_PEN_WIDTH = 2.0
    HIGHLIGHTED_LABEL_PEN_WIDTH = 1.0

    def __init__(self, bounding_box, shape, label, label_pos=None, url=None, parent=None, **kwargs):
        super(DmgNodeItem, self).__init__(bounding_box, shape, label, label_pos, url, parent, **kwargs)

        self._highlighted_color = kwargs.get('highlighted_color', self.HIGHLIGHTED_COLOR)
        self._highlighted_pen_width = kwargs.get('highlighted_pen_width', self.HIGHLIGHTED_PEN_WIDTH)
        self._highlighted_label_pen_width = kwargs.get('highlighted_label_pen_width', self.HIGHLIGHTED_LABEL_PEN_WIDTH)

        self._current_color = self._color
        self._previous_color = self._color

        self._current_pen_width = self._pen_width
        self._previous_pen_width = self._pen_width

    def set_color(self, color=None):
        if color is None:
            color = self._color

        self._previous_color = self._current_color
        self._current_color = color

        self._brush.setColor(color)
        self._graphics_item_pen.setColor(color)
        self._label_pen.setColor(color)

        self._graphics_item.setPen(self._graphics_item_pen)

        if self._label is not None:
            self._label.setBrush(self._brush)
            self._label.setPen(self._label_pen)

    def hoverEnterEvent(self, event):
        self.set_color(self._highlighted_color)
        self._highlight_connections()

    def hoverLeaveEvent(self, event):
        self.set_color()
        self._highlight_connections(False)

    def highlight(self, highlighted=True):
        if highlighted:
            self._graphics_item_pen.setWidthF(self._highlighted_pen_width)
            self._label_pen.setWidthF(self._highlighted_label_pen_width)
        else:
            self._graphics_item_pen.setWidthF(self._pen_width)
            self._label_pen.setWidthF(self._label_pen_width)
        self._label.setPen(self._label_pen)
        self._graphics_item.setPen(self._graphics_item_pen)

