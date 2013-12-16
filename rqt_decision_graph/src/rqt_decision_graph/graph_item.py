from python_qt_binding.QtGui import QGraphicsItemGroup, QColor


class GraphItem(QGraphicsItemGroup):
    COLOR_BLACK = QColor(0, 0, 0)
    COLOR_BLUE = QColor(0, 0, 204)
    COLOR_GREEN = QColor(0, 170, 0)
    COLOR_ORANGE = QColor(255, 165, 0)
    COLOR_RED = QColor(255, 0, 0)
    COLOR_TEAL = QColor(0, 170, 170)
    COLOR_DARK_GRAY = QColor(75, 75, 75)
    COLOR_GRAY = QColor(212, 212, 212)

    PEN_WIDTH = 1.0
    LABEL_PEN_WIDTH = 0.1
    COLOR = QColor(0, 0, 0)

    def __init__(self, parent=None, **kwargs):
        super(GraphItem, self).__init__(parent)

        self._pen_width = kwargs.get('pen_width', self.PEN_WIDTH)
        self._label_pen_width = kwargs.get('label_pen_width', self.LABEL_PEN_WIDTH)
        self._color = kwargs.get('color', self.COLOR)

