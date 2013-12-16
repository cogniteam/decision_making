from .graph_item_factory import GraphItemFactory
from .hovered_edge_item import HoveredEdgeItem
from .hovered_node_item import HoveredNodeItem
from python_qt_binding.QtCore import QRectF
from python_qt_binding.QtGui import QColor


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
