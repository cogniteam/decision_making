from __future__ import division

from os import path
from threading import Lock

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QColor, QComboBox

from .interactive_graphics_view import InteractiveGraphicsView
from .decision_graph import DecisionGraph
from .graph import Graph, GraphParseException
from .dot_to_qt import DotToQtGenerator
from .dmg_item_factory import DmgItemFactory
from .graph_item_factory import GraphItemFactory
from .hovered_item_factory import HoveredGraphItemFactory

class GraphWidget(QWidget):
    @staticmethod
    def get_unique_name(context):
        return ('Decision Graph (%d)' % context.serial_number()) if context.serial_number() > 1 else 'Decision Graph'

    @staticmethod
    def get_file_name(absolute_path):
        return ".".join(path.basename(absolute_path).split('.')[:-1])

    def __init__(self, ros_package):
        super(GraphWidget, self).__init__()

        self._current_graph = None
        self._lock = Lock()

        self._load_ui(ros_package)

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        factory = DmgItemFactory()
        factory.set_color(QColor(50, 50, 50))
        factory.set_highlighted_color(QColor(0, 150, 0))
        self._dot_to_qt = DotToQtGenerator(factory)

        self.initialized = False
        self.setObjectName('GraphWidget')

        self.graphics_view.setScene(self._scene)
        self.open_button.setIcon(QIcon.fromTheme('document-open'))
        self.open_button.pressed.connect(self._import)
        self.export_button.setIcon(QIcon.fromTheme('document-export'))
        self.export_button.pressed.connect(self._export)
        self.fit_to_view_button.setIcon(QIcon.fromTheme('zoom-original'))
        self.fit_to_view_button.pressed.connect(self._fit_to_view)

        self.decision_graphs_combo_box.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLength)
        self.decision_graphs_combo_box.currentIndexChanged['QString'].connect(self._graph_item_changed)

        self.decision_graphs = dict()

    def update(self, message):
        data = self._get_data_from_message(message)
        key = self._get_key(data)

        if key not in self.decision_graphs:
            try:
                self._add_graph(key, data)
                print 'INFO: Graph has been added'
            except GraphParseException as ex:
                print 'ERROR: Failed to load graph: %s', ex.message
        elif self.decision_graphs[key].graph_id != message.status[0].values[-1].value:
            self.decision_graphs[key].graph_id = message.status[0].values[-1].value
            print 'INFO: Graph id has been changed'
        elif self._current_graph == self.decision_graphs[key]:
            if not self._update_graph(data['name'], data['status']):
                print 'WARNING: Failed to find appropriate graph for update'

    def _load_ui(self, ros_package):
        user_interface_file = path.join(ros_package.get_path('rqt_decision_graph'), 'resource', 'DecisionGraph.ui')

        loadUi(user_interface_file, self, {'InteractiveGraphicsView': InteractiveGraphicsView})

    def _import(self):
        file_path, _ = QFileDialog.getOpenFileName(self, self.tr('Import custom graph'), None, self.tr('DOT graph (*.dot)'))

        if file_path is None or file_path == '':
            return

        custom_graph = Graph(self._dot_to_qt, file_path, file_path)
        self.decision_graphs[custom_graph.source] = custom_graph
        self._current_graph = custom_graph

        self.decision_graphs_combo_box.addItem(custom_graph.source)
        self.decision_graphs_combo_box.setCurrentIndex(self.decision_graphs_combo_box.findText(custom_graph.source))

        self._redraw_graph_view()
        self._fit_to_view()

    # Export graph as image
    def _export(self):
        file_name, _ = QFileDialog.getSaveFileName(self,
                                                   self.tr('Save as image'),
                                                   'graph.png',
                                                   self.tr('Image (*.bmp *.jpg *.png *.tiff)'))

        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)

    def _add_graph(self, key, data):
        self._lock.acquire()

        decision_graph = DecisionGraph(data['name'].split('/')[1],
                                       data['node_run_id'],
                                       data['node_name'],
                                       data['node_exe_file'],
                                       self._dot_to_qt)

        self.decision_graphs[key] = decision_graph
        self.decision_graphs_combo_box.addItem(key)

        self._lock.release()

    def _update_graph(self, name, status):
        self._lock.acquire()
        is_updated = False
        if self._current_graph is not None:
            for node in self._current_graph.nodes.values():
                if name == node.url:
                    node.highlight(True) if 'started' == status else node.highlight(False)
                    is_updated = True
        self._lock.release()

        return is_updated

    def _graph_item_changed(self, event):
        self._lock.acquire()
        if event in self.decision_graphs:
            self._current_graph = self.decision_graphs[event]
            self._redraw_graph_view()
            self._fit_to_view()
        self._lock.release()

    def _get_data_from_message(self, message):
        return {value.key: value.value for value in message.status[0].values}

    def _get_key(self, data):
        return data['name'].split('/')[1] + data['node_name']

    def _redraw_graph_view(self):
        self._scene.clear()
        self._current_graph.load()

        for node_item in self._current_graph.nodes.itervalues():
            self._scene.addItem(node_item)
        for edge_items in self._current_graph.edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())

    def _fit_to_view(self):
        self.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)
