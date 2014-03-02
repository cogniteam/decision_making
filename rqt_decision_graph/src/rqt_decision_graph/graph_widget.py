"""
Copyright (c) 2013, Cogniteam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

*   Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

*   Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

*   Neither the name of the Cogniteam nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from __future__ import division
from os import path
from threading import Lock
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QColor, QComboBox
from collections import namedtuple
from .interactive_graphics_view import InteractiveGraphicsView
from .decision_graph import DecisionGraph
from .graph import Graph, GraphParseException
from .dot_to_qt import DotToQtGenerator
from .dmg_item_factory import DmgItemFactory

from .dot_processor import DotProcessor


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
        self.fit_to_view_button.setIcon(QIcon.fromTheme('zoom-fit-best'))
        self.fit_to_view_button.pressed.connect(self._fit_to_view)

        self.decision_graphs_combo_box.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLength)
        self.decision_graphs_combo_box.currentIndexChanged['QString'].connect(self._graph_item_changed)

        self._dot_processor = DotProcessor(self._dot_to_qt)

        self.decision_graphs = dict()
        self.states = dict()

    def update(self, message):
        data = self._get_data_from_message(message)
        key = self._get_key(data)

        if key not in self.decision_graphs:
            try:
                self._add_graph(key, data)
                print 'INFO: Graph has been added'
            except GraphParseException as ex:
                print 'ERROR: Failed to load graph: %s', ex.message
        else:
            self.states[key] = data['name'], data['status']

            if self.decision_graphs[key].graph_id != message.status[0].values[-1].value:
                self.decision_graphs[key].graph_id = message.status[0].values[-1].value
                print 'INFO: Graph id has been changed'
            elif self._current_graph == self.decision_graphs[key]:
                if not self._update_graph(data['name'], data['status']):
                    print 'WARNING: Failed to find appropriate graph for update'

    def _load_ui(self, ros_package):
        user_interface_file = path.join(ros_package.get_path('rqt_decision_graph'), 'resource', 'DecisionGraph.ui')

        loadUi(user_interface_file, self, {'InteractiveGraphicsView': InteractiveGraphicsView})

    def _import(self):
        file_path, _ = QFileDialog.getOpenFileName(self, self.tr('Import custom graph'),
                                                   None, self.tr('DOT graph (*.dot)'))

        if file_path is None or file_path == '':
            return

        custom_graph = Graph(self._dot_processor, file_path, file_path)
        self.decision_graphs[custom_graph.source] = custom_graph
        self._current_graph = custom_graph

        self.decision_graphs_combo_box.addItem(custom_graph.source)
        self.decision_graphs_combo_box.setCurrentIndex(self.decision_graphs_combo_box.findText(custom_graph.source))

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
                                       data['node_exe_dir'],
                                       self._dot_processor,
                                       key)

        self.decision_graphs[key] = decision_graph
        self.decision_graphs_combo_box.addItem(key)

        self._lock.release()

    def _reset_graph_state(self, name, status):
        if self._current_graph is not None:
            for node in self._current_graph.nodes.values():
                if name[:len(node.url)] == node.url:
                    node.highlight(True) if 'started' == status else node.highlight(False)

    def _update_graph(self, name, status):
        self._lock.acquire()
        is_updated = False
        if self._current_graph is not None:
            for node in self._current_graph.nodes.values():
                if 'started' == status and name[:len(node.url)] == node.url:
                    node.highlight(True)
                    is_updated = True
                elif 'stopped' == status and name == node.url:
                    node.highlight(False)
                    is_updated = True

        self._lock.release()

        return is_updated

    def _graph_item_changed(self, event):
        self._lock.acquire()
        if event in self.decision_graphs:
            self._current_graph = self.decision_graphs[event]
            self._redraw_graph_view()
            self._fit_to_view()

            if isinstance(self._current_graph, DecisionGraph):
                state = self.states.get(self._current_graph.key, None)
                if state is not None:
                    self._reset_graph_state(state[0], state[1])

        self._lock.release()

    def _get_data_from_message(self, message):
        return {value.key: value.value for value in message.status[0].values}

    def _get_key(self, data):
        return data['name'].split('/')[1] + data['node_name']

    def _redraw_graph_view(self):

        self._current_graph.load()
        self._scene.clear()

        for node_item in self._current_graph.nodes.itervalues():
            self._scene.addItem(node_item)
        for edge_items in self._current_graph.edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())

    def _fit_to_view(self):
        self.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)
