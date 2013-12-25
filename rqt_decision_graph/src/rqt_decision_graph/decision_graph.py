from __future__ import division

from .graph import Graph


class DecisionGraph(Graph):
    # def __init__(self, name, graph_id, node_name, node_exe, dot_to_qt, parent=None):
    #     super(DecisionGraph, self).__init__(dot_to_qt=dot_to_qt, source=None, parent=parent)
    #     self.name = name
    #     self.graph_id = graph_id
    #     self.node_name = node_name
    #     self.node_exe = node_exe
    #
    #     if self.source is None:
    #         self.source = self._get_dot_file_name()

    def __init__(self, name, graph_id, node_name, node_exe, dot_processor, parent=None):
        super(DecisionGraph, self).__init__(dot_processor=dot_processor, source=None, parent=parent)
        self.name = name
        self.graph_id = graph_id
        self.node_name = node_name
        self.node_exe = node_exe

        if self.source is None:
            self.source = self._get_dot_file_name()

    def _get_dot_file_name(self):
        return self.node_exe.replace('lib', 'share').replace(self.node_exe.split('/')[-1], 'graphs') + '/' + self.name + '.dot'