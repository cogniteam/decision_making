from __future__ import division

import rospkg
import rospy

from diagnostic_msgs.msg import DiagnosticArray

from python_qt_binding.QtGui import QWidget

from rqt_gui_py.plugin import Plugin
from .graph_widget import GraphWidget


class DecisionMakingGraph(Plugin):

    def __init__(self, context):
        super(DecisionMakingGraph, self).__init__(context)

        self.__filter = 'decision_making'
        
        self.initialized = False
        self.setObjectName('DecisionMakingGraph')

        self._widget = GraphWidget(rospkg.RosPack())
        self._widget.setWindowTitle(GraphWidget.get_unique_name(context))

        context.add_widget(self._widget)

        self._subscriber = None
        self._subscribe('/diagnostics')

    def _on_message(self, message):
        if not message.status[0].name[-len(self.__filter):] == self.__filter:
            return

        self._widget.update(message)

    def _subscribe(self, topic):
        if self._subscriber:
            self._subscriber.unregister()

        self._subscriber = rospy.Subscriber(topic, DiagnosticArray, self._on_message)

    def shutdown_plugin(self):
        if self._subscriber is not None:
            self._subscriber.unregister()