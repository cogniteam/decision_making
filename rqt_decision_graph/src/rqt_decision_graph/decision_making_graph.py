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

from python_qt_binding.QtGui import QWidget
import rospkg
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from rqt_gui_py.plugin import Plugin
from .graph_widget import GraphWidget


class DecisionMakingGraph(Plugin):

    def __init__(self, context):
        super(DecisionMakingGraph, self).__init__(context)

        self.initialized = False
        self.setObjectName('DecisionMakingGraph')

        self._widget = GraphWidget(rospkg.RosPack())
        self._widget.setWindowTitle(GraphWidget.get_unique_name(context))

        context.add_widget(self._widget)

        self._filter = 'decision_making'

        self._subscriber = None
        self._subscribe('/decision_making/monitoring')

    def _on_message(self, message):
        if not message.status[0].name[-len(self._filter):] == self._filter:
            return

        self._widget.update(message)

    def _subscribe(self, topic):
        if self._subscriber:
            self._subscriber.unregister()

        self._subscriber = rospy.Subscriber(topic, DiagnosticArray, self._on_message)

    def shutdown_plugin(self):
        if self._subscriber is not None:
            self._subscriber.unregister()