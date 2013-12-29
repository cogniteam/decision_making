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

from python_qt_binding.QtGui import QColor
from .hovered_edge_item import HoveredEdgeItem


class DmgEdgeItem(HoveredEdgeItem):

    HIGHLIGHTED_COLOR = QColor(0, 0, 0)
    HIGHLIGHTED_PEN_WIDTH = 2.0

    def __init__(self, spline, label, label_center, from_node, to_node, parent=None, **kwargs):
        super(DmgEdgeItem, self).__init__(spline, label, label_center, from_node, to_node, parent, **kwargs)

        self._highlighted_color = kwargs.get('highlighted_color', self.HIGHLIGHTED_COLOR)
        self._highlighted_pen_width = kwargs.get('highlighted_pen_width', self.HIGHLIGHTED_PEN_WIDTH)

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
        self._edge_pen.setColor(color)
        self._label_pen.setColor(color)

        self._path.setPen(self._edge_pen)

        if self._arrow is not None:
            self._arrow.setBrush(self._brush)
            self._arrow.setPen(self._edge_pen)
        if self._label is not None:
            self._label.setBrush(self._brush)
            self._label.setPen(self._label_pen)

    def highlight(self, highlighted=True):
        if highlighted:
            self.set_color(self._highlighted_color)
        else:
            self.set_color(self._previous_color)

    def hoverEnterEvent(self, event):
        self.highlight()
        self._highlight_connections()

    def hoverLeaveEvent(self, event):
        self.highlight(False)
        self._highlight_connections(False)