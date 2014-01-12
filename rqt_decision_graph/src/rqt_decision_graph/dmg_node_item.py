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

        self._graphics_item_pen.setColor(color)
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

