"""
Copyright (c) 2011, Dirk Thomas, TU Darmstadt
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the TU Darmstadt nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Note: This is modified version by Cogniteam
"""

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

        self._pen_width = kwargs.get('penwidth', self.PEN_WIDTH)
        self._label_pen_width = kwargs.get('label_pen_width', self.LABEL_PEN_WIDTH)
        self._color = kwargs.get('color', self.COLOR)

