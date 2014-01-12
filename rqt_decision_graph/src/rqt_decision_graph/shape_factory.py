"""
Copyright (c) 2013, Cogniteam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of the Cogniteam nor the
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

from python_qt_binding.QtCore import Qt, QPointF
from python_qt_binding.QtGui import QBrush, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsSimpleTextItem, \
                                    QPainterPath, QPen, QGraphicsPolygonItem, QPolygonF


class StaticClassError(Exception):
    pass


class QGraphicsRoundRectItem(QGraphicsRectItem):
    def __init__(self, bounding_box):
        super(QGraphicsRoundRectItem, self).__init__(bounding_box)

    def paint(self, painter, option, widget):
        if self.pen() is not None:
            painter.setPen(self.pen())
        painter.drawRoundedRect(self.boundingRect(), 4.0, 4.0)


class ShapeFactory:
    message = None

    def __init__(self):
        raise StaticClassError("%s is a static class and cannot be initiated." % ShapeFactory)

    @staticmethod
    def create(shape, bounding_box):
        ShapeFactory.message = None
        graphics_item = None

        if shape in ('box', 'rect', 'rectangle'):
            graphics_item = QGraphicsRoundRectItem(bounding_box)
        elif shape in ('ellipse', 'point'):
            graphics_item = QGraphicsEllipseItem(bounding_box)
        elif shape == 'diamond':
            points = QPolygonF([QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height() / 2),

                                QPointF(bounding_box.x() + bounding_box.width() / 2,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y() + bounding_box.height() / 2),

                                QPointF(bounding_box.x() + bounding_box.width() / 2,
                                        bounding_box.y())])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'parallelogram':
            points = QPolygonF([QPointF(bounding_box.x() + bounding_box.width() * 1/6,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width() * 5/6,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height())])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'cds':
            points = QPolygonF([QPointF(bounding_box.x(),
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width() * 5/6,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y() + bounding_box.height() / 2),

                                QPointF(bounding_box.x() + bounding_box.width() * 5/6,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height())])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'rarrow':
            points = QPolygonF([QPointF(bounding_box.x(),
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width() * 4/6,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width() * 4/6,
                                        bounding_box.y() - bounding_box.height() * 2/6),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y() + bounding_box.height() / 2),

                                QPointF(bounding_box.x() + bounding_box.width() * 4/6,
                                        bounding_box.y() + bounding_box.height() * 8/6),

                                QPointF(bounding_box.x() + bounding_box.width() * 4/6,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height())])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'larrow':
            points = QPolygonF([QPointF(bounding_box.x() + bounding_box.width() * 2/6,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width() * 2/6,
                                        bounding_box.y() - bounding_box.height() * 2/6),

                                QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height() / 2),

                                QPointF(bounding_box.x() + bounding_box.width() * 2/6,
                                        bounding_box.y() + bounding_box.height() * 8/6),

                                QPointF(bounding_box.x() + bounding_box.width() * 2/6,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y())])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'record':
            graphics_item = QGraphicsRectItem(bounding_box)
        elif shape == 'hexagon':
            points = QPolygonF([QPointF(bounding_box.x() + bounding_box.width() * 1/5,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width() * 4/5,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y() + bounding_box.height() / 2),

                                QPointF(bounding_box.x() + bounding_box.width() * 4/5,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x() + bounding_box.width() * 1/5,
                                        bounding_box.y() + bounding_box.height()),

                                QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height() / 2)])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'triangle':
            points = QPolygonF([QPointF(bounding_box.x() + bounding_box.width() / 2,
                                        bounding_box.y()),

                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y() + bounding_box.height() * 3/4),

                                QPointF(bounding_box.x(),
                                        bounding_box.y() + bounding_box.height() * 3/4)])
            graphics_item = QGraphicsPolygonItem(points)
        elif shape == 'circle':
            diameter = min(bounding_box.width(), bounding_box.height())
            graphics_item = QGraphicsEllipseItem(bounding_box.x(), bounding_box.y(), diameter, diameter)
        else:
            graphics_item = QGraphicsRectItem(bounding_box)
            ShapeFactory.message = "WARNING: %s is unknown shape, box used instead" % shape

        return graphics_item