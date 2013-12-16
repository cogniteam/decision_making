from __future__ import division

from python_qt_binding.QtCore import Qt, QPointF
from python_qt_binding.QtGui import QBrush, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsSimpleTextItem, \
                                    QPainterPath, QPen, QGraphicsPolygonItem, QPolygonF


class StaticClassError(Exception):
    pass


class ShapeFactory:
    message = None

    def __init__(self):
        raise StaticClassError("%s is a static class and cannot be initiated." % ShapeFactory)

    @staticmethod
    def create(shape, bounding_box):
        ShapeFactory.message = None
        graphics_item = None

        if shape == 'box':
            graphics_item = QGraphicsRectItem(bounding_box)
        elif shape == 'ellipse':
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
                                QPointF(bounding_box.x() + bounding_box.width(),
                                        bounding_box.y()),
                                QPointF(bounding_box.x() + bounding_box.width() + bounding_box.height() / 2,
                                        bounding_box.y() + bounding_box.height() / 2),
                                QPointF(bounding_box.x() + bounding_box.width(),
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
        elif shape == 'point':
            graphics_item = QGraphicsEllipseItem(bounding_box)
        else:
            graphics_item = QGraphicsRectItem(bounding_box)
            ShapeFactory.message = "WARNING: %s is unknown shape, box used instead" % shape

        return graphics_item