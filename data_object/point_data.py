# -*- coding:utf-8 -*-
"""
2D和3D点的容器
@author: QinYu TianHao
"""
import math


class Point2D(object):
    """
    二维点对象

    x: 点的x坐标
    y: 点的y坐标
    """

    def __init__(self, x, y):
        self._x = x
        self._y = y


    def __str__(self):
        self_class = type(self)
        return "<object:{}> x:{} y:{}".format(self_class.__name__, self._x, self._y)


    def get(self):
        return self._x, self._y


    @property
    def x(self):
        return self._x


    @property
    def y(self):
        return self._y


class Point3D(Point2D):
    """
    三维点对象

    x: 点的x坐标
    y: 点的y坐标
    z: 点的z坐标
    """

    def __init__(self, x, y, z):
        super(Point3D, self).__init__(x, y)
        self._z = z


    def __str__(self):
        self_class = type(self)
        return "<object:{}> x:{} y:{} z:{}".format(self_class.__name__, self._x, self._y, self._z)


    def get(self):
        return self._x, self._y, self._z


    @property
    def z(self):
        return self._z


    def distance(self):
        return math.sqrt(self._x**2 + self._y**2 + self._z**2)


if __name__ == '__main__':
    p2d = Point2D(1, 2)
    print(p2d)
    p3d = Point3D(1, 2, 3)
    print(p3d.distance())
