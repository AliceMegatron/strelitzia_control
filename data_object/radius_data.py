# -*- coding:utf-8 -*-
"""
GNSS地图的曲率半径容器和计算函数
@author: QinYu TianHao
"""
import numpy

from .. import module_object as mo
from .. import tools


class Radius():
    """
    曲率半径数据类
    用来存储曲率半径数据
    """

    def _init_(self, R=0.0, tx=0.0, ty=0.0, scope_x=0.0, scope_y=0.0, cx=0.0, cy=0.0):
        """
        __init__(self): 类的构造函数

        参数:
            R: 拟合出的曲率半径
            tx: 计算曲率的导航点 x 坐标
            ty: 计算曲率的导航点 y 坐标
            scope_x: 经过坐标系转换后用于拟合曲率的 x 点集列表
            scope_y: 经过坐标系转换后用于拟合曲率的 y 点集列表
            cx: 经过坐标系转换后用计算曲率的导航点 x 坐标
            cy: 经过坐标系转换后用计算曲率的导航点 y 坐标
        返回: None
        """
        self.R = R
        self.tx = tx
        self.ty = ty
        self.scope_x = scope_x
        self.scope_y = scope_y
        self.cx = cx
        self.cy = cy


    def __call__(self, _radius):
        """
        __call__: 类的 call 函数,可使类对象能像函数一样传入参数

        参数:
            _radius: 曲率半径数据的元组(R, tx, ty, scope_x, scope_y, cx, cy)
        返回: None
        """
        self.R = _radius[0]
        self.tx = _radius[1]
        self.ty = _radius[2]
        self.scope_x = _radius[3]
        self.scope_y = _radius[4]
        self.cx = _radius[5]
        self.cy = _radius[6]

    def getR(self):
        """
        getR(self): 返回曲率半径

        参数: None
        返回:
            self.R: 返回曲率半径
        """
        return self.R


    def getPoint(self):
        """
        getPoint(self): 返回曲率半径计算的点数据

        参数: None
        返回:
            self.tx: 计算曲率的导航点 x 坐标
            self.ty: 计算曲率的导航点 y 坐标
            self.scope_x: 经过坐标系转换后用于拟合曲率的 x 点集列表
            self.scope_y: 经过坐标系转换后用于拟合曲率的 y 点集列表
            self.cx: 经过坐标系转换后用计算曲率的导航点 x 坐标
            self.cy: 经过坐标系转换后用计算曲率的导航点 y 坐标
        """
        return self.tx, self.ty, self.scope_x, self.scope_y, self.cx, self.cy


    def getAll(self):
        """
        getAll(self): 返回曲率半径和曲率半径计算的点数据

        参数: None
        返回:
            self.R: 拟合出的曲率半径
            self.tx: 计算曲率的导航点 x 坐标
            self.ty: 计算曲率的导航点 y 坐标
            self.scope_x: 经过坐标系转换后用于拟合曲率的 x 点集列表
            self.scope_y: 经过坐标系转换后用于拟合曲率的 y 点集列表
            self.cx: 经过坐标系转换后用计算曲率的导航点 x 坐标
            self.cy: 经过坐标系转换后用计算曲率的导航点 y 坐标
        """
        return self.R, self.tx, self.ty, self.scope_x, self.scope_y, self.cx, self.cy


def calcu_radius(gnss_data, route):
    """
    计算GNSS地图的曲率半径，用于前方弯道识别

    参数:
        gnss_data: gnss数据对象
        route: 路径对象
    返回:
        radius: 曲率半径对象
    """
    navi_x, navi_y = route.get()
    near_point_index = mo.calcu_near_point_index(gnss_data.x, gnss_data.y, navi_x, navi_y)

    # 曲率半径检测点提前量
    if near_point_index + 3 < len(navi_x):
        front_target = near_point_index + 3
    else:
        front_target = near_point_index + 3 - len(navi_x)
    radius = Radius()
    # 计算 target_index 的曲率半径,把计算结果存入 Radius 对象中
    radius(compute_R(front_target, gnss_data, 2, navi_x, navi_y))
    return radius


def is_curve(radius_data, curve_radius=100):
    """
    通过GNSS路径的区率大小来检查是否为弯道

    参数:
        radius_data: 曲率半径对象
        curve_radius: 当小于此值认为是弯道(默认100)
    返回:
        是否是弯道
    """
    if radius_data.getR() <= curve_radius:
        return True
    else:
        return False


def compute_R(index, gnss_data, scope, navi_x, navi_y):
    """ 
    计算曲率半径

    参数：
        index    : 追踪的导航点的索引
        gnss_data：车辆状态
        scope    : 设定拟合多项式使用的导航点索引范围
        navi_x   : 全局导航点x坐标
        navi_y   ：全局导航点y坐标
    返回:
        R: 拟合出的曲率半径
        tx: 计算曲率的导航点 x 坐标
        ty: 计算曲率的导航点 y 坐标
        scope_x: 经过坐标系转换后用于拟合曲率的 x 点集列表
        scope_y: 经过坐标系转换后用于拟合曲率的 y 点集列表
        cx: 经过坐标系转换后用计算曲率的导航点 x 坐标
        cy: 经过坐标系转换后用计算曲率的导航点 y 坐标
    """
    last_index = len(navi_x)
    
    # 找出需拟合多项式的导航点的起始索引和结束索引
    if index - scope <= 0:
        start_index = 0
    else:
        start_index = index - scope

    if (index + scope + 1) < last_index:
        end_index = index + scope + 1
    else:
        end_index = -1

    # 找出需要拟合多项式的导航点
    scope_x = navi_x[start_index : end_index]
    scope_y = navi_y[start_index : end_index]

    # 选出计算曲率的导航点
    tx, ty = navi_x[index], navi_y[index]
    cx, cy = tx, ty

    # 是否转换为以车为原点的坐标系
    scope_x, scope_y = tools.change_system(gnss_data.yaw, gnss_data.x, gnss_data.y, scope_x, scope_y)
    cx, cy = tools.change_system(gnss_data.yaw, gnss_data.x, gnss_data.y, tx, ty)

    # 使用前面选出的导航点，拟合出一个二次多项式 (poly实际上是一个保存多项式系数的元组)
    poly = numpy.polyfit(scope_x, scope_y, 2)

    # 通过 R = （1 + y1**2)**1.5 / np.absolute(y2) 公式，带入 x 点求出 x 点的曲率半径
    # y1 是多项式的一阶导数，y2 是多项式的二阶导数
    y1 = 2*poly[0]*cx + poly[1]
    y2 = 2*poly[0]
    R = (1 + y1**2)**1.5 / numpy.absolute(y2)
    
    return R, tx, ty, scope_x, scope_y, cx, cy