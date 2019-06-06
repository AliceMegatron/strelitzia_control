# -*- coding:utf-8 -*-
"""
GNSS点追踪器
@author: QinYu TianHao
"""
import math
import numpy as np


def calcu_front_distance(v, k, b):
    """
    计算前视距离
    
    参数:
        v: 当前行驶速度
        k: 前视距离速度系数
        b: 前视距离基数
    返回:
        前视距离
    """
    return  v * k + b


def calcu_near_point_index(x, y, navi_X, navi_Y):
    """
    计算出导航点的下标
    根据距离车辆最近导航点的下标

    参数:
        x: 当前车辆所在点的x坐标
        y: 当前车辆所在点的y坐标
        navi_X: 全局导航坐标点x点集
        navi_Y: 全局导航坐标点y点集
    返回:
        near_index: 最近点的下标
    """

    # 搜索最临近的路点
    diff_X = (x - ix for ix in navi_X)
    diff_Y = (y - iy for iy in navi_Y)

    distances = np.array([abs(math.sqrt(idx**2 + idy**2)) for (idx, idy) in zip(diff_X, diff_Y)])
    near_index = distances.argmin()
    return near_index


def calcu_navigation_point_index(x, y, navi_X, navi_Y, front_distance):
    """
    计算出导航点的下标
    根据距离车辆最近导航点和前视距离来计算导航点的下标

    参数:
        x: 当前车辆所在点的x坐标
        y: 当前车辆所在点的y坐标
        navi_X: 全局导航坐标点x点集
        navi_Y: 全局导航坐标点y点集
        front_distance: 前视距离
    返回:
        navi_index: 导航点的下标
    """

    # 搜索最临近的路点
    navi_index = calcu_near_point_index(x, y, navi_X, navi_Y)

    # 最近点向前,点之间距离之和
    L = 0.0

    # 最近点前面点之间距离和 前视距离比较 来求得导航点的下标
    while front_distance > L and (navi_index + 1) < len(navi_X):
        dx = navi_X[navi_index + 1] - navi_X[navi_index]
        dy = navi_Y[navi_index + 1] - navi_Y[navi_index]
        L += math.sqrt(dx**2 + dy**2)
        navi_index += 1

    return navi_index


def pure_pursuit_point(x, y, yaw, v, navi_X, navi_Y, prev_index, front_distance, wheelbase):
    """
    纯追踪算法
    通过当前车辆航向角和坐标求得把车辆行驶到导航点的前轮角度
    
    参数:
        x: 当前车辆所在点的x坐标
        y: 当前车辆所在点的y坐标
        yaw: 当前车辆航向角
        v: 当前车辆速度
        navi_X: 全局导航坐标点x点集
        navi_Y: 全局导航坐标点y点集
        last_navigation_point_index: 上一次计算的导航点下标
        front_distance: 前视距离
        wheelbase: 车辆轴距
    返回:
        delta: 前轮角度
        navigation_point_index: 导航点下标
    """

    # 根据当前车辆坐标计算导航点的下标
    navi_index = calcu_navigation_point_index(x, y, navi_X, navi_Y, front_distance)

    if navi_index is None:
        prev_index = navi_index

    # 保证导航点只会按顺序向前
    if prev_index >= navi_index:
        navi_index = prev_index

    # 保证导航点下标不会越界,如果越界就取最后一个导航点
    if navi_index < len(navi_X):
        navi_x = navi_X[navi_index]
        navi_y = navi_Y[navi_index]
    else:
        navi_x = navi_X[-1]
        navi_y = navi_Y[-1]
        navi_index = len(navi_X) - 1

    # 计算alpha角度(车和导航点之间的夹角)
    alpha = math.atan2(navi_y - y, navi_x - x) - yaw

    # 当倒车时用 180 度减去 alpha 
    if v < 0:  # back
        alpha = math.pi - alpha

    # 通过轴距、alpha角度, 前视距离计算出前轮角度
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha) / front_distance, 1.0)

    return delta, navi_index


def delta_to_wheel_degree(delta, scale):
    '''
    把追踪算法求得的前轮角度delta转换为方向盘角度wheel_degree

    参数:
        delta: 追踪算法求得的前轮角度
        scale: 方向盘和车轮转角比例,一般来说方向盘转角差不多10倍于车轮转角
    返回:
        angle: 方向盘角度
    '''
    # 因求出的前轮转角是反向的,所以去负来矫正
    delta = -delta
    # 因求出的前轮转角是弧度,这里转换为度数,并乘以方向盘转角和前轮转角比,求出方向盘转角
    wheel_degree = delta / math.pi * 180 * scale

    return wheel_degree


class GnssTracking(object):
    """
    gnss点追踪功能类
    基于纯追踪算法追踪gnss点得出方向盘转角，使得车辆追踪gnss点行驶

    front_distance_k: 前视距离速度系数\n
    front_distance_b: 前视距离基数\n
    wheelbase: 轴距\n
    wheel_degree_scale: 方向盘和车轮转角比例\n
    wheel_degree_offset: 方向盘偏移校准量，确保车轮回中
    """

    def __init__(self, front_distance_k, front_distance_b, wheelbase,  wheel_degree_scale):
        """
        初始化gnss点追踪功能需要的属性

        参数:
            self._front_distance_k: 前视距离速度系数
            self._front_distance_b: 前视距离基数
            self._wheelbase: 轴距
            self._navigation_point_index: 追踪的导航点
            self._wheel_degree_scale: 方向盘和车轮转角比例
        """
        
        self._front_distance_k = front_distance_k
        self._front_distance_b = front_distance_b

        self._wheelbase = wheelbase
        self._navi_index = None
        
        self._wheel_degree_scale = wheel_degree_scale


    def pure_tracking(self, gnss_data, route):
        """
        gnss点纯追踪功能方法

        参数:
            gnss_data: 当前车辆gnss测量信息
            route： 当前车辆使用的gnss路径
        返回:
            wheel_degree: 方向盘角度
        """
        # 求出前视距离,前视距离用来辅助计算导航点
        front_distance = calcu_front_distance(0, self._front_distance_k, self._front_distance_b)

        # 缓存上次找到的导航点
        prev_index = self._navi_index
        # 下面的if语句是用来解决环绕问题的的临时办法，当追踪的点为最后一个点时，把追踪点换为全局导航点的第一个点
        if prev_index >= len(route):
            prev_index = 0

        # 获取当前车辆所在的xy坐标和车辆航向角
        x, y, yaw = gnss_data.get()
        # 获取路径导航点集X和点集Y
        navi_X, navi_Y = route.get()
        # 因当前无法获得速度，所以速度设定为零
        v = 0

        # 追踪算法求出前轮转角以及导航点
        delta, self._navigation_point_index = pure_pursuit_point(x, y, yaw, v, navi_X, navi_Y,
                                                prev_index, front_distance, self._wheelbase)
        
        # 把前轮转角转换为方向盘转角
        wheel_degree = delta_to_wheel_degree(delta, self._wheel_degree_scale)

        return wheel_degree


if __name__ == '__main__':
    pass