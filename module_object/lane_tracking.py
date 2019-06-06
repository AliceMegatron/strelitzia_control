# -*- coding:utf-8 -*-
"""
车道追踪器
@author: QinYu TianHao
"""
import math

from ..tools import change_system, trans_wheel_degree_format
from ..data_object import Point2D


def calcu_angle_between_car_lane(point_far, point_near):
    """
    求出车道线和激光雷达x轴的夹角 

    参数:
        point_far: 距离车较远的点
        point_near: 距离车较近的点
    返回:
        angle: 车道线和激光雷达x轴的夹角
    """
    angle = math.atan2((point_far.x - point_near.x), (point_far.y - point_near.y)) - math.pi / 2
    return angle


def calcu_average_angle(angle_left, angle_right):
    """
    求左右车道线和激光雷达x轴夹角的平均值

    参数:
        angle_left: 左车道线和激光雷达x轴夹角
        angle_right: 右车道线和激光雷达x轴夹角
    返回:
        angle_average: 车道线和激光雷达x轴的夹角的平均值
    """
    angle_average = (angle_left + angle_right) / 2
    return angle_average


def rotate_coordinate(rotate, point_input):
    """
    旋转坐标轴，使得旋转后的坐标系x轴和车道平行

    参数:
        rotate: 坐标系旋转的角度
        x_input: 被旋转点的x坐标
        y_input: 被旋转点的y坐标
    返回:
        x_output: 旋转后的x坐标
        y_output: 旋转后的y坐标
    """
    x_output, y_output = change_system(rotate, 0, 0, point_input.x, point_input.y)
    point_output = Point2D(x_output, y_output)
    return point_output


def calcu_distance_for_target_line(y_left, y_right, target_line):
    """
    求出距离目标线的距离(负右左正)
    目标线是基于车道中心线设定的
    加一个正数是把目标线设定到车道中心线左边
    减去一个正数是把目标线设定到车道中心的右边
    """
    road_width = abs(y_left) + abs(y_right)
    distance_for_road_center = (abs(y_left) - abs(y_right)) / 2
    distance_for_target_line = distance_for_road_center + target_line
    return distance_for_target_line, road_width


def calcu_wheel_degree(wheel_degree_scale, angle_average, distance_for_target_line, road_width):
    '''
    通过车与车道的夹角以及车距离目标线的距离(默认是车道中心线)和路宽的比例求出方向盘角度
    '''
    wheel_degree = wheel_degree_scale * (angle_average - distance_for_target_line / road_width)
    # 转换弧度为角度
    wheel_degree = wheel_degree / math.pi * 180

    return wheel_degree



class LaneTracking(object):
    """
    车道线追踪车道功能类
    根据车道线和车的距离角度关系求出方向盘角度，使得车辆能沿着车道行驶

    target_line: 目标线(负右左正, 0是正中)\n
    wheel_degree_scale: 方向盘比例尺
    """

    def __init__(self, target_line=0, wheel_degree_scale=5):
        """
        初始化类，设定目标线

        参数:
            target_line: 目标线(负右左正, 0是正中)
            wheel_degree_scale: 方向盘比例尺
        """
        # 负右左正
        self._target_line = target_line
        self._wheel_degree_scale = wheel_degree_scale


    def lane_keeping(self, lane_data):
        """
        车道保持功能方法，通过输入的车道线点求出方向盘转角

        参数:
            lane_data: 车道线点云数据
        返回:
            wheel_degree: 方向盘转角
        """
        # 求出车辆和车道线的夹角，方便把激光雷达的坐标旋转至和车道平行(这样才能方便的使用激光雷达点云中的y坐标
        # 因为旋转后的坐标系的y值就为激光雷达到两边车道的距离
        angle_left = calcu_angle_between_car_lane(lane_data.left[0], lane_data.left[1])
        angle_right = calcu_angle_between_car_lane(lane_data.right[0], lane_data.right[1])
        angle_average = calcu_average_angle(angle_left, angle_right)
        rotate = -angle_average

        # 旋转激光雷达坐标系，把点云坐标放入新的坐标系中方便计算
        point_left_0 = rotate_coordinate(rotate, lane_data.left[0])
        point_left_1 = rotate_coordinate(rotate, lane_data.left[1])
        point_right_0 = rotate_coordinate(rotate, lane_data.right[0])
        point_right_1 = rotate_coordinate(rotate, lane_data.right[1])

        # 得出左右车道线距离车的平均距离
        y_left_average = (point_left_0.y + point_left_1.y) / 2
        y_right_average = (point_right_0.y + point_right_1.y) / 2

        # 计算出车辆距离目标先距离， 道路宽度，方向盘
        distance_for_target_line, road_width = calcu_distance_for_target_line(y_left_average, y_right_average, self._target_line)
        wheel_degree = calcu_wheel_degree(self._wheel_degree_scale, angle_average, distance_for_target_line, road_width)

        return wheel_degree