# -*- coding:utf-8 -*-
"""
车道线点云数据的容器
@author: QinYu TianHao
"""
from . import point_data as cpd


def msg_to_lanedata(lane_msg):
    """
    读取车道线信息

    参数:
        lane_msg: 车道线点云ROS消息
    返回:
        lane_data: 车道信息对象
    """
    length = len(lane_msg.distance_points)
    if length % 2 != 0:
        raise ValueError('<func:msg_to_lanedata> the lane message array is not even')

    # ------------------------------------------------------------------------------
    # 此区间的代码需要根据ROS消息结构体来改变
    lane_is_valid = lane_msg.is_valid

    left_cloud_points = []
    right_cloud_points = []

    for i, idata in enumerate(lane_msg.distance_points):
        if i < (length // 2):
            left_cloud_points.append(cpd.Point3D(idata.x, idata.y, idata.z))
        else:
            right_cloud_points.append(cpd.Point3D(idata.x, idata.y, idata.z))

    if lane_is_valid:
        lane_usable = True
    else:
        lane_usable = False
    # ------------------------------------------------------------------------------
    
    lane_data = LaneData(left_cloud_points, right_cloud_points, lane_usable)

    return lane_data


class LaneData(object):
    """
    车道信息类

    left_cloud_points: 左车道点云点集(距离车由远到近)
    right_cloud_points: 右车道点云点集(距离车由远到近)
    lane_usable: 车道是否可用
    """

    def __init__(self, left_cloud_points=(), right_cloud_points=(), lane_usable=False):
        """
        初始化车道信息类属性
        
        参数:
            left_cloud_points: 左车道点云点集
            right_cloud_points: 右车道点云点集
            lane_usable: 车道是否可用
        """
        self._left_cloud_points = left_cloud_points
        self._right_cloud_points = right_cloud_points
        self._lane_usable = lane_usable


    @property
    def usable(self):
        return self._lane_usable


    @property
    def left(self):
        return self._left_cloud_points


    @property
    def right(self):
        return self._right_cloud_points


    def copy(self):
        """
        复制自身对象
        """
        self_class = type(self)
        new_self_object = self_class(self.left, self.right, self.usable)
        return new_self_object


"""
将来多车道可以使用LaneData数组或者新写一个多车道对象
"""


if __name__ == "__main__":
    pass