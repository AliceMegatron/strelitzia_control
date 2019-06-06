# -*- coding:utf-8 -*-
"""
目标反馈器
@author: QinYu TianHao
"""
class ObjectFeedback(object):
    """
    对目标检测的障碍物反馈信息
    设置左中右三个警戒区，反馈警戒区中的障碍物情况
    
    length: 长\n
    width: 宽
    """

    def __init__(self, length=7, width=2):
        """
        初始化类，设定警戒区域长和宽

        参数:
            length: 长
            width: 宽
        """
        half_width = width / 2

        self._zone_len = length
        self._zone_wid = (-half_width, half_width)


    def _is_in_mid_zone(self, x, y):
        """
        用于判断目标是否在中间的警戒区，中间的警戒区主要是车辆行驶正前方

        参数:
            x: 目标的点云x坐标
            y: 目标的点云y坐标
        返回:
            如果在警戒区返回True,不在警戒区返回False
        """
        if x <= self._zone_len:
            if y >= self._zone_wid[0] and y <= self._zone_wid[1]:
                return True
        return False


    def _is_in_left_zone(self, x, y, y_offset=1.5):
        """
        用于判断目标是否在左边的警戒区，左边的警戒区主要是车辆行驶左前方用于左转时判断障碍物

        参数:
            x: 目标的点云x坐标
            y: 目标的点云y坐标
        返回:
            如果在警戒区返回True,不在警戒区返回False
        """
        if x <= self._zone_len:
            if y >= self._zone_wid[0] + y_offset and y <= self._zone_wid[1] + y_offset:
                return True
        return False


    def _is_in_right_zone(self, x, y, y_offset=-1.5):
        """
        用于判断目标是否在右边的警戒区，右边的警戒区主要是车辆行驶右前方用于右转时判断障碍物

        参数:
            x: 目标的点云x坐标
            y: 目标的点云y坐标
        返回:
            如果在警戒区返回True,不在警戒区返回False
        """
        if x <= self._zone_len:
            if y >= self._zone_wid[0] + y_offset and y <= self._zone_wid[1] + y_offset:
                return True
        return False


    def mid_zone_count(self, object_data):
        """
        计算中间区域有多少个障碍物
        
        参数:
            object_data: 检测出的对象点云数据
        返回:
            返回在区域内障碍物数量，最少是0
        """
        alert_zone_list = []
        for iobject in object_data:
            if self._is_in_mid_zone(iobject.x, iobject.y):
                alert_zone_list.append(iobject)

        return len(alert_zone_list)


    def is_zone_barrier(self, object_data):
        """
        警戒区是否有障碍物
        
        参数:
            object_data: 检测出的对象点云数据
        返回:
            返回左中右三个警戒区是否有障碍物的元组 有是True 无是False(分别用于左转预警，前方预警， 右转预警)
        """
        left = []
        mid = [] 
        right = []

        for iobject in object_data:
            if self._is_in_left_zone(iobject.x, iobject.y):
                left.append(iobject)
            if self._is_in_mid_zone(iobject.x, iobject.y):
                mid.append(iobject)
            if self._is_in_right_zone(iobject.x, iobject.y):
                right.append(iobject)

        left_barrier = False if len(left) == 0 else True
        mid_barrier = False if len(mid) == 0 else True
        right_barrier = False if len(right) == 0 else True

        return left_barrier, mid_barrier, right_barrier