# -*- coding:utf-8 -*-
"""
GNSS数据的容器
@author: QinYu TianHao
"""
from .. import navigation_map as nmap
import math


def msg_to_gnssdata(gnss_msg):
    """
    读取gnss传回的数据
    把经纬度和罗盘角以及gnss状态码转换
    为xy坐标和xy坐标下的航向角和数据是否可用标志
    
    参数:
        gnss_msg: gnss的ROS消息
    返回:
        gnss_data: GnssData数据对象
    """
    # -----------------------------------------------
    # 此区间的代码需要根据ROS消息结构体来改变
    latitude         = gnss_msg.linear_acceleration.x
    longitude        = gnss_msg.linear_acceleration.y   
    compass_yaw      = gnss_msg.angular_velocity.x
    gnss_status_code = gnss_msg.angular_velocity.z
    # -----------------------------------------------

    def compass_yaw_to_xy_yaw(compass_yaw):
        '''
        因gnss是罗盘角，需要转换为投影坐标系中和x正半轴的夹角
        
        返回:
            xy_yaw: 投影坐标系中和x正半轴的夹角
        '''
        xy_yaw  = math.radians(360 - compass_yaw)
        return xy_yaw


    def gnss_status_validity(gnss_status_code):
        '''
        根据设备说明，只有当状态码为4时的定位精度才是正常的

        返回:
            gnss信号是否可用标志
        '''
        if gnss_status_code == 4:
            return True
        else:
            return False

    # 把经纬度转换为投影坐标（平面坐标）
    # 把罗盘角转换为投影坐标中和x正半轴的夹角
    # 把gnss状态码转换为信号是否可用的标志
    x, y  = nmap.geographic_to_projected(latitude, longitude)
    xy_yaw = compass_yaw_to_xy_yaw(compass_yaw)
    gnss_usable = gnss_status_validity(gnss_status_code)

    # 使用转换后的投影坐标和夹角初始化对象
    gnss_data = GnssData(x, y, xy_yaw, gnss_usable)

    return gnss_data


class GnssData(object):
    """
    用来存储接收的gnss数据的类

    x: 投影坐标x
    y: 投影坐标y
    yaw: 投影坐标航向角 
    gnss_usable: 数据是否可用
    """

    def __init__(self, x=0, y=0, yaw=0, gnss_usable=False):
        """
        初始化gnss数据类属性

        参数:
            x: 投影坐标x
            y: 投usab影坐标y
            yaw: 投影坐标航向角 
            gnss_le: 数据是否可用
        """
        self._x = x
        self._y = y
        self._yaw = yaw
        self._gnss_usable = gnss_usable


    def __str__(self):
        self_class = type(self)
        return '<object:{}> x:{} y:{} yaw:{} usalbe:{}'.format(self_class.__name__, self.x, self.y, self.yaw, self.usable)


    def __call__(self, x, y, yaw, gnss_usable):
        self._x = x
        self._y = y
        self._yaw = yaw
        self._gnss_usable = gnss_usable


    def copy(self):
        """
        复制自身对象返回一个值相同的新对象
        
        返回:
            new_self_object: 值相同的新对象
        """
        self_class = type(self)
        new_self_object = self_class(self.x, self.y, self.yaw, self.usable)

        return new_self_object


    @property
    def usable(self):
        return self._gnss_usable


    def get(self):
        return self.x, self.y, self.yaw


    @property
    def x(self):
        return self._x


    @property
    def y(self):
        return self._y


    @property
    def yaw(self):
        return self._yaw


if __name__ == "__main__":
    """
    测试正常
    """
    pass