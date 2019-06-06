# -*- coding:utf-8 -*-
"""
小配件模块
@author: QinYu TianHao
"""
import math
import numpy
import time


def change_system(theta, a, b, x, y):
    """
    转换坐标系，并求出新坐标系中点的坐标

    参数：
        theta : 坐标系旋转的角度
        a     : 坐标系x方向的平移量
        b     : 坐标系y方向的平移量
        x     : 原坐标系点的 x 坐标
        y     : 原坐标系点的 y 坐标

    返回：
        x1    : 旋转后坐标系后点的 x 坐标(点集)
        y1    : 旋转后坐标系后点的 y 坐标(点集)
    """
    
    # 检查输入的点是不是序列
    try:
        len(x)
    except:
        # 不是序列
        x = x - a   # 先平移
        y = y - b

        x1 = x * math.cos(theta) + y * math.sin(theta)  # 再旋转
        y1 = y * math.cos(theta) - x * math.sin(theta)

        return x1, y1
    else:
        # 如果是序列 分别求出每个点在新坐标系下的新坐标
        x1 = []
        y1 = []
        for ix, iy in zip(x, y):
            ix = ix - a     # 先平移
            iy = iy - b

            x1.append(ix * math.cos(theta) + iy * math.sin(theta))  # 再旋转
            y1.append(iy * math.cos(theta) - ix * math.sin(theta))
        
        return x1, y1


def trans_wheel_degree_format(wheel_degree_input, model='target', wheel_degree_offset=-13):
    '''
    转换方向盘角度为can通信的格式

    参数:
        model: 选择can通信模式的格式(不同模式的格式不相同)
        wheel_degree_offset: 方向盘偏移校准量，确保车轮回中
    返回:
        转换格式后的方向盘角度
    '''
    models = ('target',)
    assert model in models

    if model == 'target':
        # CAN 控制方向盘的目标模式,目标模式 1000 是方向盘居中.所以需要加上 1000
        # 因方向盘向左偏了 13 度,所以减去 13 度来矫正
        wheel_degree_output = wheel_degree_input + 1000 + wheel_degree_offset
        return wheel_degree_output
    else:
        raise ValueError('<func:trans_wheel_degree_format> pattern incorrect')


def EZdata(controlmsg, name, value):
    """
    方便控制数据输入到ROS消息数据中

    参数:
        carmsg: 自定义ROS的消息类型
        name  : 需要控制的模块的名称(s:方向盘, t:油门, b:刹车, g:档位)
        value : 控制量
    
    返回: None
    """
    # 设定通用报头属性
    controlmsg.SendType   = 1
    controlmsg.RemoteFlag = 0
    controlmsg.ExternFlag = 0

    if name == 's':                 # steering 方向盘
        controlmsg.DataLen = 4
        controlmsg.ID = 0x6c1           # 设定 can ID
        controlmsg.Data.append(2)       # 控制方向盘中,默认使用目标模式 1000 是方向盘回中
        controlmsg.Data.append(value)   # 方向盘转角
        controlmsg.Data.append(2)       # 转动速度, 0:慢速 1:中速 2:快速 默认快速
    elif name == 't':               # throttle 油门
        controlmsg.ID = 0x6c3  
        controlmsg.Data.append(value)   # 暂时未使用,为编写解析函数
    elif name == 'b':               # brake 刹车
        controlmsg.DataLen = 3
        controlmsg.ID = 0x6c5
        controlmsg.Data.append(value)   # 完全松开 0-8 完全踩下 
    elif name == 'g':               # gearbox 变速器
        controlmsg.DataLen = 3 
        controlmsg.ID = 0x6c7
        controlmsg.Data.append(value)   # 0:P 1:R 2:N 3:D
    elif name == 'n':               # 空消息
        controlmsg.ID = 0x0
    else:
        # 如果输入的模块名不正确,抛出异常
        raise ValueError("{} must be in ('s', 't', 'b', 'g')".format(name))


class ShowMessage(object):
    """
    方便调试使用的标准输出
    可设定显示驻留时间和显示开关
    """

    def __init__(self, dely=0, is_display=True):
        """
        初始化

        参数:
            dely: 显示后要暂停多少秒
            is_display: 是否开启显示(默认开启)
        """
        assert isinstance(dely, (float, int))
        assert isinstance(is_display, bool)
        self._dely = dely
        self._is_display = is_display


    def show(self, message, dely=None, is_display=None):
        """
        显示标准输出
        这个方法中设定的显示驻留时间和显示开关有最高优先级
        如果不设定会统一使用类初始化时的参数
        如果设定就会单独使用这个方法输入的参数

        参数:
            dely: 显示后要暂停多少秒
            is_display: 是否开启显示(默认开启)
        """
        if dely is None:
            current_dely = self._dely
        else:
            assert isinstance(dely, (float, int))
            current_dely = dely

        if is_display is None:
            current_is_display = self._is_display
        else:
            assert isinstance(is_display, bool)
            current_is_display = is_display

        if current_is_display:
            print(message)
        
        if current_dely:
            time.sleep(current_dely)