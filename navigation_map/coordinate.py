# -*- coding:utf-8 -*-
"""
地图坐标点容器
@author: QinYu TianHao
"""

# 测试时取消注释，非测试时注释
# import sys  
# reload(sys)
# sys.setdefaultencoding('utf-8') 

import numpy as np
import math
import numbers
import matplotlib.pyplot as plt
import copy


def geog_to_proj(geographic_coordinate_obj):
    """
    经纬度坐标对象转换为投影坐标对象(已移至类方法中，此函数暂时保留)

    参数:
        geographic_coordinate_obj: 经纬度坐标对象
    返回:
        projected_coordinate_obj： 投影坐标对象
    """
    projected_coordinate_obj = ProjectedCoordinate()

    for lat, lon in geographic_coordinate_obj:
        x, y = geographic_to_projected(lat, lon)
        projected_coordinate_obj.append(x, y)

    return projected_coordinate_obj


def geographic_to_projected(latitude, longitude):
    """
    经纬度转换平面xy坐标的函数,转换后的单位是米,x轴朝南

    参数:
        latitude: 纬度
        longitude: 经度
    返回:
        x: x坐标
        y: y坐标
    """

    def change_to_radian(input_value):
        # 把维度的小数部分和整数部分分开
        input_value_float, input_value_int_degree = math.modf(input_value)
        # 把小数部分转换为度数
        input_value_float_degree = input_value_float * 100 / 60
        # 得到转换为度数后的纬度值
        input_value_degree = input_value_int_degree + input_value_float_degree
        # 把纬度值转换为弧度
        output_value_radian = input_value_degree / 180 * math.pi

        return output_value_radian


    # 设定地球赤道半径和极地半径
    equatorial_radius = 6378136.49
    polar_radius = 6356755.00
    # 求出地球赤道半径和极地半径的平方值
    equatorial_radius_square = equatorial_radius**2
    polar_radius_square = polar_radius**2
    # 对输入的经纬度坐标格式xxxx.xxx统一到xxx.xxxx格式
    latitude  = latitude  / 100
    longitude = longitude / 100
    # 设定转换后的原点坐标
    x_origin = 5486044.985049794
    y_origin = 3242390.0422246247
    # x_origin 处的经度转换为弧度
    x_origin_longitude = 103.5533365075
    x_origin_longitude_radian = change_to_radian(x_origin_longitude)

    # 把输入的纬度转换为弧度
    latitude_radian = change_to_radian(latitude)
    # 通过纬度求出计算x坐标的中间值
    tgBO = math.tan(latitude_radian)
    ctgBO = 1 / math.tan(latitude_radian)
    x_latitude = equatorial_radius_square / (equatorial_radius_square + polar_radius_square * tgBO * tgBO)**0.5
    y_latitude = polar_radius_square / (polar_radius_square + equatorial_radius_square * ctgBO * ctgBO)**0.5
    # 求出转换后的x坐标
    x = ((x_latitude - x_origin)**2 + (y_latitude - y_origin)**2)**0.5
    
    # 输入的经度转换为弧度
    longitude_radian = change_to_radian(longitude)
    # 通过经度求出计算y坐标的中间值经度差
    Lob = longitude_radian - x_origin_longitude_radian
    # 求出转换后的y坐标
    y = Lob * x_origin

    return x, y



class MapCoordinate(object):
    '''
    地图坐标点基类
    地图坐标点的容器
    '''

    def __init__(self, input_X=None, input_Y=None):
        '''
        初始化地图坐标点类对象
        有三种初始化方式
            初始化空对象
            使用 list 数据初始化对象
            使用 int 或 float 数据初始化对象

        参数:
            input_X: 地图坐标点 x 坐标，可以是(None, int, float, list)
            input_Y: 地图坐标点 y 坐标，可以是(None, int, float, list)
        '''
        if input_X is None and input_X is None:
            self._X = []
            self._Y = []
            self._len = 0
        elif isinstance(input_X, list) and isinstance(input_Y, list):
            X_len = len(input_X)
            Y_len = len(input_Y)
            if X_len == Y_len:
                self._X = copy.copy(input_X)
                self._Y = copy.copy(input_Y)
                self._len = X_len
            else:
                raise ValueError('{self_class.__name__} '
                    'input_X length should equal input_Y length'.format(self_class=type(self)))
        elif isinstance(input_X, (int, float)) and isinstance(input_Y, (int, float)):
            self._X = [input_X]
            self._Y = [input_Y]
            self._len = 1
        else:
            raise TypeError('{self_class.__name__} '
                'input_X value and input_Y value should be int float list'.format(self_class=type(self)))


    def __len__(self):
        '''
        响应 len() 函数的特殊函数

        返回:
            self.len: 地图坐标点对象的数据长度
        '''
        return self._len


    def __iter__(self):
        '''
        响应迭代操作

        返回:
            依次返回坐标中的每个元素
        '''
        return (coordinate for coordinate in zip(self._X, self._Y))


    def __getitem__(self, index):
        '''
        响应切片和索引操作

        返回:
            切片后的新对象
            或是索引后的值
        '''
        self_class = type(self)

        if isinstance(index, slice):
            return self_class(self._X[index], self._Y[index])
        elif isinstance(index, numbers.Integral):
            return self._X[index], self._Y[index]
        else:
            msg = '{self_class.__name__} indices must be integers'
            raise TypeError(msg.format(self_class=self_class))
        

    def _check_len(self):
        '''
        对 self.len 做检查的函数
        确保在计算时数据长度不会越界
        '''
        if self._len == 0:
            return False
        elif self._len > 0:
            return True
        else:
            msg = '{self_class.__name__} length less than zero'
            raise IndexError(msg.format(self_class=type(self)))

    
    def append(self, input_x, input_y):
        '''
        单数据增添函数
        把新增的数据放到序列的尾部
        '''
        if isinstance(input_x, (int, float)) and isinstance(input_y, (int, float)):
            self._X.append(input_x)
            self._Y.append(input_y)
            self._len += 1
        else:
            msg = '{self_class.__name__} x type and y type should be int float list'
            raise TypeError(msg.format(self_class=type(self)))


    def extend(self, input_X, input_Y):
        '''
        多数据拼接函数
        把新增的序列数据拼接到序列的尾部
        '''
        if isinstance(input_X, list) and isinstance(input_Y, list):
            X_len = len(input_X)
            Y_len = len(input_Y)
            if X_len == Y_len:
                self._X.extend(input_X)
                self._Y.extend(input_Y)
                self._len += X_len
            else:
                raise ValueError('{self_class.__name__} '
                    'input_X length should equal input_Y length'.format(self_class=type(self)))
        else:
            raise TypeError('{self_class.__name__} '
                'input_X value and input_Y value not list'.format(self_class=type(self)))
        


    def pop(self, index=-1):
        '''
        删除数据
        默认删除序列的最后一个数据，也可以指定 index 删除指定位置的数据

        返回:
            output_x, output_y: 被删除的地图坐标点的 xy 值
        '''
        if self._check_len():
            output_x = self._X.pop(index)
            output_y = self._Y.pop(index)
            self._len -= 1
            return output_x, output_y 
        else:
            raise IndexError('{self_class.__name__} '
                'pop from empty'.format(self_class=type(self)))


    def get(self):
        '''
        获取坐标点序列

        返回:
            self.X: 地图点 x 坐标序列
            self.Y: 地图点 y 坐标序列
        '''
        return self._X, self._Y



class ProjectedCoordinate(MapCoordinate):
    """
    投影坐标类
    投影坐标的容器
    """

    def __init__(self, input_X=None, input_Y=None):
        '''
        初始化投影坐标类
        使用父类 MapPoints 来完成初始化
        '''
        super(ProjectedCoordinate, self).__init__(input_X, input_Y)


    def __str__(self):
        '''
        用于响应 print 函数

        返回:
            打印时的输出内容
        '''
        self_class = type(self)
        return '<object:{}> X:{} Y:{}'.format(self_class.__name__, self.X, self.Y)

    
    @property
    def X(self):
        '''
        获取x坐标列表

        返回:
            x坐标列表
        '''
        return self._X

    
    @property
    def Y(self):
        '''
        获取y坐标列表

        返回:
            y坐标列表
        '''
        return self._Y



class GeographicCoordinate(MapCoordinate):
    """
    经纬度坐标类
    经纬度坐标的容器
    """

    def __init__(self, latitudes=None, longitudes=None):
        '''
        初始化经纬度坐标类
        使用父类 MapPoints 来完成初始化
        '''
        super(GeographicCoordinate, self).__init__(latitudes, longitudes)

    
    def __str__(self):
        '''
        用于响应 print 函数

        返回:
            打印时的输出内容
        '''
        self_class = type(self)
        return '<object:{}> lat:{} lon:{}'.format(self_class.__name__, self.lats, self.lons)


    @property
    def lats(self):
        '''
        获取纬度坐标列表

        返回:
            纬度坐标列表
        '''
        return self._X

    
    @property
    def lons(self):
        '''
        获取经度坐标列表
        
        返回:
            经度坐标列表
        '''
        return self._Y


    def to_projected(self):
        """
        经纬度坐标对象转换为投影坐标对象

        返回:
            projected_coordinate_obj： 投影坐标对象
        """
        projected_coordinate_obj = ProjectedCoordinate()

        for lat, lon in zip(self.lats, self.lons):
            x, y = geographic_to_projected(lat, lon)
            projected_coordinate_obj.append(x, y)

        return projected_coordinate_obj


def read_coordinate(file_path, class_type=ProjectedCoordinate):
    """
    读取存储的坐标函数
    从 .txt 文件中读取坐标
    文件中每行是一对坐标点,坐标中间由空格隔开

    参数:
        route_file_path: 路径点文件读取路径
    返回:
        coordinate： class_type 类型的坐标对象
    """

    print('Reading {}: {}.'.format(class_type.__name__, file_path))
    
    coordinate = class_type()
    with open(file_path, 'r') as f:
        for line in f.readlines():
            line = line.split()
            coordinate.append(float(line[0]), float(line[1]))

    print('Complete read {}.'.format(class_type.__name__))

    return coordinate


if __name__ == '__main__':
    """
    函数功能测试(正常)
    """
    cod1 = read_coordinate('/home/y/文档/CodeHub/Python_Practice/work_space/control_ros/data/Original Cartesian coordinates_1.txt')
    cod2 = read_coordinate('/home/y/文档/CodeHub/Python_Practice/work_space/control_ros/data/Sparse_gps.txt', GeographicCoordinate)

    plt.figure()
    plt.grid(True)
    plt.axis('equal')
    plt.scatter(*cod1.get(), s=1, c='r')
    plt.title('ProjectedCoordinate')

    plt.figure()
    plt.grid(True)
    plt.axis('equal')
    plt.scatter(*cod2.get(), s=1, c='b')
    plt.title('GeographicCoordinate')

    plt.figure()
    plt.grid(True)
    plt.axis('equal')
    cod3 = cod2.to_projected()
    plt.scatter(*cod3.get(), s=1, c='b')
    plt.title('ProjectedCoordinate from GeographicCoordinate')
    plt.show()