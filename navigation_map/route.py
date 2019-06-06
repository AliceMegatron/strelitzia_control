# -*- coding:utf-8 -*-
"""
路径容器
@author: QinYu TianHao
"""
# import sys  
# reload(sys)
# sys.setdefaultencoding('utf-8') 

import coordinate
# from strelitzia_control.navigation_map import coordinate
import numbers
import matplotlib.pyplot as plt


def build_route(projected_coordinate0, projected_coordinate1, driveway=0):
    """
    通过多个坐标点对象建立路径信息(车道编号从行驶方向右到左增大)

    参数:
        projected_coordinate1: 1号车道gnss点对象
        projected_coordinate2: 2号车道gnss点对象
        driveway: 当前车道号
    返回:
        路径信息对象
    """
    projected_coordinate_tuple = (projected_coordinate0, projected_coordinate1)
    route = Route(projected_coordinate_tuple, driveway)

    return route


class Route(object):
    """
    构建一个有虚拟双车道的路径
    可以通过 driveway_change 方法来切换路径
    """

    def __init__(self, projected_coordinate_tuple=(), driveway=0):
        """
        通过多个坐标点对象建立路径信息(车道编号从行驶方向右到左增大)

        参数:
            projected_coordinate_tuple: 车道gnss点对象元组，每个元素是一条路径点集
            driveway: 当前车道号
         """
        assert isinstance(projected_coordinate_tuple, tuple)
        self.projected_coordinate_tuple = projected_coordinate_tuple
        self.driveway_num = tuple(range(len(self.projected_coordinate_tuple)))
        self._driveway = driveway
        self.current_projected_coordinate = self.projected_coordinate_tuple[self.driveway]


    @property
    def driveway(self):
        """
        返回当前车道编号
        """
        return self._driveway

    @property
    def num_of_driveway(self):
        """
        返回所有车道的数量
        """
        return len(self.driveway_num)


    def change(self, driveway):
        """
        切换路径
        """
        assert isinstance(driveway, numbers.Integral)

        if self.driveway == driveway:
            print('It`s already in the driveway, don`t need to change.')
        else:
            if driveway in self.driveway_num:
                self._driveway = driveway
                self.current_projected_coordinate = self.projected_coordinate_tuple[self._driveway]


    def __len__(self):
        """
        相应len函数
        返回:
            当前路径的元素长度
        """
        return len(self.current_projected_coordinate)


    def __str__(self):
        """
        相应print操作

        返回:
            对象内容字符串
        """
        self_class = type(self)
        return '<object:{}> {}'.format(self_class.__name__, self.projected_coordinate_tuple)


    def __iter__(self):
        '''
        响应迭代操作

        返回:
            依次返回当前路径坐标中的每个元素
        '''
        return (coordinate for coordinate in self.current_projected_coordinate)


    def __getitem__(self, index):
        '''
        响应切片和索引操作

        返回:
            切片后的新对象
            或是索引后的值
        '''
        self_class = type(self)

        if isinstance(index, slice):
            new_projected_coordinate_tuple = (self.projected_coordinate_tuple[0][index], 
                                              self.projected_coordinate_tuple[1][index])
            return self_class(new_projected_coordinate_tuple, self.driveway)
        elif isinstance(index, numbers.Integral):
            return self.current_projected_coordinate[index], self.current_projected_coordinate[index]
        else:
            msg = '{self_class.__name__} indices must be integers'
            raise TypeError(msg.format(self_class=self_class))

    def get(self):
        """
        获取当前使用的坐标点序列
        
        返回:
            当前路径的坐标点序列
        """
        return self.current_projected_coordinate.get()


    def get_all(self):
        """
        获取所有路径坐标点序列

        返回:
            所有路径的坐标点序列
        """
        return self.projected_coordinate_tuple[0].get(), self.projected_coordinate_tuple[1].get()


if __name__ == "__main__":
    """
    函数功能测试(正常)
    """
    cod1 = coordinate.read_coordinate('/home/y/文档/CodeHub/Python_Practice/work_space/control_ros/data/Original Cartesian coordinates_1.txt')
    cod2 = coordinate.read_coordinate('/home/y/文档/CodeHub/Python_Practice/work_space/control_ros/data/Original Cartesian coordinates_2.txt')

    rot = build_route(cod1, cod2, 1)

    plt.figure()
    plt.scatter(*rot.get(), s=1, c='r')
    rot.change(0)
    plt.scatter(*rot.get(), s=1, c='b')
    plt.axis('equal')
    rot_1 = rot[100:200]
    plt.scatter(*rot_1.get(), s=1, c='y')
    plt.show()