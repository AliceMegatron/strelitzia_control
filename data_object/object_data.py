# -*- coding:utf-8 -*-
"""
目标点云数据的容器
@author: QinYu TianHao
"""
from . import point_data as cpd


def msg_to_objectdata(object_msg):
    """
    读取目标对象

    参数:
        object_msg: 目标信息点云ROS消息
    返回:
        目标信息对象
    """
    # ----------------------------------------------------------------------------------------------
    # 此区间的代码需要根据ROS消息结构体来改变
    object_cloud_points_list = []
    for idata in object_msg.distance_points:
        # 因为实际检测中，远距离的目标偶发会被返回0的距离值，这里是对这种0值进行过滤
        if idata.x > 0:
            object_cloud_points_list.append(ObjectPoint3D(idata.flag, idata.x, idata.y, idata.z))
        else:
            continue

    # 当list中有数据，说明有目标被检测到
    if len(object_cloud_points_list) == 0:
        object_usalbe = False
    else:
        object_usalbe = True
    # ----------------------------------------------------------------------------------------------

    return ObjectData(tuple(object_cloud_points_list), object_usalbe)
# -*- coding:utf-8 -*-


class ObjectPoint3D(cpd.Point3D):
    """
    用于存储目标检测出的点云数据，比普通点云数据多出了类别属性
    """

    def __init__(self, kind, x, y, z):
        super(ObjectPoint3D, self).__init__(x, y, z)
        self._kind = kind


    @property
    def kind(self):
        return self._kind


    def get(self):
        return self.kind, self.x, self.y, self.z


    def __str__(self):
        self_class = type(self)
        return "<object:{}> kind:{} x:{} y:{} z:{}".format(self_class.__name__, self.kind, self._x, self._y, self._z)


class ObjectData(object):
    """
    目标信息类
    """

    def __init__(self, object_cloud_points_tuple=(), object_usalbe=False):
        """
        初始化目标信息属性
        参数:
            object_cloud_points_tuple: 目标点云点集
            object_usalbe: 目标信息是否可用
        """
        self._object_cloud_points_tuple = object_cloud_points_tuple
        self._object_usalbe = object_usalbe


    def __iter__(self):
        '''
        响应for等迭代操作
        '''
        return (point for point in self._object_cloud_points_tuple)
        

    def __len__(self):
        '''
        响应len返回对象元素长度
        '''
        return len(self._object_cloud_points_tuple)


    def __str__(self):
        str_out = ''
        for object_cloud_point in self._object_cloud_points_tuple:
            str_out += "{}".format(object_cloud_point) + "\n"
        return str_out


    @property
    def usable(self):
        """
        可用属性返回对象数据是否可用
        """
        return self._object_usalbe


    def copy(self):
        """
        复制自身对象
        """
        self_class = type(self)
        new_self_object = self_class(self._object_cloud_points_tuple, self.usable)
        return new_self_object