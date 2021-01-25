#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division

# Python imports
import numpy as np
import scipy.io as sio
import os, sys, time

# ROS imports
import rospy
import ros_numpy
# tf imports 
import tf 
from tf import transformations
import std_msgs.msg
from rospkg import RosPack
from std_msgs.msg import UInt8

from std_msgs.msg import Header
from pyquaternion import Quaternion
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

import pdb

package = RosPack()
package_path = package.get_path('yolov3_pytorch_ros')


def transform_xyz_points(trfm_mat, cloud_array, remove_nans=True, dtype=np.float):
    '''
    '''
    if remove_nans:
        #pdb.set_trace()
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z']) & np.isfinite(cloud_array['intensity'])
        cloud_array = cloud_array[mask]

    # now let us transform the points 
    xyz = tuple(np.dot(trfm_mat, np.array([cloud_array['x'], cloud_array['y'], cloud_array['z'], 1.0], dtype=object)))[:3]
    points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
    points[...,0] = xyz[0]
    points[...,1] = xyz[1]
    points[...,2] = xyz[2]
    points[...,3] = cloud_array['intensity']
    # print("points: {0} ".format(points))

    return points 

def xyz_array_to_pointcloud2(points_sum, msg_in):
    '''
    Create a sensor_msgs.PointCloud2 from an array of points.
    '''
    msg = PointCloud2()
    msg.header.stamp = msg_in.header.stamp
    msg.header.frame_id = "rs32"
    msg.height = 1 
    msg.width = points_sum.shape[0]
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = points_sum.shape[0]
    msg.is_dense = int(np.isfinite(points_sum).all())
    msg.data = np.asarray(points_sum, np.float32).tobytes()
    return msg

# Converter 
class RS16_to_RS32_TFManager():
    def __init__(self):
        # Load image parameter and confidence threshold
        self.rs16_topic = rospy.get_param('~rs16_topic', '/ns1/rslidar_points')
        self.rs32_topic = rospy.get_param('~rs32_topic', '/ns2/rslidar_points')

        print("we should set default value for the tf: rs16 to rs32 if they are static")
        self.trans = (-0.314, -0.0155, -0.261) # x, y, z  
        self.rot_quaternion = (-0.0117, -0.00218, 1.0, -0.00688) # (x, y, z, w)
        self.trfm_matrix = self.tm_from_trans_rot(self.trans,self.rot_quaternion)

        # Load publisher topics
        self.pub_pc_rs16_under_rs32_topic = rospy.get_param('~rs16_to_rs32_topic')

        rospy.loginfo("Start converting pc in rs16 to rs32")

        # Define subscribers
        self.pc_rs16_sub = rospy.Subscriber(self.rs16_topic, PointCloud2, self.rs16_callback, queue_size = 1, buff_size = 2**24)
        self.pc_rs32_sub = rospy.Subscriber(self.rs32_topic, PointCloud2, self.rs32_callback, queue_size = 1, buff_size = 2**24)

        # Define publishers
        self.pub_ = rospy.Publisher(self.pub_pc_rs16_under_rs32_topic, PointCloud2, queue_size=10)
        rospy.loginfo("Launched node for tf converter")

        # tf listener  
        self.listener = tf.TransformListener(rospy.Duration(10.0))
        # Spin
        rospy.spin()

    def tm_from_trans_rot(self, translation, rotation):
        """
        :param translation: translation expressed as a tuple (x,y,z)
        :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
        :return: a :class:`numpy.matrix` 4x4 representation of the transform     
        Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
        """
        return np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    def rs16_callback(self, msg):
        # related data info 
        msg_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg) # msg is of type pointcloud2 
        # transformed data 
        # monitor tf to get the transformation matrix 
        try:
            (trans,rot_quaternion) = self.listener.lookupTransform('rs16', 'rs32', rospy.Time(0))
            # print("trans: {0}".format(trans)) 
            # print("rot: {0}".format(rot_quaternion))
            # make transformation matrix 
            trfm_matrix = self.tm_from_trans_rot(trans,rot_quaternion)
            trans_points = transform_xyz_points(trfm_matrix, msg_cloud, True) # size: N x 4 (last row is (0, 0, 0, 1)), so it will maintrain its intensity, format: (x, y, z, intensity)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # set it to default value 
            print("using default value since the transform is static for the two lidars")
            # use default value 
            trans_points = transform_xyz_points(self.trfm_matrix, msg_cloud, True) # size: N x 4 (last row is (0, 0, 0, 1)), so it will maintrain its intensity, format: (x, y, z, intensity)

        # msg is passed in to set the meta information 
        msg_transformed = xyz_array_to_pointcloud2(trans_points, msg)
        self.pub_.publish(msg_transformed)

    def rs32_callback(self, msg):
        pass 

if __name__=="__main__":
    # Initialize node
    rospy.init_node("RS16_to_RS32_TFManager_Node")

    dm = RS16_to_RS32_TFManager()

