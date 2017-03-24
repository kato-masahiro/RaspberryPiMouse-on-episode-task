#!/usr/bin/env python
#coding:utf-8

import PFoE
import rospy
import os
import random
import math
import sys

from raspimouse_ros.msg import LightSensorValues
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

def sensors_callback(message):
    rospy.loginfo("message:%s",message)
    rf = message.right_forward
    rs = message.right_side
    ls = message.left_side
    lf = message.left_forward
    sen_sum = rf + rs + ls + lf
    rospy.loginfo("sum:%d",sen_sum)

    vel = Twist()
    if sen_sum <= 2000:
        vel.linear.x = 0.1
    else:
        vel.linear.x = -0.1
    pub.publish(vel)

def position_callback(message):
    """
    ロボットの現在位置をsubscribeする関数
    """
    x = message.pose[-1].position.x
    y = message.pose[-1].position.y
#   rospy.loginfo("x:%d",x);
#   rospy.loginfo("y:%d",y);

rospy.init_node("particle_filter_on_episode");
pub = rospy.Publisher("/raspimouse/diff_drive_controller/cmd_vel",Twist,queue_size = 10);
sub1 = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,sensors_callback);
sub2 = rospy.Subscriber("/gazebo/model_states",ModelStates,position_callback);
rospy.spin()
