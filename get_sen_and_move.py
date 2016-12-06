#!usr/bin/env python
#coding:utf-8

"""
gazeboのラズパイマウスシミュレータを動かすためのプログラム
センサ値をsubscribeして、その値を処理してロボットに速度をpublishする
"""

import rospy
from raspimouse_ros.msg import LightSensorValues
from geometry_msgs.msg import Twist

rospy.init_node("getsen_and_move")
pub = rospy.Publisher("/raspimouse/diff_drive_controller/cmd_vel",Twist,queue_size = 10)

#-------------------------------------------------------------------------#
# 関数:callback
# センサ値をsubscribeして速度をpublishする一番重要な関数
#-------------------------------------------------------------------------#
def callback(message):
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
    
#-------------------------------------------------------------------------#
# プログラムのメインの繰り返し部分
#-------------------------------------------------------------------------#
sub = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,callback)


rospy.spin()
