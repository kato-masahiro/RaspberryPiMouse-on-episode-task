#!/usr/bin/env python
#coding:utf-8

"""
gazeboのラズパイマウスシミュレータを動かすためのプログラム

マウスのセンサ値をN回ごとに平均を取って表示する
"""
import rospy
from raspimouse_ros.msg import LightSensorValues
from gazebo_msgs.msg import ModelStates

sum_sensors = [0,0,0,0] #rf,rs,ls,lfの和を入れるための変数
counter = 0
N = 20

rospy.init_node("get_sen_and_position")
def callback(message):
    rf = message.right_forward
    rs = message.right_side
    ls = message.left_side
    lf = message.left_forward

    global sum_sensors
    sum_sensors[0] += rf
    sum_sensors[1] += rs
    sum_sensors[2] += ls
    sum_sensors[3] += lf
    global counter
    counter += 1

    if counter%N == 0:
        counter = 0
        for i in range (4):
            sum_sensors[i] /= N
        print"sum_sensors:",sum_sensors
        sum_sensors = [0,0,0,0]

sub = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,callback)
rospy.spin()
