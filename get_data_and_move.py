#!/usr/bin/env python
# coding: UTF-8

"""
gazeboのラズパイマウスシミュレータを動かすためのプログラム

クラスを使って書く

1.マウスのセンサ値をsubscribe
2.マウスの絶対座標値をsubscribeして画面出力
3.マウスの前に壁があれば左旋回、なければ直進という動きをpublish
(壁があるかどうかの判定はセンサ値の合計で行う)

"""
 
import rospy
from raspimouse_ros.msg import LightSensorValues
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
 
class Controller:
    #各センサの値
    rf = 0
    rs = 0
    ls = 0
    lf = 0

    vel = Twist()

    # publisher,subscriber等の宣言
    def __init__(self):
        rospy.init_node("get_data_and_move")
        self.sub1 = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,self.callback1)
        self.sub2 = rospy.Subscriber("/gazebo/model_states",ModelStates,self.callback2)
        self.pub = rospy.Publisher("/raspimouse/diff_drive_controller/cmd_vel",Twist,queue_size = 10)

    # センサ値を獲得する関数
    def callback1(self,message):
        self.rf = message.right_forward
        self.rs = message.right_side
        self.ls = message.left_side
        self.lf = message.left_forward

    # 絶対座標値を表示する関数
    def callback2(self,message):
        rate = rospy.Rate(10)
        x_coord = message.pose[-1].position.x
        y_coord = message.pose[-1].position.y
        print "---"
        print "x:",x_coord
        print "y:",y_coord
        rate.sleep()

    # センサ値を処理してcmd_velをpublishする関数
    def calcurater(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.rf + self.rs + self.ls + self.lf <= 4000: #壁がないと判断
                self.vel.linear.x = 0.1
                self.vel.angular.z = 0.0
            else:
                self.vel.angular.z = 1.0
                self.vel.linear.x = 0.0
            self.pub.publish(self.vel)
            rate.sleep()
 
if __name__ == '__main__':
    controller = Controller()
    controller.calcurater()
    rospy.spin()
