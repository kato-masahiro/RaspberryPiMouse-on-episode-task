#!/usr/bin/env python
#coding:utf-8
"""
パーティクルフィルタでエピソード的タスクを学習させる

while(報酬==0):
    すべてのパーティクルの位置を時刻tに対して+1ずらす
    センサ値をN回取得して平均を取る
    latest_sen = 平均
    パーティクルの投票で行動を決定する
    while(行動終了の条件を満たさない):
        一瞬だけ行動する
        N回センサ値を取得して平均を取る
    報酬を得る
    パーティクルを尤度関数を用いて再配置する
    latest_sen,行動、報酬をエピソード集合に追加
"""
import rospy
import os
import random
from raspimouse_ros.msg import LightSensorValues
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

####################################
#     グローバル変数の定義         #
####################################
x = 0.0 ;y = 0.0 #ロボットの座標
rf = 0; rs = 0; ls = 0; lf = 0 #センサ値
sensors_val = [0,0,0,0] #平均を取るためにrf,rs,ls,lfの和を入れるための変数
latest_sen = [0,0,0,0] #ロボットが行動決定に用いる最新のセンサ値情報
counter = 0 #sensors_callbackを何回実行したか
N = 10 #何回分のセンサ値の平均を取って利用するか
T = 1 #最新の時間ステップ(いままで経験したエピソードの数+1)
action = "" # 行動."f","r","l"の3種類(前進、右旋回、左旋回)
reward = 0.0 # 報酬
moving_flag = False #ロボットが行動中かどうかのフラグ
got_average_flag = False #センサ値が平均値をとっているかどうかのフラグ
fw_threshold = 3000 #前進をやめるかどうかの判定に使われる閾値(rf+rs+ls+lf)
turn_threshold = 1000 #旋回をやめるかどうかの判定に使われる閾値(rf+lf)
particle = range(1000) #パーティクルの位置、重みが入るリスト。パーティクルの重みの合計は1
for i in particle:
    particle[i] = [0, 0.001]
latest_episode = [ 0, 0, 0, 0,"",0] #最新のエピソード。センサ値、行動、報酬。
episode_set = [[" "]] #過去のエピソードの集合。センサ値、行動、報酬

###########################################################
#    particle,episode_setについてファイルから読み込む     #
###########################################################
if os.path.exists("./particle.txt"):
    f = open("particle.txt","r")
    particle = f.read()
    f.close()
if os.path.exists("./episode_set.txt"):
    f = open("episode_set.txt","r")
    episode_set = f.read()
    f.close
    T = len(episode_set) + 1

print episode_set

########################################
#      センサ値の平均を取る関数        #
########################################
def sensors_ave():
    global rf;global rs;global ls;global lf
    global sensors_val
    global got_average_flag
    sensors_val[0] += rf
    sensors_val[1] += rs
    sensors_val[2] += ls
    sensors_val[3] += lf
    if counter%N == 0:
        got_average_flag = True
        for i in range(4):
            sensors_val[i] /= N
    else:
        got_average_flag = False

######################################################
#   尤度関数でパーティクルをリサンプリングする関数   #
######################################################
#def resampling():
#    if T > 1:
#        likelihood = range(episode_set)
#        for i in len(episode_set):#過去に経験したエピソードの尤度を求める
#            if #a = b or not a == b で分岐するが。。。
#            likelihood[i] = 
            
############################################
#   投票アルゴリズムで行動を決定する関数   #
############################################
def voting(particle):
    vote = range(1000) #各パーティクルが自分の所属しているエピソードに対して持つ評価
    for i in range(vote):
        vote[i] = 0

    if T == 1: #まだどんなエピソードも経験していないのでランダムに行動させる
        return random.choice("frl")
    else:#各パーティクルが投票で決める
        for i in range(1000):
            distance = 0 #パーティクルがいるエピソードとその直後の非ゼロ報酬が得られたエピソードとの距離
            non_zero_reword = 0
            for l in range(len(episode_set) - particle[i][0] - 1):
                distance += 1 
                if episode_set[ particle[i][0] + distance ][5] != 0:
                    non_zero_reword = episode_set[particle[i][0] + distance][5]
                    break
            if non_zero_reword != 0:
                vote[i] = non_zero_reword / distance
            else vote[i] = 0

#######################################
#  パーティクルをスライドさせる関数   #
#######################################
def slide(particle)
    for i in range(1000):
        particle[i][0] += 1
    return particle

##################################################
#    センサ値をsubscribeするコールバック関数     #
##################################################
def sensors_callback(message):
    global rf;global rs;global ls;global lf
    global sensors_val
    global counter
    global latest_sen

    counter += 1

    #センサデータを読み込む
    rf = message.right_forward
    rs = message.right_side
    ls = message.left_side
    lf = message.left_forward
    sensors_ave() #N回分のセンサ値の平均を取る
    if got_average_flag == True and moving_flag == False:
        particle = slide():
        for i in range(4):
            latest_sen[i] = sensors_val[i]
            sensors_val[i] = 0
            latest_episode[i] = latest_sen[i] #最新のepisode_setにlatest_senを追加
        action = voting(particle) #投票で行動を決定する
        print latest_sen,"--->",sum(latest_sen)

#########################################################
#   ロボットの現在位置をsubscribeするコールバック関数   #
#########################################################
def position_callback(message):
    global x;global y
    x = message.pose[-1].position.x
    y = message.pose[-1].position.y

rospy.init_node("particle_filter_on_episode")
sub1 = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,sensors_callback)
sub2 = rospy.Subscriber("/gazebo/model_states",ModelStates,position_callback)
rospy.spin()
