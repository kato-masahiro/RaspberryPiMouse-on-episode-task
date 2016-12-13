#!/usr/bin/env python
#coding:utf-8
"""
パーティクルフィルタでエピソード的タスクを学習させる

while(報酬==0):
    センサ値をN回取得して平均を取る
    latest_sen = 平均
    パーティクルを尤度関数を用いて再配置する(報酬は前回得たものを用いる)
    パーティクルの投票で行動を決定する
    前回の報酬、観測、行動をepisode_setに追加
    while(行動終了の条件を満たさない):
        一瞬だけ行動する
        N回センサ値を取得して平均を取る
    報酬を得る
    報酬をlaetst_episodeに追加
"""
import rospy
import os
import random
import math

from raspimouse_ros.msg import LightSensorValues
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

####################################
#     グローバル変数の定義         #
####################################
reward_arm = "right"                  # 報酬が得られる腕。right or left
x = 0.0; y = 0.0                      # ロボットの座標
rf = 0; rs = 0; ls = 0; lf = 0        # センサ値
sensors_val = [0,0,0,0]               # 平均を取るためにrf,rs,ls,lfの和を入れるための変数
latest_sen = [0,0,0,0]                # ロボットが行動決定に用いる最新のセンサ値情報
counter = 0                           # sensors_callbackを何回実行したか
N = 10                                # 何回分のセンサ値の平均を取って利用するか
T = 1                                 # 最新の時間ステップ(いままで経験したエピソードの数+1)
action = ""                           # 行動."f","r","l","s"の3種類(前進、右旋回、左旋回,待機)待機は実際には行われない
moving_flag = False                   # ロボットが行動中かどうかのフラグ
got_average_flag = False              # センサ値が平均値をとっているかどうかのフラグ
end_flag = False                      # 非ゼロ報酬を得たらこのフラグが立って、すべての処理を終わらせる。
fw_threshold = 3000                   # 前進をやめるかどうかの判定に使われる閾値(rf+rs+ls+lf)
turn_threshold = 2000                 # 旋回をやめるかどうかの判定に使われる閾値(rf+lf)
particle = range(1000)                # パーティクルの位置、重みが入るリスト。パーティクルの重みの合計は1
for i in particle:
    particle[i] = [0, 0.001]
latest_episode = [0.0 ,0, 0, 0, 0,""] # 最新のエピソード。報酬値、センサ値、行動。
episode_set = []                    # 過去のエピソードの集合。報酬値、センサ値、行動
alpha = 0.0

epsiron = 10 #ランダムに行動する確率(グリーディではなく)

###########################################################
#    particle,episode_setについてファイルから読み込む     #
###########################################################
if os.path.exists("./particle.txt"):
    f = open("particle.txt","r")
    particle = f.read()
    f.close()
    print "ファイル:particle.txtを読み込みました"

if os.path.exists("./episode_set.txt"):
    f = open("episode_set.txt","r")
    episode_set = f.read()
    f.close
    T = len(episode_set) + 1
    print "ファイル:episode_set.txtを読み込みました"
    print "---episode_set---"
    print episode_set

print episode_set

####################################
#   センサの平均値を求める関数     #
####################################
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
        print "### _sensors_ave_ : ave=",sensors_val
    else:
        got_average_flag = False

########################################
#     ロボットの位置で報酬値を決定     #
########################################
def reward_check(x,y):
    global end_flag
    global latest_episode
    if reward_arm == "right":
        if(x - 0.36) ** 2 + (y + 0.15) ** 2 <= 0.01:
            print "###_reward_check_:ロボットは正解に到達"
            latest_episode[0] = 1.0
            end_flag = True
        elif(x - 0.36) ** 2 + (y - 0.15) ** 2 <= 0.01:
            print "###_reward_check_:ロボットは不正解に到達"
            latest_episode[0] = -1.0
            end_flag = True
        else:
            print "###_reward_check_:ロボットは行動を続行"
            latest_episode[0] = 0.0
            end_flag = False

    elif reward_arm == "left":
        if(x - 0.36) ** 2 + (y - 0.15) ** 2 <= 0.01:
            print "###_reward_check_:ロボットは正解に到達"
            latest_episode[0] = 1.0
            end_flag = True
        elif(x - 0.36) ** 2 + (y + 0.15) ** 2 <= 0.01:
            print "###_reward_check_:ロボットは不正解に到達"
            laetst_episode[0] = -1.0
            end_flag = True
        else:
            print "ロボットは行動を続行"
            latest_episode[0] - 0.0
            end_flag = False

#############################################################
#     パーティクルの尤度を求める関数                        #
#  !! この関数の実行後、particle[][1]の和は必ず1になる !!   #
#############################################################
def sensor_update():
    global alpha
    global particle
    alpha = 0.0
    if T != 1:
        print "###_sensor_update_:episode_set",episode_set
        for i in range(1000):
            if episode_set[ particle[i][0] ][0] == latest_episode[0]: #過去のエピソードで得られた報酬が現在のものと等しい
                l1 = math.fabs(latest_episode[1] - episode_set[ particle[i][0] ][1])
                l2 = math.fabs(latest_episode[2] - episode_set[ particle[i][0] ][2])
                l3 = math.fabs(latest_episode[3] - episode_set[ particle[i][0] ][3])
                l4 = math.fabs(latest_episode[4] - episode_set[ particle[i][0] ][4])
                particle[i][1] = 0.5 ** (l1+l2+l3+l4)
            else:
                particle[i][1] = 0.0
    elif T == 1:
        for i in range(1000):
            particle[i][1] = 0.001

    #alphaも求める
    for i in range(1000):
        alpha += particle[i][1]

    #alphaで正規化
    if alpha > 0.0:
        for i in range(1000):
            particle[i][1] /= alpha
    else:
        for i in range(1000):
            particle[i][1] = 0.001

########################################################
#   尤度に基づきパーティクルをリサンプリングする関数   #
#  パーティクルがいないエピソードができないように注意  #
########################################################
def motion_update(particle):
    if T != 1: #重みに基づいてリサンプリング
        likelihood = range(len(episode_set))
        for i in range(len(likelihood)):#パーティクルの尤度からエピソードの尤度(likelihood)を求める
            likelihood[i] = 0.0
            for ii in range (1000):
                if particle[ii][0] == i:
                    likelihood[i] += particle[ii][1]
        #likelihoodの分布に基づき900個のパーティクルを配置する
        for i in range(900):
            seed = random.randint(1,100)
            for ii in range(len(likelihood)):
                seed -= likelihood[ii] * 100
                if seed <= 0:
                    particle[i][0] = ii
                    break
        #likelihoodとは無関係に100個のパーティクルを配置する
        for i in range(900,1000):            
           seed = random.randint(0,len(episode_set)-1)
           particle[i][0] = seed

    elif T == 0: 
        for i in range(1000):
            particle[i][0] = 0

    return particle

######################################
#   投票によって行動を決定する関数   #
#!!終了時の行動stayを追加した。この行動が評価されることが無いようにチェックする必要がある
#追記:stay行動を取ったエピソードにも票が入る。これは仕方がないので、入った上でうまく処理するように変更する必要あり
######################################
def decision_making(particle):
    if T == 1:#まだどんなエピソードも経験していないのでランダムに行動させる
        return random.choice("frl")
        
    vote = range(1000)#各パーティクルが自分の所属しているエピソードに対して持つ評価
    for i in range(1000):
        vote[i] = 0.0
    else:#各パーティクルが投票で決める
        for i in range (1000):
            distance = 0 #パーティクルがいるエピソードとその直後の非ゼロ報酬が得られたエピソードとの距離
            non_zero_reward = 0.0
            for l in range(len(episode_set) - particle[i][0] - 1):
                distance += 1
                if episode_set[ particle[i][0] + distance ][0] != 0.0:
                    non_zero_reward = episode_set[particle[i][0] + distance][0]
                    break
            if non_zero_reward != 0:
                vote[i] = non_zero_reward / distance
            else:
                vote[i] = 0.0

    #voteに基づく行動決定。voteの合計がゼロやマイナスになる可能性がある点に注意
    got = {"f":0.0,"r":0.0,"l":0.0,"s":0.0} # 得票数が入るディクショナリ
    for i in range(1000):
        if int(vote[i]) != 0:
            got [episode_set[particle[i][0]] [5]] += vote[i]
        print "###_decision_making_:得票数=",got
        #グリーディならgotの中で最大の数字を持つもののキーをひとつ返す
        #参考:http://cointoss.hatenablog.com/entry/2013/10/16/123129
        if (random.randint(1,100) > epsiron):
            if max(got.items(),key = lambda x:x[1])[0] == "s":
                return random.choice("frl")
            else:
                return max(got.items(),key = lambda x:x[1])[0]
        else:
            print "###_decision_making_:random_choice"
            return random.choice("frl")

######################################################
#  センサ値が閾値を超えたらmoving_flagをFalseにする  #
######################################################
def stop(action):
    global moving_flag
    print "###_stop_:センサの合計:",sum(sensors_val)
    if action == "f":
        if sum(sensors_val) >= fw_threshold:
            print "### _stop_:行動fは閾値によって中断された"
            moving_flag = False
        else:
            print "### _sotep_:動き続けます"
            moving_flag = True
    else:
        if (sensors_val[0] + sensors_val[3]) >= turn_threshold:
            print "### _stop_:行動r or l は閾値によって中断された"
            moving_flag = False
        else:
            print "### _stop_:動き続けます"
            moving_flag = True

##################################################
#    センサ値をsubscribeするコールバック関数     #
#   main
##################################################
def sensors_callback(message):
    vel = Twist()
    vel.linear.x = 0.0
    vel.angular.z = 0.0

    global rf;global rs;global ls;global lf
    global sensors_val
    global counter
    global latest_sen
    global moving_flag
    global T
    global action

    counter += 1

    # センサデータを読み込む
    rf = message.right_forward
    rs = message.right_side
    ls = message.left_side
    lf = message.left_forward
    sensors_ave() # N回分のセンサ値の平均を取る
    if got_average_flag == True and moving_flag == False:
        print "###_sensors_callback_:リサンプリングとか色々します"
        for i in range(4):
            latest_sen[i] = sensors_val[i]
            sensors_val[i] = 0
            latest_episode[i + 1] = latest_sen[i] # 最新のepisode_setにlatest_senを追加
        reward_check(x,y)
        if end_flag == True:
            #センサ値、行動(stay)を書き込んで色々保存して終了
            latest_episode[5] = "s"
            episode_set.append(latest_episode)
            #episode_set,particle をファイルに書き込んで終了
            f = open("episode_set.txt","a")
            f.write(episode_set)
            f.close()
            f = open("particle.txt","a")
            f.write(particle)
            f.close()
            exit()
        sensor_update() #パーティクル集合の尤度を求める
        motion_update(particle) #尤度に基づきパーティクルの分布を更新する
        action = decision_making(particle) #パーティクルの投票に基づき行動を決定する
        latest_episode[5] = action #最新のepisode_setにactionを追加
        episode_set.append(latest_episode)#一連のエピソードをエピソード集合に追加
        T += 1
        moving_flag = True
    elif got_average_flag == True and moving_flag == True:
        print "###_sensors_callback_:動かします"
        if action == "f":
            vel.linear.x = 0.2
        elif action == "r":
            vel.angular.z = -2.0
        elif action == "l":
            vel.angular.z = 2.0
        stop(action) 

    pub.publish(vel)

#########################################################
#   ロボットの現在位置をsubscribeするコールバック関数   #
#########################################################
def position_callback(message):
    global x;global y
    x = message.pose[-1].position.x
    y = message.pose[-1].position.y

rospy.init_node("particle_filter_on_episode")
pub = rospy.Publisher("/raspimouse/diff_drive_controller/cmd_vel",Twist,queue_size = 10)
sub1 = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,sensors_callback)
sub2 = rospy.Subscriber("/gazebo/model_states",ModelStates,position_callback)
rospy.spin()
