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
import sys

from raspimouse_ros.msg import LightSensorValues
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

args = sys.argv

try:
    reward_arm = args[1]
except IndexError:
    print "実行時に引数として'right'または'left'を指定してください"
    sys.exit()

####################################
#     グローバル変数の定義         #
####################################
p = 1000                               # パーティクルの数
lmd = 12                            #retrospective_resettingの時、いくつのエピソードを残すか
x = 0.0; y = 0.0                      # ロボットの座標
rf = 0; rs = 0; ls = 0; lf = 0        # センサ値
sensors_val = [0,0,0,0]               # 平均を取るためにrf,rs,ls,lfの和を入れるための変数
latest_sen = [0,0,0,0]
counter = 0                           # sensors_callbackを何回実行したか
N = 10                                # 何回分のセンサ値の平均を取って利用するか
T = 1                                 # 最新の時間ステップ(いままで経験したエピソードの数+1)
T0 = 1
action = ""                           # 行動."f","r","l","s"の3種類(前進、右旋回、左旋回,待機)待機は実際には行われない
moving_flag = False                   # ロボットが行動中かどうかのフラグ
got_average_flag = False              # センサ値が平均値をとっているかどうかのフラグ
end_flag = False                      # 非ゼロ報酬を得たらこのフラグが立って、すべての処理を終わらせる。
fw_threshold = 5000                   # 前進をやめるかどうかの判定に使われる閾値(rf+rs+ls+lf)
turn_threshold = 2000                 # 旋回をやめるかどうかの判定に使われる閾値(rf+lf)
alpha_threshold = 0.0                 # retrospective_resettingを行うかどうかの閾値。0.0だと行わない。1.0だと常に行う。
particle = range(p)                # パーティクルの位置、重みが入るリスト。パーティクルの重みの合計は1
for i in particle:
    particle[i] = [0, 1.0/p]
latest_episode = [0.0 ,0, 0, 0, 0,""] # 最新のエピソード。報酬値、センサ値、行動。
episode_set = []                    # 過去のエピソードの集合。報酬値、センサ値、行動
alpha = 0.0

epsiron = 0 #ランダムに行動する確率(グリーディではなく)

###########################################################
#    particle,episode_setについてファイルから読み込む     #
###########################################################
if os.path.exists("./particle.txt"):
    f = open("particle.txt","r")
    particle = f.read()
    particle = eval(particle)
    f.close()
    print "ファイル:particle.txtを読み込みました"

if os.path.exists("./episode_set.txt"):
    f = open("episode_set.txt","r")
    episode_set = f.read()
    episode_set = eval(episode_set)
    f.close
    T = len(episode_set) + 1
    T0 = len(episode_set) + 1
    print "ファイル:episode_set.txtを読み込みました"

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
    else:
        got_average_flag = False

########################################
#     ロボットの位置で報酬値を決定     #
########################################
def reward_check(x,y):
    global end_flag
    global latest_episode
    if reward_arm == "right":
    #   print "###_reward_check_:正解との距離:",(x-0.36) ** 2 + (y + 0.15) ** 2
        if(x - 0.36) ** 2 + (y + 0.15) ** 2 <= 0.005:
            print "###_reward_check_:ロボットは正解に到達"
            f = open("result.txt","a")
            f.write("S")
            f.close()
            latest_episode[0] = 1.0
            end_flag = True
        elif(x - 0.36) ** 2 + (y - 0.15) ** 2 <= 0.005:
            print "###_reward_check_:ロボットは不正解に到達"
            f = open("result.txt","a")
            f.write("F")
            f.close()
            latest_episode[0] = -1.0
            end_flag = True
        else:
    #       print "###_reward_check_:ロボットは行動を続行"
            latest_episode[0] = 0.0
            end_flag = False

    elif reward_arm == "left":
        if(x - 0.36) ** 2 + (y - 0.15) ** 2 <= 0.005:
            print "###_reward_check_:ロボットは正解に到達"
            f = open("result.txt","a")
            f.write("S")
            f.close()
            latest_episode[0] = 1.0
            end_flag = True
        elif(x - 0.36) ** 2 + (y + 0.15) ** 2 <= 0.005:
            print "###_reward_check_:ロボットは不正解に到達"
            f = open("result.txt","a")
            f.write("F")
            f.close()
            latest_episode[0] = -1.0
            end_flag = True
        else:
    #       print "ロボットは行動を続行"
            latest_episode[0] = 0.0
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
        for i in range(p):
            if episode_set[ particle[i][0] ][0] == latest_episode[0] and episode_set[particle[i][0]][5] == episode_set[-1][5]: #過去のエピソードで得られた報酬と取った行動が現在のものと等しい
                l1 = math.fabs(latest_episode[1] - episode_set[ particle[i][0] ][1])
                l2 = math.fabs(latest_episode[2] - episode_set[ particle[i][0] ][2])
                l3 = math.fabs(latest_episode[3] - episode_set[ particle[i][0] ][3])
                l4 = math.fabs(latest_episode[4] - episode_set[ particle[i][0] ][4])
                particle[i][1] = 0.5 ** ((l1+l2+l3+l4) / 4000)
            else:
                particle[i][1] = 0.0
    #        print "###_sensor_update_###:エピソード",particle[i][0],"に存在するパーティクルNo",i,"の尤度は",particle[i][1],"と判定されました。"
    elif T == 1:
        for i in range(p):
            particle[i][1] = 1.0/p

#   print "###_sensor_update_###:過去のエピソード集合 = ",episode_set
#   print "###_sensor_update_###:今回のエピソード = ",latest_episode
#   print "###_sensor_update_###:各パーティクルの尤度 = ",particle

    #alphaも求める
    for i in range(p):
        alpha += particle[i][1]
    print "###_sensor_update_###: 各パーティクルの尤度の平均値α = ",alpha / p

    #alphaで正規化
    if math.fabs(alpha) >= 0.0001:
        for i in range(p):
            particle[i][1] /= alpha
    else:
        for i in range(p):
            particle[i][1] = 1.0/p

#################################################################
#   alphaが閾値より小さい時、retrospective_resettingを行う関数  #
#################################################################
def retrospective_resetting(alpha):
    global episode_set
    global particle
    if alpha < alpha_threshold:
        if len(episode_set) >= lmd:
            print "###_retrospective_resetting_###:条件を満たしているためretrospective_resettingを行う"
            episode_set = episode_set[-lmd::]
            for i in range(p):
                particle[i][0] = random.randint(0,lmd-1)
                particle[i][1] = 1.0/p

########################################################
#   尤度に基づきパーティクルをリサンプリングする関数   #
#  パーティクルがいないエピソードができないように注意  #
########################################################
def motion_update(particle):
    if T != 1: #重みに基づいてリサンプリング
        likelihood = [0.0 for i in range(len(episode_set))]
        for i in range(len(likelihood)):#パーティクルの尤度からエピソードの尤度(likelihood)を求める
            for ii in range (p):
                if particle[ii][0] == i:
                    likelihood[i] = particle[ii][1]
        l_sum = sum(likelihood)
        for i in range(len(likelihood)):
            likelihood[i] = likelihood[i]/l_sum
        print "likelihood(合計は1):",likelihood
        #likelihoodの分布に基づき8割のパーティクルを配置する
        for i in range(int(p * 0.9)):
            seed = random.randint(1,100)
            for ii in range(len(likelihood)):
                seed -= likelihood[ii] * 100
                if seed <= 0:
                    particle[i][0] = ii
                    break
        #likelihoodとは無関係に残りのパーティクルを配置する
        for i in range(int(p * 0.9),p):            
            seed = random.randint(0,len(episode_set)-1)
            particle[i][0] = seed

        #パーティクルがどこにいくつあるか表示する
        particle_numbers = [0 for i in range(len(episode_set))]
        for i in range(p):
            particle_numbers[particle[i][0]] += 1
        print "===パーティクルの分布==="
        cnt = 0
        for i in range(len(particle_numbers)):
            print particle_numbers[i],
            cnt += 1
            if cnt % 4 == 0:
                print " "
        print "T"

    elif T == 0: 
        for i in range(p):
            particle[i][0] = 0

    return particle

######################################
#   投票によって行動を決定する関数   #
#!!終了時の行動stayを追加した。この行動が評価されることが無いようにチェックする必要がある
#追記:stay行動を取ったエピソードにも票が入る。これは仕方がないので、入った上でうまく処理するように変更する必要あり
######################################
def decision_making(particle):
    global latest_sen
    if T == 1:#まだどんなエピソードも経験していない 前進させる
        return "f"
        
    else:#各パーティクルが投票で決める
        vote = range(p)#各パーティクルが自分の所属しているエピソードに対して持つ評価
        for i in range(p):
            vote[i] = 0.0
        for i in range (p):
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

        #print "###_decision_making_###:パーティクル = ",particle
    #    print "###_decision_making_###:各パーティクルが持つ票 = ",vote

    #voteに基づく行動決定。voteの合計がゼロやマイナスになる可能性がある点に注意
    got = [0.0 ,0.0 ,0.0 ,-10000.0] #得票数が入るリスト f,r,l,sの順番
    for i in range(p):
        if episode_set[particle[i][0]][5] == "f":
            got[0] += vote[i]
        elif episode_set[particle[i][0]][5] == "r":
            got[1] += vote[i]
        elif episode_set[particle[i][0]][5] == "l":
            got[2] += vote[i]
    print "###_decision_making_###:得票数 =",got

        #gotの中で最大値を持つ行動に対応した値をランダムに返す
        #行動がセンサ地の合計に対して適切なものになるように調節する
    if (random.randint(1,100) > epsiron):
        while(True):
            seed = random.randint(0,3)
            if got[seed] == max(got):
                if seed == 0:
                    if sum(latest_sen) >= fw_threshold:
                        if got[1] == got[2]:
                            print"r or l で適当に"
                            return random.choice("rl")
                        elif got[1] > got[2]:
                            print "旋回でどちらかといえばr"
                            return "r"
                        elif got[1] < got[2]:
                            print "旋回でどちらかといえばl"
                            return "l"
                    else:
                        print"###最大値的にf###"
                        return "f"
                elif seed == 1:
#                   print "### decision_making ###:sum(latest_sen)=",sum(latest_sen)
                    if sum(latest_sen) < fw_threshold:
                        print"### 前に壁があるのでf ###"
                        return "f"
                    else:
                        print"### 最大値的にr ###"
                        return "r"
                elif seed == 2:
#                   print "### decision_making ###:sum(latest_sen)=",sum(latest_sen)
                    if sum(latest_sen) < fw_threshold:
                        print"### 前に壁があるのでf ###"
                        return "f"
                    else:
                        print"### 最大値的にl ###"
                        return "l"
                elif seed == 3:
                        print"### おかしい ###"
                        return random.choice("frl")
                break
    else:
        print "###_decision_making_###:ランダムな行動決定"
        return random.choice("frl")

######################################################
#  センサ値が閾値を超えたらmoving_flagをFalseにする  #
######################################################
def stop(action):
    global moving_flag
    if action == "f":
        if sum(sensors_val) >= fw_threshold:
#           print "### _stop_:前に壁があるので前進は終了する"
            moving_flag = False
        else:
            moving_flag = True
    else:
        if sum(sensors_val) < fw_threshold:
#           print "### _stop_:前に壁がなくなったので旋回は終了する"
            moving_flag = False
        else:
            moving_flag = True

#########################################
#   パーティクルをスライドさせる関数    #
#########################################
def slide():
    print "すべてのパーティクルをひとつずらした"
    global particle
    for i in range(p):
        particle[i][0] += 1
    # 最新の行動と違う行動を取ったエピソードにいるパーティクルの重みはゼロにされる
    print "最新の行動=",action

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
    global moving_flag
    global T
    global action
    global latest_episode
    global episode_set
    global latest_sen

    counter += 1

    # センサデータを読み込む
    rf = message.right_forward
    rs = message.right_side
    ls = message.left_side
    lf = message.left_forward
    sensors_ave() # N回分のセンサ値の平均を取る
    if got_average_flag == True and moving_flag == False and end_flag == False:
        print "=========================###_sensors_callback_###============================"
        for i in range(4):
            latest_episode[i+1] = sensors_val[i]
            latest_sen[i] = sensors_val[i]
            sensors_val[i] = 0

        reward_check(x,y)

        if end_flag == True:
            #センサ値、行動(stay)を書き込んで色々保存して終了
            print "###_sensors_callback_###:トライアルを終了します"
            print "T= ",T
            print "T0 = ",T0
            if (T - T0) != 3:
                print "### エピソードの時間ステップが範囲外だったので取り消します ###"
                sys.exit()
            sensor_update()
            retrospective_resetting(alpha)
            motion_update(particle)
            action = "s"
            latest_episode[5] = "s"
            print "###_sensors_callback_###:latest_episode=",latest_episode
            episode_set.append(list(latest_episode))
            slide()
            #episode_set,particle をファイルに書き込んで終了
            f = open("episode_set.txt","w")
            f.write(str(episode_set))
            f.close()
            f = open("particle.txt","w")
            f.write(str(particle))
            f.close()
            sys.exit()

        sensor_update() #パーティクル集合の尤度を求める
        retrospective_resetting(alpha)
        motion_update(particle) #尤度に基づきパーティクルの分布を更新する
        action = decision_making(particle) #パーティクルの投票に基づき行動を決定する
        latest_episode[5] = action #最新のepisode_setにactionを追加
        print "###_sensors_callback_###:latest_episode=",latest_episode
        episode_set.append(list(latest_episode))#一連のエピソードをエピソード集合に追加
        if T != 1:
            slide()
        T += 1
        moving_flag = True
    elif got_average_flag == True and moving_flag == True:
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
