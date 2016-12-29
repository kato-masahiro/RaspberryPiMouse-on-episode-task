#!/usr/bin/env python
#coding:utf-8
"""
パーティクルフィルタでエピソード的タスクを学習させる

全体的な流れ:

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

# 変更可能なパラメータ
p = 1000                              # パーティクルの数
lmd = 16                              #retrospective_resettingの時、いくつのイベントを残すか
N = 5                                # 何回分のセンサ値の平均を取って利用するか
fw_threshold = 5000                   # 前進をやめるかどうかの判定に使われる閾値(rf+rs+ls+lf)
alpha_threshold = 0.0                 # retrospective_resettingを行うかどうかの閾値。0.0だと行わない。1.0だと常に行う。
greedy_particles = 0.9                # パーティクルが尤度関数に基づいてリサンプリングされる確率

# その他のグローバル変数
x = 0.0; y = 0.0                      # ロボットの座標
rf = 0; rs = 0; ls = 0; lf = 0        # センサ値
sensors_val = [0,0,0,0]               # 平均を取るためにrf,rs,ls,lfの和を入れるための変数
counter = 0                           # sensors_callbackを何回実行したか
T = 1                                 # 最新の時間ステップ(いままで経験したエピソードの数+1)
T0 = 1
action = ""                           # 行動."f","r","l","s"の3種類(前進、右旋回、左旋回,待機)待機は実際には行われない
moving_flag = False                   # ロボットが行動中かどうかのフラグ
got_average_flag = False              # センサ値が平均値をとっているかどうかのフラグ
end_flag = False                      # 非ゼロ報酬を得たらこのフラグが立って、すべての処理を終わらせる。
particle = range(p)                # パーティクルの位置、重みが入るリスト。パーティクルの重みの合計は1
for i in particle:
    particle[i] = [0, 1.0/p]
latest_episode = [0.0 ,0, 0, 0, 0,""] # 最新のエピソード。報酬値、センサ値、行動。
episode_set = []                    # 過去のエピソードの集合。報酬値、センサ値、行動
alpha = 0.0

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

def sensors_ave():
    """センサの平均値を求める"""
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

def reward_check(x,y):
    """
    ロボットの位置に基づき、正解・不正解・行動の続行等を決定する
    """
    global end_flag
    global latest_episode
    if reward_arm == "right":
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
            latest_episode[0] = 0.0
            end_flag = False

def sensor_update(particle):
    """
    パーティクルの尤度を更新し、αも求める
    引数:particle
    戻り値:particle,alpha
    処理:
        すべてのパーティクルの位置を一つづつスライドさせる
        各パーティクルの重みを求める
        αを求める
        αを用いてパーティクルの重みを正規化する
    """
    alpha = 0.0
    if T != 1:
        for i in range(p):
            if episode_set[ particle[i][0] ][0] == latest_episode[0] and particle[i][1] != "X":
                l1 = math.fabs(latest_episode[1] - episode_set[ particle[i][0] ][1])
                l2 = math.fabs(latest_episode[2] - episode_set[ particle[i][0] ][2])
                l3 = math.fabs(latest_episode[3] - episode_set[ particle[i][0] ][3])
                l4 = math.fabs(latest_episode[4] - episode_set[ particle[i][0] ][4])
                particle[i][1] = 0.5 ** ((l1+l2+l3+l4) / 4000)
            else:
                particle[i][1] = 0.0
    elif T == 1:
        for i in range(p):
            particle[i][1] = 1.0/p

    #alphaも求める
    for i in range(p):
        alpha += particle[i][1]
    #alphaで正規化
    if math.fabs(alpha) >= 0.0001:
        for i in range(p):
            particle[i][1] /= alpha
    else:
        for i in range(p):
            particle[i][1] = 1.0/p
    alpha /= p
    print "alpha:",alpha
    return particle,alpha

def retrospective_resetting():
    """
    retrospective_resettingを行う
    処理:
        エピソードを直近のいくつかだけを残して削除する
        削除されたエピソードの中に、パーティクルを均等に配置する
    """
    global episode_set
    global particle
    episode_set = episode_set[-lmd::]
    for i in range(p):
        particle[i][0] = random.randint(0,lmd-1)
        particle[i][1] = 1.0/p

def motion_update(particle):
    """
    尤度に基づいてパーティクルをリサンプリングする関数
    リサンプリング後のパーティクルの分布も表示する
    引数:particle
    戻り値:particle
    """
    if T != 1: #重みに基づいてリサンプリング
        likelihood = [0.0 for i in range(len(episode_set))]
        for i in range(len(likelihood)):#パーティクルの尤度からエピソードの尤度(likelihood)を求める
            for ii in range (p):
                if particle[ii][0] == i:
                    likelihood[i]+= particle[ii][1]
        #likelihoodの分布に基づき8割のパーティクルを配置する
        for i in range(int(p * greedy_particles)):
            seed = random.randint(1,100)
            for ii in range(len(likelihood)):
                seed -= likelihood[ii] * 100
                if seed <= 0:
                    particle[i][0] = ii
                    break
        #likelihoodとは無関係に残りのパーティクルを配置する
        for i in range(int(p * greedy_particles),p):            
            seed = random.randint(0,len(episode_set)-1)
            particle[i][0] = seed

        #パーティクルがどこにいくつあるか表示する
        particle_numbers = [0 for i in range(len(episode_set))]
        for i in range(p):
            particle_numbers[particle[i][0]] += 1
        print "===パーティクルの分布==="
        cnt = 0
        for i in range(len(particle_numbers)):
            print particle_numbers[i],"\t",
            cnt += 1
            if cnt % 4 == 0:
                print " "
        print "T"

    elif T == 0: 
        for i in range(p):
            particle[i][0] = 0

    return particle

def decision_making(particle,latest_episode):
    """
    投票によって行動を決定する
    引数:particle,latest_episode
    戻り値:action
    """
    if T == 1:#まだどんなエピソードも経験していない 前進させる
        return "f"
        
    else: #各パーティクルが投票で決める
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

    print "センサ値:",latest_episode[1:5]
    #voteに基づく行動決定。voteの合計がゼロやマイナスになる可能性がある点に注意
    got = [0.0 ,0.0 ,0.0 ,0.0] #得票数が入るリスト f,r,l,sの順番
    for i in range(p):
        if episode_set[particle[i][0]][5] == "f":
            got[0] += vote[i]
        elif episode_set[particle[i][0]][5] == "r":
            got[1] += vote[i]
        elif episode_set[particle[i][0]][5] == "l":
            got[2] += vote[i]
    print "###_decision_making_###:得票数 =",got

    #前に壁がなければ投票にかかわらず前進させる
    if sum(latest_episode[1:5]) < fw_threshold:
        return "f"
    elif got[1] == got[2]:
        return random.choice("rl")
    elif got[1] > got[2]:
        return "r"
    elif got[1] < got[2]:
        return "l"
    else:
        print("###_decision_making_###:error")

def stop(action):
    """
    閾値によってmoving_flagをオンオフする
    """
    global moving_flag
    if action == "f":
        if sum(sensors_val) >= fw_threshold:
            moving_flag = False
        else:
            moving_flag = True
    else:
        if sum(sensors_val) < fw_threshold:
            moving_flag = False
        else:
            moving_flag = True

def slide(particle):
    """
    すべてのパーティクルの位置を一つ＋１する
    引数:particle
    戻り値:particle
    """
    for i in range(p):
        particle[i][0] += 1
        if episode_set[ particle[i][0] -1 ][5] != latest_episode[5]:
            particle[i][1] = "X" #あとで(sensor_updateのとき)ゼロになる
    return particle

def sensors_callback(message):
    """
    センサ値をsubscribeするコールバック関数
    main
    """
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
    global particle

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
            sensors_val[i] = 0

        reward_check(x,y)

        if end_flag == True:
            #センサ値、行動(stay)を書き込んで色々保存して終了
            print T/4," 回目のトライアルが終了しました"
            print "==================================="
            if (T - T0) != 3:
                print "### エピソードの時間ステップが範囲外だったので取り消します ###"
                sys.exit()

            particle,alpha = sensor_update(particle)
            if alpha < alpha_threshold and len(episode_set) >= lmd:
                print "リセッティングを行う"
                retrospective_resetting()
                particle,alpha = sensor_update(particle)
            motion_update(particle)
            action = "s"
            latest_episode[5] = "s"
            print "###_sensors_callback_###:latest_episode=",latest_episode
            episode_set.append(list(latest_episode))
            particle = slide(particle)
            #episode_set,particle をファイルに書き込んで終了
            f = open("episode_set.txt","w")
            f.write(str(episode_set))
            f.close()
            f = open("particle.txt","w")
            f.write(str(particle))
            f.close()
            sys.exit()

        particle,alpha = sensor_update(particle) 
        if alpha < alpha_threshold and len(episode_set) >= lmd:
            print "リセッティングを行う"
            retrospective_resetting()
            particle,alpha = sensor_update(particle)
        motion_update(particle) #尤度に基づきパーティクルの分布を更新する
        action = decision_making(particle,latest_episode) #パーティクルの投票に基づき行動を決定する
        latest_episode[5] = action #最新のepisode_setにactionを追加
        print "###_sensors_callback_###:latest_episode=",latest_episode
        episode_set.append(list(latest_episode))#一連のエピソードをエピソード集合に追加
        if T > 1:
            particle = slide(particle)
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

def position_callback(message):
    """ロボットの現在位置をsubscribeする関数"""
    global x;global y
    x = message.pose[-1].position.x
    y = message.pose[-1].position.y

rospy.init_node("particle_filter_on_episode")
pub = rospy.Publisher("/raspimouse/diff_drive_controller/cmd_vel",Twist,queue_size = 10)
sub1 = rospy.Subscriber("/raspimouse/lightsensors",LightSensorValues,sensors_callback)
sub2 = rospy.Subscriber("/gazebo/model_states",ModelStates,position_callback)
rospy.spin()
