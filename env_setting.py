#coding:utf-8
import subprocess
import sys

args = sys.argv

"""
========================== * 目的 * =================================

 このプログラムはラズパイマウスシミュレータの環境を設定します
 4×4マスの迷路を生成し、ロボットを任意の初期位置/姿勢で配置します


======================== * 実行すると * ==============================

 このプログラムを実行すると、
 ~/catkin_ws/src/raspimouse_sim/raspimouse_gazebo/launch下の
 raspimouse_with_samplemaze.launch
 および、
 ~/catkin_ws/src/raspimouse_sim/raspimouse_gazebo/materials/下の
 sample_maze.world.xacroを書き換えます


=========================== * 注意 * ================================

このプログラムを実行するディレクトリに
"xacro"という名前のファイルが存在すると、そのファイルは消えてしまいます


=========================== * 使い方 * ================================ 

 以下の例のように、半角スペース、"|","_"のみを使って、
 作成したい4×4マスの迷路の形状を左詰めで表示してください。
 ただし、迷路の外周はすべて存在していなくてはなりません

例: 
maze = ”””
 _ _ _ _
| |   | |
| |_ _| |
|  _ _ _|
|_|_ _ _|

”””

 また以下のようにロボットの任意の初期配置を示してください。
 ただし、一番左下のマス目の中央が(0.0,0.0)で、1マスぶんの長さが1.0です。

例:
x = 0.0
y = 0.0

"""

# ======================= 書き換えるところ ============================ #

maze = """
 _ _ _ _  
|_ _ _  |
|_   _| |
| | |   |
|_|_|_ _|

"""

x = 1.0
y = 0.0

try:
    if args[1] == "right":
        x = 1.1
    elif args[1] == "left":
        x = 0.9
except IndexError:
    x = 1.0



# ====================================================================== #
lines = ["","","","","","","","",""]

# mazeを行ごとに分割してlinesに入れる
i = 0
for cnt in range (len(maze)):
   if maze[cnt] != "\n":
       lines[i] += maze[cnt]
   else:
       i += 1

# code:壁の配置のコード
code = ["","","","",""]
code[0] = [0,0,0,0,0,0,0,0,0]
code[1] = [0,0,0,0,0,0,0,0,0]
code[2] = [0,0,0,0,0,0,0,0,0]
code[3] = [0,0,0,0,0,0,0,0,0]
code[4] = [0,0,0,0,0,0,0,0,0]

# linesの何行目から迷路が配置されているか調べる
start = 0
for cnt in range(len(lines)):
    if len(lines[cnt]) != 0:
        start = cnt
        break

print lines[start]
print lines[start + 1]
print lines[start + 2]
print lines[start + 3]
print lines[start + 4]

# linesからcodeを設定する
for i in range (start,start+5):
    for ii in range (len(lines[i])):
        if lines[i][ii] != " ":
            code[i-start][ii] = 1

print code[0]
print code[1]
print code[2]
print code[3]
print code[4]

# code[]を用いて設定ファイルを作成する

L = 0.181 #1マスぶんの長さ

string = """<?xml version="1.0" ?>
<sdf version="1.4" xmlns:xacro="http://ros.org/wiki/xacro">
<!-- <sdf version="1.4"> -->
<xacro:include filename="$(find raspimouse_gazebo)/materials/sample_maze.wall.xacro"/>
<world name="default">
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <light name='sun' type='directional'>
    <cast_shadows>1</cast_shadows>
    <pose>0 0 5 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.5 0.1 -0.9</direction>
  </light>

  <physics type='ode'>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.8</gravity>
  </physics>


  <gui fullscreen='0'>
    <camera name='user_camera'>
      <pose>-0.556842 -0.914575 1.68773 0 1.05964 0.520195</pose>
      <view_controller>orbit</view_controller>
    </camera>
  </gui>"""


xacro_f = """
  <xacro:wall>
    <pose>"""

xacro_b = """</pose>
  </xacro:wall>"""

# 横線のファイルを作成
for i in range(5):
    for ii in range(1,8,2):
        if code[i][ii] == 1:
            string += xacro_f
            string += str((3.5 - i) * L)
            string += " "
            string += str((1.0 - ii/2) * L)
            string += " 0.025 0.0 0.0 1.57"
            string += xacro_b
            
# 縦線のファイルを作成
for i in range(5):
    for ii in range(0,9,2):
        if code[i][ii] == 1:
            string += xacro_f
            string += str((4.0 - i) * L)
            string += " "
            string += str((1.5 - ii/2) * L)
            string += " 0.025 0.0 0.0 0.0"
            string += xacro_b

string += """
</world>
</sdf>"""

#作成したファイルで上書きする
f = open("xacro","w")
f.write(string)
f.close()

proc = subprocess.call("cp xacro ~/catkin_ws/src/raspimouse_sim/raspimouse_gazebo/materials/sample_maze.world.xacro",shell = True)
proc = subprocess.call("rosrun xacro xacro.py ~/catkin_ws/src/raspimouse_sim/raspimouse_gazebo/materials/sample_maze.world.xacro > ~/catkin_ws/src/raspimouse_sim/raspimouse_gazebo/worlds/sample_maze.world",shell = True)

# x,yの処理
x /= 5.525
y /= 5.525

x,y = y,-x
y += 0.181

string = """<?xml version="1.0"?>
<launch>
  <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="model" default="$(find raspimouse_description)/urdf/raspimouse.urdf.xacro"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find raspimouse_gazebo)/worlds/sample_maze.world"/>
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>

  <!-- Load the URDF into the ROS Parameter Server -->
  <param name="raspimouse/robot_description" command="$(find xacro)/xacro.py '$(arg model)'" />
  
  <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -model RasPiMouseV2 """
string += "-x "
string += str(x)
string += " "
string += "-y "
string += str(y)
string += " "
string += """ -param raspimouse/robot_description"/>
  <!-- ros_control motoman launch file -->
  <include file="$(find raspimouse_control)/launch/raspimouse_control.launch"/>
</launch>"""

f2 = open("xacro","w")
f2.write(string)
f2.close()

proc = subprocess.call("cp xacro ~/catkin_ws/src/raspimouse_sim/raspimouse_gazebo/launch/raspimouse_with_samplemaze.launch",shell = True)
