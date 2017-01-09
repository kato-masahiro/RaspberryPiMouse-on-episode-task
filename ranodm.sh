#!/bin/bash
if [ $(($RANDOM%2)) -eq 1 ]; then
    python env_setting.py right
    roslaunch raspimouse_gazebo raspimouse_with_samplemaze.launch
else
    python env_setting.py left
    roslaunch raspimouse_gazebo raspimouse_with_samplemaze.launch
fi
