#!/bin/bash

terminator
xdotool key Super+Up
sleep 1

xdotool type  --delay 1 --clearmodifiers "cd .."
xdotool key --delay 100 Return

# 터미널 8개 창으로 분할
xdotool key ctrl+shift+o
xdotool key alt+Up
xdotool key ctrl+shift+e
xdotool key alt+Left
xdotool key ctrl+shift+e
xdotool key alt+Right
xdotool key ctrl+shift+e
xdotool key alt+Down
xdotool key ctrl+shift+e
xdotool key alt+Left
xdotool key ctrl+shift+e
xdotool key alt+Right
xdotool key ctrl+shift+e

# 첫 번째 터미널로 이동
xdotool key alt+Up
xdotool key alt+Left
xdotool key alt+Left
xdotool key alt+Left

xdotool type  --delay 1 --clearmodifiers "sudo chmod 777 /dev/tty*"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "1"
xdotool key --delay 100 Return
sleep 1


# 첫 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool type  --delay 1 --clearmodifiers "sd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "roslaunch myahrs_driver myahrs_driver.launch"
xdotool key --delay 100 Return
sleep 1

# 두 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool key --delay 100 alt+Right
xdotool type  --delay 1 --clearmodifiers "sd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "roslaunch ntrip_ros ntrip_ros.launch"
xdotool key --delay 100 Return
sleep 1

# 세 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool key --delay 100 alt+Right
xdotool type  --delay 1 --clearmodifiers "cd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM2 _baud:=9600"
sleep 1


# 세 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool key --delay 100 alt+Right
xdotool type  --delay 1 --clearmodifiers "cd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "arduino"
sleep 1


# 세 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool key --delay 100 alt+Down
xdotool type  --delay 1 --clearmodifiers "sd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "rosrun planning lattice_planner"
xdotool key --delay 100 Return
sleep 1

# 세 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool key --delay 100 alt+Left
xdotool type  --delay 1 --clearmodifiers "sd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "rosrun planning auto_drive"
xdotool key --delay 100 Return
sleep 1

# 세 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
xdotool key --delay 100 alt+Left
xdotool type  --delay 1 --clearmodifiers "sd"
xdotool key --delay 100 Return
xdotool type  --delay 1 --clearmodifiers "roslaunch localization Local.launch"
xdotool key --delay 100 Return
sleep 1




# 네 번째 창을 수직 분할하여 새로운 터미널을 엽니다.
# xdotool key --delay 100 alt+Right
# xdotool type  --delay 1 --clearmodifiers "cd"
# xdotool key --delay 100 Return
# xdotool type  --delay 1 --clearmodifiers "cd rosbag"
# xdotool key --delay 100 Return
# xdotool type  --delay 1 --clearmodifiers "rosbag play Monday3.bag"
# xdotool key --delay 100 Return
# sleep 1

