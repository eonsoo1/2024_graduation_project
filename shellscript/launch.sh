#! /bin/bash

gnome-terminal -e "roslaunch localization IMU.launch" &
sleep 2
gnome-terminal -e "rosrun localization GPS_node"&
sleep 2
gnome-terminal -e "rosrun localization EKF_node"
# sleep 2
# gnome-terminal -e "plotjuggler-ros"

# gnome-terminal -e "roslaunch localization EKF.launch m_init_yaw:=81.3" &
# sleep 3
# gnome-terminal -e "rqt_graph"
~
