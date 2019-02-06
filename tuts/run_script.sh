#!/bin/bash
#$cmd1="python ~/Desktop/SITLLeapMotion/cam_server.py ; sleep 1"
#cmd2 = "roscore; sleep 1;"
#cmd3 = "roslaunch hls_lfcd_lds_driver hlds_laser.launch ; sleep 1;"
#cmd4 = "roslaunch hector_slam_launch tutorial.launch"
#full = $cmd1$cmd2$cmd3$cmd4
#echo $cmd1

cmd1="python ~/Desktop/SITLLeapMotion/cam_server.py ; sleep 1;"
cmd2="roscore; sleep 1;"
cmd3="roslaunch hls_lfcd_lds_driver hlds_laser.launch ; sleep 1;"
cmd4="roslaunch hector_slam_launch tutorial.launch"
#full=$cmd1$cmd2$cmd3$cmd4
#xterm -title "App 1" -e $cmd1
sleep 5
#gnome-terminal -x python ~/Desktop/SITLLeapMotion/cam_server.py
source /opt/ros/kinetic/setup.bash
sleep 2
gnome-terminal -x roscore
sleep 10
source /home/odroid/catkin_ws/devel/setup.bash
gnome-terminal -x roslaunch hls_lfcd_lds_driver hlds_laser.launch
sleep 3
gnome-terminal -x roslaunch hector_slam_launch tutorial.launch
#xterm -e python ~/Desktop/SITLLeapMotion/cam_server.py
#sleep 1 # 1 second sleep
# Lidar
#xterm -e roscore
#sleep 1 
#xterm -e roslaunch hls_lfcd_lds_driver hlds_laser.launch
#sleep 1
#xterm -e roslaunch hector_slam_launch tutorial.launch
