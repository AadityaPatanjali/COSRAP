#!/usr/env bash
gnome-terminal -x sh -c "roscore; bash"
sleep 2
gnome-terminal -x sh -c "rosrun turtlesim turtlesim_node; bash"
sleep 1
gnome-terminal -x sh -c "python ~/catkin_ws/src/cosrap/src/scripts/trace.py; bash"
gnome-terminal -x sh -c "python ~/catkin_ws/src/cosrap/src/scripts/trace2.py; bash"
gnome-terminal -x sh -c "python ~/catkin_ws/src/cosrap/src/scripts/camera_capture.py; bash"
sleep 33

