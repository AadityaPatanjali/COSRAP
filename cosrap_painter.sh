#!/usr/env bash
gnome-terminal -x sh -c "roscore; bash"
sleep 2
gnome-terminal -x sh -c "rosrun turtlesim turtlesim_node; bash"
sleep 5  
gnome-terminal -x sh -c "rosservice call turtle1/set_pen 69 86 255 3.14 off; bash"
sleep 1
rosservice call turtle1/teleport_absolute 3 1 1.57079632679
sleep 1
rosservice call turtle1/set_pen 255 255 255 3.14 off
rosservice call /spawn 8 8.5 4.712388980384 ""
rosservice call turtle2/set_pen 255 255 255 3.14 off
gnome-terminal -x sh -c "python ~/catkin_ws/src/cosrap/trace.py; bash"
gnome-terminal -x sh -c "python ~/catkin_ws/src/cosrap/trace2.py; bash"
sleep 33
poof
