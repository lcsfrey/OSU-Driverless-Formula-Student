#!/bin/bash

#roslaunch ouster_ros os1.launch replay:=true
#rviz -d ~/ouster/ouster_example/ouster_ros/viz.rviz
#

speed=$1
bag_file=$2

while [ 1 ]; do
    rosbag play -r $speed --clock $bag_file
    sleep 3
done
		
