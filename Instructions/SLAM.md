## Prerequises We require:
- Ubuntu 16.04 LTS
- ROS Kinetic
- Bash shell ROS Kinetic installation link:
http://wiki.ros.org/kinetic/Installation/Ubuntu

We strongly reccommend install the full version, which is ros-kinetic-desktop-full (please see the link above for more details) 

## Compile 
Make sure you are standing at top level directory of this project

catkin_make

This command will generate build and devel at the top level directory 

## Setup environment 
At these two lines into your ~/.bashrc

source /opt/ros/kinetic/setup.bash
source <absolute path to the top level directory of this project>/ros_ws/devel/setup.bash

Execute source ~/.bashrc for the current terminal. After later newly opened terminal, you don't have to do this.

## How to Run SLAM

Running SLAM:
-Run “roscore” in a terminal. This will be running the entire time.
-In new terminal in base workspace: catkin_make
-To start the SLAM node: rosrun see_localization see_localization

This is how it will be running on the actual car. We will use sample data and a visualizer to show that it is working right now.

Running visualiser:
-In new terminal in base workspace, run: rosmake rviz
-to start rviz node: rosrun rviz rviz
-In the rviz program, in the menu on the left, change the value of fixed frame to “slam”
-Also on the left menu, at the bottom click “add”. Go to the “by topic” tab. These are all messages that can be visualized. I recommend adding all the topics under see_localization. Saving these changes in rviz will save you the time of adding them again.

Running sample data node:
-To start sample data node in new terminal: rosrun see_localization sample_data
-If all nodes are running, you should be able to see markers being added to rviz.
-To kill nodes it may be necessary to use: killall see_localization sample_data

At this point a track and car location should appear in rviz. This is the information that will be sent to trajectory planning.

## How to Run EKF Localization With Visualiser

Step 1: Open a terminal and run the command "roscore" to start ros core

Step 2: Open another terminal and run the command "rviz" to start rviz, in the menu on the left, change the value of fixed frame to “slam” -Also on the left menu, at the bottom click “add”. Go to the “by topic” tab. These are all messages that can be visualized. I recommend adding all the topics under see_localization. Saving these changes in rviz will save you the time of adding them again.

Step 3: Open another terminal and run the command "rosrun see_localization EKF_Localization.py" to run the main EKF localization program. if it shows that's unexecutable, you might go to the /see_localization/scripts to run chmod -x EKF_Localization.py to make it executable, same method applied to all following python program.

Step 4: Open another terminal and run the command "rosrun see_localization publish_to_rviz.py" to run the node that publish data to the visualiser

Step 5: Open yet another terminal and run the command "rosrun see_localization sample_data.py" to feed the data to EKF main program.

Note that the order of running those program matters

After you start feeding data to EKF main program, there should be something shown in the rviz you open and config previously, chack that out.
