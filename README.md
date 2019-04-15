# OSU-Driverless-Formula-Student
Repository of code used by senior design team for driverless formula student race car

## Prerequises

We require:
- Ubuntu 16.04 LTS
- ROS Kinetic
- Bash shell

ROS Kinetic installation link:
```
http://wiki.ros.org/kinetic/Installation/Ubuntu
```
We strongly reccommend install the full version, which is `ros-kinetic-desktop-full` 
(please see the link above for more details)

## Compile

Make sure you are standing at top level directory of this project

```
catkin_make
```
This command will generate `build` and `devel` at the top level directory

## Setup environment

At these two lines into your `~/.bashrc` 
```
source /opt/ros/kinetic/setup.bash
source <absolute path to the top level directory of this project>/ros_ws/devel/setup.bash
```
Execute `source ~/.bashrc` for the current terminal. After later newly opened terminal, you don't have to do this.

## How to run lidar component

Note for **each** of these following steps below:
- **We assume you are standing at top level directory of this project. (the directory you cd into after you clone this repository)**
- **Open a new terminal before following instructions in each step**

1. Run ouster ROS node:
**Open a new terminal**
```
cd ros_ws/src/SEE/see_lidar_ouster_driver/ouster_ros
roslaunch ouster_ros os1.launch replay:=true os1_hostname:=localhost
```

2. Replay recorded lidar data:
**Open a new terminal**
```
cd data/lidar/ros_bags
./playSimuBag.sh 1 ./<name of the .bag file you want to test>
```
For a good `.bag` file, we suggest `2019-01-26-16-19-02.bag`

3. Run our ROS node
**Open a new terminal**
```
cd ros_ws/src/SEE/see_lidar_processing
roslaunch see_lidar_processing see_lidar_processing.launch | tee log.txt
```
Output on your screen is also written in `log.txt` at the `ros_ws/src/SEE/see_lidar_processing` directory

4. Visualization
**Open a new terminal**
```
cd ros_ws/src/SEE/see_lidar_ouster_driver/ouster_ros
rviz -d ./viz.rviz
```

If you installed the full version of ROS kinetic, `rviz` should already been installed. 
If otherwise or somehow `rviz` was not installed, you can install `rviz` with
```
sudo apt-get install ros-kinetic-rviz
```

If nothing usual happens, Rviz application will open. On the left side pane, select the dropdown menu on the right of the `Topic` option, 
select the `/see_lidar/see_lidar_cones_center` topic to see the cone detection result. Other topics include: `/os1_node/points` 
for the original raw data from the lidar, `/see_lidar/see_lidar_points_of_interest` for the prefiltered data, 
and `/see_lidar/see_lidar_clustered_point_clouds` for the data clustered into groups of potential cones. 

You can see around by using your mouse: Drag mouse to move your camera, scroll mouse to zoom in and out. Here is a sample screenshot:
![RViz UI](https://raw.githubusercontent.com/lcsfrey/OSU-Driverless-Formula-Student/master/images/lidar/lidar_rviz_ui.png)

What Rviz does is listening to the topic that our lidar processing ROS node publish to, and then use that data to visualize our output.
In the driverless system as a whole, the SLAM component subscribes (listens) to this ROS topic and use our output as their input. 
Through the visualization of the centers of the detected cone as you can see in Rviz, 
we demonstrate that our lidar processing component works as expected in the requirements.
