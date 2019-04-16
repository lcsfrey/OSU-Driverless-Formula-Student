# OSU-Driverless-Formula-Student
Repository of code used by senior design team for driverless formula student race car

## Prerequises

We require:
- Ubuntu 16.04 LTS
- ROS Kinetic
- Bash shell
- Python 2.7.15
- CUDA 9.0 compatible Nvidia GPU with at least 4GB of VRAM (tested on GTX 1080) (Note: You do not need to install cuda yourself. This will be handled by package that needs it.)
- Anaconda

ROS Kinetic installation link:
```
http://wiki.ros.org/kinetic/Installation/Ubuntu
```
We strongly reccommend install the full version, which is `ros-kinetic-desktop-full` 
(please see the link above for more details)


## Compile

Make sure you are standing at top level directory of this project

```
cd ros_ws
catkin_make
```
This command will generate `build` and `devel` at the top level directory

## Setup environment

1. Install an anaconda environement and activate it

```
conda create -n DV python=2.7.15
conda activate DV
```

2. Install shared python dependences. 
At the top level directory, run:

```
pip install -r requirements.txt
```

3. At these two lines into your `~/.bashrc` 
```
source /opt/ros/kinetic/setup.bash
source <absolute path to the top level directory of this project>/ros_ws/devel/setup.bash
```
Execute `source ~/.bashrc` for the current terminal. After later newly opened terminal, you don't have to do this.

## How to run the lidar component

Note for **each** of these following steps below:
- **We assume you are standing at top level directory of this project. (the directory you cd into after you clone this repository)**
- **Open a new terminal before following instructions in each step**

1. Run ouster ROS node:
**Open a new terminal**
```
cd ros_ws/src/see_lidar_ouster_driver/ouster_ros
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
cd ros_ws/src/see_lidar_processing
roslaunch see_lidar_processing see_lidar_processing.launch | tee log.txt
```
Output on your screen is also written in `log.txt` at the `ros_ws/src/see_lidar_processing` directory

4. Visualization
**Open a new terminal**
```
cd ros_ws/src/see_lidar_ouster_driver/ouster_ros
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
![RViz UI](images/lidar/lidar_rviz_ui.png?raw=true "RViz UI")

What Rviz does is listening to the topic that our lidar processing ROS node publish to, and then use that data to visualize our output.
In the driverless system as a whole, the SLAM component subscribes (listens) to this ROS topic and use our output as their input. 
Through the visualization of the centers of the detected cone as you can see in Rviz, 
we demonstrate that our lidar processing component works as expected in the requirements.

## How to run the camera component

1. Install prerequisites
In the top level directory run
```
cd ros_ws/src/see_camera_processing
conda install pytorch torchvision cudatoolkit=9.0 -c pytorch
pip install -r requirements.txt
```

2. Run the component
```
roslaunch launch/see_camera_processing_debug_test.launch
```
This will open four ROS nodes:
- see_camera_processing_image_generator
- see_camera_processing_image_correction
- see_camera_processing_cone_detection (the work done by our team)
- see_camera_processing_coordinate_extraction

A debugging window will appear. After a few seconds, test images will start to appear
which were read in from see_camera_processing/testdata. (During the competition,
this node will be replaced by a node which reads in images from the camera. For now, the see_camera_processing_image_generator node is just to demonstrate the capability of the cone detection node.)

