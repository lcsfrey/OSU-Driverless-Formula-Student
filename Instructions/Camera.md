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

