# OSU-Driverless-Formula-Student
Repository of code used by senior design team for driverless formula student race car

## Prerequises

We require:
- Ubuntu 16.04 LTS
- ROS Kinetic
- Bash shell
- Python 2.7.15
- CUDA 9.0 compatible Nvidia GPU with at least 4GB of VRAM (tested on GTX 1080) (Note: You do not need to install cuda yourself. This will be handled by package that needs it.)
- Anaconda (conda 4.6.12)
- pip (18.1)

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
This command will generate `build` and `devel` directories under the `ros_ws` directory

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

# Running & Grading Instruction

We have seperated running & grading instruction of each component in each markdown file under the [Instructions](Instructions) directory
Please see further instructions under the [Instructions](Instructions) directory
