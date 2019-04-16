# OSU-Driverless-Formula-Student
Repository of code used by senior design team for driverless formula student race car

## Prerequises

We require:
- Ubuntu 16.04 LTS
- ROS Kinetic
- Bash shell
- Python 2.7
- Git (at least version 2.7)
- Git LFS (latest version). Download it [here](https://git-lfs.github.com/) and follow [this instruction](https://help.github.com/en/articles/installing-git-large-file-storage) to install it **before cloning this repository**

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

At these two lines into your `~/.bashrc` 
```
source /opt/ros/kinetic/setup.bash
source <absolute path to the top level directory of this project>/ros_ws/devel/setup.bash
```
Execute `source ~/.bashrc` for the current terminal. After later newly opened terminal, you don't have to do this.

# Running & Grading Instruction

We have seperated running & grading instruction of each component in each markdown file under the [Instructions](Instructions) directory
Please see further instructions under the [Instructions](Instructions) directory
