# see_lidar_processing

This node implements basic ROS communication interfaces.

**Important: This file is not up to date**\
**use `roslaunch see_lidar_processing see_lidar_processing.launch` to start.**\
**use the magic script to get the workspace (gfr19d_dev_ws)**\
**the published messages are different than described below**

## Usage

After starting `roscore` in another terminal, execute the following commands.

  `cd gfr19d_ws`\
  `catkin_make`\
  `rosrun see_lidar_processing see_lidar_processing`

## Nodes

### see_lidar_processing

Offers publication, subscription and service interfaces.

#### Subscribed Topics

* **`see_lidar_processing_subscription`** [std_msgs/String]

  Receives a string and does nothing.

#### Published Topics

* **`see_lidar_processing_publication`** [std_msgs/String]

  Publishes a string with a frequency of 1 Hz.

#### Services

* **`see_lidar_processing_service`** [std_srvs/Trigger]

  Can be polled for string retrieval.
