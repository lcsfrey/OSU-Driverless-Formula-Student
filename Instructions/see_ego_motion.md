# Egomotion / Odometry

This node keeps track of the location and velocity of the car using non-vision sensor data. 

# Install instructions
Run ros_ws/src/see_ego_motion/src/install.bash

# Prerequisite Packages

message_generation
message_runtime
visualization_messages
utm

# Other prerequisites

AMZ example bag data

# Use

To run the odometry system with existing data and visualization, use `roslaunch see_ego_motion/see_ego_motion_with_test.launch`

To run the core of the odometry system for use as a part of the larger perception system, use `roslaunch see_ego_motion/see_ego_motion.launch`