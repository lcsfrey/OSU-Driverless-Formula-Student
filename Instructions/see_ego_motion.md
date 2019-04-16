# Egomotion / Odometry

This node keeps track of the location and velocity of the car using non-vision sensor data. 

# Prerequisite Packages

message_generation
message_runtime
visualization_messages

# Other prerequisites

AMZ example bag data

# Installation Instructions

Run the main installation script (INSTALL.bash)

# Use

To run the odometry system with existing data and visualization, use `roslaunch see_ego_motion/see_ego_motion_with_test.launch`

To run the core of the odometry system for use as a part of the larger perception system, use `roslaunch see_ego_motion/see_ego_motion.launch`