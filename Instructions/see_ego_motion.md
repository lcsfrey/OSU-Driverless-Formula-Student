# Egomotion / Odometry

This node keeps track of the location and velocity of the car using non-vision sensor data. 

# Install instructions
Run `bash ros_ws/src/see_ego_motion/src/install.bash`

# Use

To run the odometry system with existing data and visualization, use `roslaunch see_ego_motion/see_ego_motion_with_test.launch`
Change the Fixed Frame setting to "map"
Below the panel on the left side, click "Add"
Change to the "By Topic" panel
Add the "see_ego_motion_test/odometry_filtered" marker
Add the "see_ego_motion_test/odometry_visualization/particles" marker

To run the core of the odometry system for use as a part of the larger perception system, use `roslaunch see_ego_motion/see_ego_motion.launch`

# Prerequisite Packages

message_generation
message_runtime
visualization_messages
utm

# Other prerequisites

AMZ example bag data
