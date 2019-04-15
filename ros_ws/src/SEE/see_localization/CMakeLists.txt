cmake_minimum_required(VERSION 2.8.3)
project(see_localization)

## Specify libraries for shared use with other packages
set(LIBRARY "src/Algorithm.cpp")

## Specify source files for generation of executables only available for this package
set(EXECUTABLES "src/${PROJECT_NAME}_node.cpp" "src/SeeLocalization.cpp" )

## Specify catkin packages your project and dependent projects need
set(CATKIN_DEPENDENCIES "roscpp" "message_runtime")

## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
## Only customize below this line if you really know what you're doing
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

## Use C++11
add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    see_lidar_processing
    see_ego_motion
    see_camera_processing
)

find_package(Eigen3 REQUIRED)

##Add messages
add_message_files(
  FILES
  world_map_msg.msg
  SLAM_cone_msg.msg
  vehicle_pose_msg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: this is necessary if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    ${CATKIN_DEPENDENCIES}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)



## Declare a cpp library
add_library(${PROJECT_NAME}_core
  ${LIBRARY}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  ${EXECUTABLES} src/SLAM.cpp src/Cone.cpp src/Particle.cpp src/LandmarkEstimate.cpp src/SLAMmath.cpp src/Sensor.cpp
)

add_executable(sample_data src/sample_data_node.cpp src/Cone.cpp src/SLAM.cpp src/Particle.cpp src/Sensor.cpp src/LandmarkEstimate.cpp src/SLAMmath.cpp)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(sample_data ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(sample_data ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
