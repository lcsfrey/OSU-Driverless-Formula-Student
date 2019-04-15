cmake_minimum_required(VERSION 2.8.3)
project(see_lidar_processing)

## Specify libraries for shared use with other packages
set(LIBRARY "src/Algorithm.cpp")

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
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    ouster_client
    ouster_ros
)

add_message_files(
  FILES
  ConeContainer.msg
  Cone.msg
  Coordinates.msg
  CovarianceMatrix.msg
  ConeColor.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: this is necessary if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    ${CATKIN_DEPENDENCIES}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  ${LIBRARY}
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## following nodes have an own main()-function, so an own executable is necessary
add_executable(see_lidar_pointcloud_prefilter
  src/SeeLidarPointcloudPrefilter.cpp
)

target_link_libraries(see_lidar_pointcloud_prefilter
#  see_lidar_pointcloud_prefilter_core
  ${catkin_LIBRARIES}
)

add_executable(see_lidar_pointcloud_clusterer
  src/SeeLidarPointcloudClusterer.cpp
)
target_link_libraries(see_lidar_pointcloud_clusterer ${catkin_LIBRARIES})
#add_dependencies(see_lidar_pointcloud_clusterer see_lidar_messages)

add_executable(see_lidar_cone_detector
  src/SeeLidarConeDetector.cpp
)
target_link_libraries(see_lidar_cone_detector ${catkin_LIBRARIES})
# The Cone Detector creates the cone-messages, therefore this node needs the generated message files
add_dependencies(see_lidar_cone_detector see_lidar_processing_generate_messages_cpp)

add_executable(see_lidar_cone_identificator
  src/SeeLidarConeIdentificator.cpp
)
target_link_libraries(see_lidar_cone_identificator ${catkin_LIBRARIES})
# The Cone Identificator creates the cone-messages, therefore this node needs the generated message files
add_dependencies(see_lidar_cone_identificator see_lidar_processing_generate_messages_cpp)

# Mark executables and/or libraries for installation
install(
  TARGETS see_lidar_pointcloud_prefilter see_lidar_pointcloud_clusterer see_lidar_cone_detector see_lidar_cone_identificator
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
