cmake_minimum_required(VERSION 2.8.3)
project(see_ego_motion)

## Specify libraries for shared use with other packages
set(LIBRARY "src/Algorithm.cpp")

## Specify source files for generation of executables only available for this package
set(BASE_EXECUTABLES "src/${PROJECT_NAME}_node.cpp" "src/SeeEgoMotion.cpp")
set(TEST_EXECUTABLES "src/testing/${PROJECT_NAME}_test_node.cpp" "src/testing/SeeEgoMotionTest.cpp")

## Specify catkin packages your project and dependent projects need
set(CATKIN_DEPENDENCIES "roscpp" "message_generation" "message_runtime")

## Specify messages files for generation of message headers for this package
set(MESSAGE_FILES "Dynamics.msg" "OdomDeltas.msg" "see_ego_motion_interface.msg" "Error.msg")

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
)

add_message_files(FILES ${MESSAGE_FILES})
generate_messages(DEPENDENCIES std_msgs)

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

add_dependencies(${PROJECT_NAME}_core ${PROJECT_NAME}_generate_messages_cpp)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  ${BASE_EXECUTABLES}
)

add_executable(${PROJECT_NAME}_test
  ${TEST_EXECUTABLES}
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

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