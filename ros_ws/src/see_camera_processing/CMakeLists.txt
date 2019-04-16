cmake_minimum_required(VERSION 2.8.3)
project(see_camera_processing)

## Specify libraries for shared use with other packages
set(LIBRARY "src/Algorithm.cpp")

## Specify source files for generation of executables only available for this package
set(EXECUTABLES "src/${PROJECT_NAME}_node.cpp" "src/SeeCameraProcessing.cpp"
  # "src/cone_detection_node.cpp" 
  # "src/coordinate_extraction_node.cpp"
  # "src/image_correction_node.cpp"  
)

## Specify catkin packages your project and dependent projects need
set(CATKIN_DEPENDENCIES "roscpp" "rospy" "message_runtime" )

## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
## Only customize below this line if you really know what you're doing
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

## Use C++11
add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    image_transport
    cv_bridge 
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ConeContainer.msg
  Cone.msg
  Coordinates.msg
  CovarianceMatrix.msg
  ConeColor.msg
  ImageCones.msg
  ImageCoordinates.msg
  ImageCone.msg
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
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  ${LIBRARY}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  ${EXECUTABLES}
)

add_executable(synthetic_data_generator
  src/synthetic_output_generator_node.cpp
)

add_executable(cone_detection
  src/cone_detection_node.cpp
)

add_executable(image_correction
  src/image_correction_node.cpp
)

add_executable(coordinate_extraction
  src/coordinate_extraction_node.cpp
)

add_executable(image_generator
  src/image_generator.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(cone_detection
  see_camera_processing_generate_messages_cpp
)
add_dependencies(image_generator
  see_camera_processing_generate_messages_cpp
)
add_dependencies(coordinate_extraction
  see_camera_processing_generate_messages_cpp
)
add_dependencies(synthetic_data_generator
  see_camera_processing_generate_messages_cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(synthetic_data_generator
  ${catkin_LIBRARIES}
)

target_link_libraries(cone_detection
  ${catkin_LIBRARIES}
)

target_link_libraries(image_correction
  ${catkin_LIBRARIES}
)

target_link_libraries(coordinate_extraction
  ${catkin_LIBRARIES}
)

target_link_libraries(image_generator
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
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

################################################
## Install Cone Detection Model Dependencies  ##
################################################
if (FALSE)
find_program(PYTHON "python")
find_program(PIP "pip")

if (PYTHON)
    set(PYTORCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/pytorch")
    set(PYTORCH_BUILD_DIR "${PYTORCH_DIR}/build")
    file(MAKE_DIRECTORY ${PYTORCH_BUILD_DIR})
    #set(ENV{USE_OPENCV} 1)

    # download, compile, and install pytorch
    add_custom_command(
        OUTPUT ${PYTORCH_DIR}/build/timestamp
        #COMMAND git clone --recursive https://github.com/pytorch/pytorch ${PYTORCH_DIR}
        COMMAND ${PIP} install -r ${PYTORCH_DIR}/requirements.txt --user
        COMMAND ${CMAKE_COMMAND} -E touch ${PYTORCH_BUILD_DIR}/timestamp
        COMMAND ${PYTHON} ${PYTORCH_DIR}/setup.py build_ext -b ${PYTORCH_BUILD_DIR} install
    #    DEPENDS ${DEPS}
    )
    
    add_custom_target(
        pytorch_target ALL DEPENDS ${PYTORCH_BUILD_DIR}/timestamp
    )
    
    #install(CODE "execute_process(COMMAND ${PYTHON} ${PYTORCH_DIR}/setup.py install")
    

    set(DETECTRON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/detectron")
    set(DETECTRON_BUILD_DIR "${DETECTRON_DIR}/build")
    file(MAKE_DIRECTORY ${DETECTRON_BUILD_DIR})
    # compile and install detectron
    add_custom_command(
        OUTPUT ${DETECTRON_DIR}/build/timestamp

        COMMAND ${PIP} install -r ${DETECTRON_DIR}/requirements.txt --user
        COMMAND ${CMAKE_COMMAND} -E touch ${DETECTRON_BUILD_DIR}/timestamp
        ${PYTHON} ${DETECTRON_DIR}/setup.py build_ext -b ${DETECTRON_BUILD_DIR} install
        DEPENDS pytorch_target
    )

    add_custom_target(
        detectron_target ALL DEPENDS ${DETECTRON_BUILD_DIR}/timestamp
    )
    #install(CODE "execute_process(COMMAND ${PYTHON} ${DETECTRON_DIR}/setup.py install)")
endif()
endif()

