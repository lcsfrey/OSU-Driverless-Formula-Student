# see_camera_processing

This node implements basic ROS communication interfaces.

## Installation

Before running node, execute the following command:

  `cd gfr19d_ws/src/see_camera_processing/src/install` \
  `bash install_all.sh` 
  
This will call three separate scripts to install all required dependencies of the cone detector model. These three scripts install:
* CUDA 9.0 and CUDNN 7.0
* Pytorch (Machine learning library) and all of its dependencies
* Detectron (Object detection model library) and all of its dependencies

## Usage

After starting `roscore` in another terminal, execute the following commands.

  `cd gfr19d_ws`\
  `catkin_make`\
  `source devel/setup.bash` \
  `rosrun see_camera_processing see_camera_processing`

Currently, the cone detector must be launched separately using a launch file. The launch files `launch/see_camera_processing.launch` and `launch/see_camera_processing_debug.launch` can be used. the debug launch file also have the node publish an additional debug topic containing the image with bounding boxes overlayed on the cones. The debug launch file will also open a window for viewing the debugging image.

## Nodes

### see_camera_processing

Offers publication, subscription and service interfaces.


#### Subscribed Topics

* **`see_camera_processing_subscription`** [std_msgs/String]

  Receives a string and does nothing.

#### Published Topics

* **`see_camera_processing_publication`** [std_msgs/String]

  Publishes a string with a frequency of 1 Hz.
  
#### Services

* **`see_camera_processing_service`** [std_srvs/Trigger]

  Can be polled for string retrieval.
  
  
  
### see_camera_processing_detection

Subscribes to image topic, detects cones within the image, and publishes bounding boxes as well as the original image with bounding boxes overlayed for debugging purposes.  

The cone detector requires two arguments: 
* `--cfg CONFIG_PATH` the full path to the model configuration file 
* `--wts WEIGHT_PATH` the full path to the model weights file

Optional parameters include:
* `--thresh THRESH_VAL` a float value between 0 and 1 corresponding to the level of confidence needed for a prediction to be considered a cone. (default: 0.6)
* `--camera-topic TOPIC_NAME` the topic name the node subscribes to to get images. (default: 'camera/image_raw') 
* `--debug` boolean flag for debugging purposes. If true, an image with the bounding boxes overlayed will be published to see_camera_processing/debug (default: True)

#### Subscribed Topics
  
* **`camera/image_raw`** [sensor_msgs/Image]

  Receives an image and detects cones in the image. 

#### Published Topics

* **`see_camera_processing/cone_boxes`** [see_camera_processing/ConeBoxes]

  Bounding boxes of cones in image with timestamp of when image was recieved.
  
* **`see_camera_processing/debug`** [sensor_msgs/Image]

  Image with bounding boxes overlayed on detected cones.
  

