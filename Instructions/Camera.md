
# How to run the camera component

1. Install prerequisites
In the top level directory run
```
cd ros_ws/src/see_camera_processing
conda install pytorch torchvision cudatoolkit=9.0 -c pytorch
pip install -r requirements.txt
echo "export PYTHONPATH=$PYTHONPATH:$PWD" >> ~/.bashrc
source ~/.bashrc
conda activate DV
```

2. Run the component
```
roslaunch launch/see_camera_processing_debug_test.launch
```
This will open four ROS nodes:
- see_camera_processing_image_generator
- see_camera_processing_image_correction
- see_camera_processing_cone_detection (the work done by our team)
- see_camera_processing_coordinate_extraction

A debugging window will appear. After a few seconds, test images will start to appear
which were read in from see_camera_processing/testdata. (During the competition,
this node will be replaced by a node which reads in images from the camera. For now, the see_camera_processing_image_generator node is just to demonstrate the capability of the cone detection node.)

