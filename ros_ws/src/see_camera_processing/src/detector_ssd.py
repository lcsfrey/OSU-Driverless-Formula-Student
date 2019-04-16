#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import os
import sys
import roslib
roslib.load_manifest('see_camera_processing')
import rospy

import argparse
import time

from multiprocessing import Process, Manager
from threading import Thread
try:
    # Using Python 3
    from queue import Empty as EmptyQueueError
    from queue import Full as FullQueueError
except ImportError:
    # Using Python 2
    from Queue import Empty as EmptyQueueError
    from Queue import Full as FullQueueError
import cv2  # NOQA (Must import before importing caffe2 due to bug in cv2)
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from see_camera_processing.msg import ImageCones, ImageCone, ConeColor, ImageCoordinates 

from ssds.lib.ssds import ObjectDetector
from ssds.lib.utils.config_parse import *

NUM_CLASSES = 3 # TODO: CHANGE TO 4 WHEN MODEL IS TRAINED ON 4 TYPES OF CONES

print("test")
def parse_args():
    parser = argparse.ArgumentParser(description='Cone Detector')
    parser.add_argument(
        '--cfg',
        dest='cfg',
        help='cfg model file (/path/to/model_config.yaml)',
        default=None,
        type=str
    )
    parser.add_argument(
        '--wts',
        dest='weights',
        help='weights model file (/path/to/model_weights.pkl)',
        default=None,
        type=str
    )
    parser.add_argument(
        '--camera-topic',
        dest='camera_topic',
        help='output image file format (default: camera/image_raw)',
        default='camera/image_raw',
        type=str
    )
    parser.add_argument(
        '--thresh',
        dest='thresh',
        help='Threshold for visualizing and publishing detections',
        default=0.6,
        type=float
    )
    parser.add_argument(
        '--display-fps',
        dest='fps',
        help='Displays fps on image if true',
        default=True,
        type=bool
    )    
    parser.add_argument(
        '-d', '--debug',
        dest='debug',
        help='Publishes image with detections overlayed to the topic /cone_detector/debug',
        default=False,
        type=bool
    )

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args, unknown = parser.parse_known_args()
    return args

def convert_from_cls_format(cls_boxes, cls_segms):
    """Convert from the class boxes/segms format generated by the testing code."""
    box_list = [b for b in cls_boxes if len(b) > 0]
    if len(box_list) > 0:
        boxes = np.concatenate(box_list)
    else:
        boxes = None
    if cls_segms is not None:
        segms = [s for slist in cls_segms for s in slist]
    else:
        segms = None

    classes = []
    for j in range(len(cls_boxes)):
        classes += [j] * len(cls_boxes[j])

    return boxes, segms, classes


def draw_bounding_boxes(im, boxes, classes, confidences):
    """Visual debugging of detections."""

    class_id_to_string = {0: "Yellow", 1: "Blue", 2:"Orange", 3: "Background"} 
    class_id_to_color = {0: (0, 190, 255), 1:(255, 0, 0), 2:(0, 0, 255), 3:(0,0,0)}

    for box, class_id, confidence in zip(boxes, classes, confidences):
        if class_id == 3:
            continue
        box_color = class_id_to_color[class_id]
        cv2.rectangle(im, (box[0], box[1]), (box[2], box[3]), color=box_color, thickness=3)
        rospy.loginfo("{: <6} | Confidence: {:.4f} | Bbox: ({:>4},{:>4}) ({:>4},{:>4})".format(
                      class_id_to_string[class_id], confidence, *box))

    if len(boxes) > 0:
        rospy.loginfo('-'*80)
    return im

def filter_good_detections(boxes, classes):
    """ Return bounding boxes with a confidence score larger than args.thresh """
    filtered_boxes, filtered_classes = [], []
    for i in range(len(boxes)):
        confidence = boxes[i, -1]
        if confidence < args.thresh or classes[i] >= NUM_CLASSES: 
            continue
        filtered_boxes.append(boxes[i])
        filtered_classes.append(classes[i])

    return np.array(filtered_boxes), tuple(filtered_classes)

class Detector:
    """
    Cone Detector subscribes to a camera topic and publishes ImageCones if detections are present
    
    subscribers: 
        args.camera_topic - Type: sensor_msgs.msg.Image - 
    publishers:
        /see_camera_processing/debug - Type: sensor_msgs.msg.Image - Image with cone detection overlayed
        /see_camera_processing/cone_boxes - Type: see_camera_processing.msg.ImageCones - Message containing bounding boxes, 

    """
    def __init__(self, args):
        self.pub_boxes = rospy.Publisher('/see_camera_processing/cone_boxes', ImageCones, queue_size=5)
        rospy.loginfo("Debug: {}".format(args.debug))
        if args.debug:

            rospy.loginfo("Publishing debugging image to /see_camera_processing/debug")
            self.pub_debug = rospy.Publisher('/see_camera_processing/debug', Image, queue_size=5)

        self.args = args
        self.manager = Manager()
        self.image_queue = self.manager.Queue(maxsize=1)
        self.detections = self.manager.Queue(maxsize=5)
        self.detect_process = Thread(target=self.detect_loop)
        self.publish_process = Thread(target=self.publish_loop)
        self.is_alive = self.manager.Value('c_bool', True)

        self.bridge = CvBridge()

        # callback for recieving images to detect
        def img_sub_callback(msg):
            try:
                # convert ROS message to opencv image
                cv_image = self.bridge.imgmsg_to_cv2(msg, str("bgr8"))
            except CvBridgeError as e:
                print(e)

            # send image to detector for inference
            # TODO: GET TIME FROM SUBSCRIBER. GETTING TIME HERE IS NOT ACCURATE
            self.submit((cv_image, rospy.Time().now()))

        rospy.Subscriber(args.camera_topic, Image, img_sub_callback, queue_size=1)

        self.start()
        
    def start(self):
        """Starts the detection and publishing loops."""
        rospy.logdebug("Starting box publisher...")
        self.publish_process.start()
        rospy.logdebug("Box publisher started.")
        time.sleep(2)
        rospy.logdebug("Starting detector loop...")
        self.detect_process.start()
        rospy.logdebug("Detector loop started.")

    def stop(self):
        """Stops the detection and publishing loops."""
        self.is_alive.value = False
        rospy.logdebug("Stopping Detector...")
        self.detect_process.join()
        rospy.logdebug("Detector stopped.")

        rospy.logdebug("Stopping box publisher...")
        self.publish_process.join()
        rospy.logdebug("Box publisher stopped.")

    def submit(self, image_data):
        """
        Submit an image to run inference on.
        image_data is a tuple (image, time_stamp)
        """
        if self.image_queue.full():
            try:
                self.image_queue.get(block=False)
            except EmptyQueueError:
                pass
        try:
            self.image_queue.put(image_data, block=False)
        except FullQueueError:
            pass

    def publish_loop(self):
        """ Thread which publishes any detections in the detection queue"""
        # CURRENT DETECTOR COLORS
        DETECTOR_COLOR_BLUE = 0
        DETECTOR_COLOR_YELLOW = 1
        DETECTOR_COLOR_ORANGE = 2 

        # CURRENT PUBLISH COLORS
        CONE_COLOR_BLUE = 0
        CONE_COLOR_YELLOW = 1
        CONE_COLOR_RED = 2
        CONE_COLOR_SMALL_ORANGE = 3
        CONE_COLOR_BIG_ORGANGE = 4
        CONE_COLOR_UNKNOWN = 5

        # used to convert between the detector's color map and 
        # the color map used by depth estimation
        detector_color_to_publish_color = {
            DETECTOR_COLOR_BLUE: CONE_COLOR_BLUE,
            DETECTOR_COLOR_YELLOW: CONE_COLOR_YELLOW,
            DETECTOR_COLOR_ORANGE: CONE_COLOR_BIG_ORGANGE
        }

        while self.is_alive.value:
            try:
                # see if the model has finished doing inference on an image
                image, (_labels, _scores, _coords), time_stamp = self.detections.get(block=True)
            except EmptyQueueError:
                # skip computation is we are still waiting on an image
                continue

            if len(_labels) == 0:
                rospy.logdebug("No boxes found.")
                if self.args.debug:
                    self.pub_debug.publish(self.bridge.cv2_to_imgmsg(image, str("bgr8")))
                continue

            if self.args.debug:
                # publish boxes to debugging window
                image = draw_bounding_boxes(image, _coords, _labels, _scores)
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(image, str("bgr8")))

            # publish the cones
            image_cones = ImageCones()
            image_cones.timestamp = time_stamp
            # TODO: DON'T USE IMAGE SHAPE BECAUSE WE MAY FEED SMALLER CROPS TO DETECTOR
            # TODO: READ IN ORIGINAL SHAPE ALONG WITH IMAGE SUBSCRIPTION
            image_cones.imageWidth = image.shape[1] 
            image_cones.imageHeight = image.shape[0]
            
            # convert x2, y2 to width and height
            _coords[:, 2] -= _coords[:, 0]
            _coords[:, 3] -= _coords[:, 1]

            # construct cones array
            # NOTE: It is significantly faster to build the array using 
            # using list comprehension shown below. Do NOT convert this
            # to an ordinary for loop.
            image_cone_tuple = tuple([
                ImageCone(
                    coordinatesTopLeft=ImageCoordinates(x=_coords[i][0], y=_coords[i][1]),
                    coneType = ConeColor(
                        color=detector_color_to_publish_color[_labels[i]],
                        probability = _scores[i]),
                    boxWidth = _coords[i][2],
                    boxHeight = _coords[i][3]
                ) for i in range(len(_coords))
            ])

            image_cones.cones = image_cone_tuple

            self.pub_boxes.publish(image_cones)


    def detect_loop(self):
        """ Thread which runs runs inference """
        global cfg
        
        cfg_from_file(self.args.cfg)
        end = self.args.cfg.find('experiments')
        root_dir = self.args.cfg[:end]
        cfg.EXP_DIR = os.path.join(root_dir, cfg.EXP_DIR)
        cfg.LOG_DIR = os.path.join(root_dir, cfg.LOG_DIR)
        print(cfg)
        rospy.loginfo("Initializing model...")
        object_detector = ObjectDetector(cfg, root_dir=root_dir)
        rospy.loginfo("Model initialized.")
        rospy.loginfo("Waiting for images on topic: {}".format(self.args.camera_topic))

        # compute bounding boxes
        while self.is_alive.value:
            # get the most recent image, blocking until one is available
            image_data = self.image_queue.get(block=True)

            (image, time_stamp) = image_data

            # run inference on image using model
            _labels, _scores, _coords = object_detector.predict(image)

            _scores = np.array([s.cpu().numpy() for s in _scores])
            _coords = np.array([c.cpu().numpy().round().astype(int) for c in _coords])  
            if len(_coords) > 0:   
                _coords[:, [0,2]] = np.clip(_coords[:, [0,2]], 0, image.shape[1]-1)
                _coords[:, [1,3]] = np.clip(_coords[:, [1,3]], 0, image.shape[0]-1)
            # add detections to publishing queue
            try:
                self.detections.put((image, (_labels, _scores, _coords), time_stamp), block=False)
            except FullQueueError:
                pass
                
        rospy.logdebug("Leaving Detector main loop...")


if __name__ == "__main__":
    rospy.init_node('see_camera_processing_cone_detection')
    args = parse_args()

    det = Detector(args)
    
    # tell the detector to shutdown when this shuts down
    rospy.on_shutdown(det.stop)

    # Give the model more time to fully load
    #time.sleep(5)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass