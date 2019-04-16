# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
import os
import argparse
import numpy as np
if '/data/software/opencv-3.4.0/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/data/software/opencv-3.4.0/lib/python2.7/dist-packages')
if '/data/software/opencv-3.3.1/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/data/software/opencv-3.3.1/lib/python2.7/dist-packages')
import cv2

from lib.ssds import ObjectDetector
from lib.utils.config_parse import *
from lib.utils.model_logging import setup_logger

VOC_CLASSES = ( 'aeroplane', 'bicycle', 'bird', 'boat',
    'bottle', 'bus', 'car', 'cat', 'chair',
    'cow', 'diningtable', 'dog', 'horse',
    'motorbike', 'person', 'pottedplant',
    'sheep', 'sofa', 'train', 'tvmonitor')

CONE_CLASSES = ('yellow', 'blue', 'orange')

def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(description='Demo a ssds.pytorch network')
    parser.add_argument('--cfg', dest='confg_file',
            help='the address of optional config file', default=None, type=str, required=True)
    parser.add_argument('--demo', dest='demo_file',
            help='the address of the demo file', default=None, type=str, required=True)
    parser.add_argument('-t', '--type', dest='type',
            help='the type of the demo file, could be "image", "video", "camera" or "time", default is "image"', default='image', type=str)
    parser.add_argument('-d', '--display', dest='display',
            help='whether display the detection result, default is False', default=False, type=bool)
    parser.add_argument('-s', '--save', dest='save',
            help='whether write the detection result, default is False', default=False, type=bool)
    parser.add_argument('-o', '--output_dir', dest='output_dir',
            help='whether write the detection result, default is False', default='model_output', type=str)
    parser.add_argument('--skip', dest='skip_count',
            help='Skip the initial n frames of video, default is 0', default=0, type=int)

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    return args


COLORS = [(255, 255, 0), (255, 0, 0), (0, 127, 255)]
FONT = cv2.FONT_HERSHEY_SIMPLEX

def demo(args, image_path):
    # 2. load detector based on the configure file
    object_detector = ObjectDetector()

    # 3. load image
    image = cv2.imread(image_path)

    # 4. detect
    _labels, _scores, _coords = object_detector.predict(image)

    # 5. draw bounding box on the image
    for labels, scores, coords in zip(_labels, _scores, _coords):
        cv2.rectangle(image, (int(coords[0]), int(coords[1])), (int(coords[2]), int(coords[3])), COLORS[labels % 3], 2)
        cv2.putText(image, '{label}: {score:.3f}'.format(label=VOC_CLASSES[labels], score=scores), (int(coords[0]), int(coords[1])), FONT, 0.5, COLORS[labels % 3], 2)
    
    # 6. visualize result
    if args.display is True:
        cv2.imshow('result', image)
        cv2.waitKey(0)

    # 7. write result
    if args.save is True:
        os.path.makedirs(args.output_dir, exists_ok=True)
        path, _ = os.path.splitext(image_path)
        cv2.imwrite(os.path.join(args.output_dir, path + '_result.jpg'), image)
    

def demo_live(args, video_path):

    # 2. load detector based on the configure file
    object_detector = ObjectDetector()

    # 3. load video
    video = cv2.VideoCapture(video_path)

    if args.save:
        path = os.path.split(os.path.splitext(video_path)[0])[1]
        output_path = os.path.join(cfg.EXP_DIR, args.output_dir, path + '_result')
        if not os.path.exists(output_path):
            os.makedirs(output_path)

    index = -1
    import time
    total_time = 0
    while(video.isOpened()):
        index = index + 1
        print('Process image: {} \r'.format(index))
        # 4. read image
        flag, image = video.read()
        if flag == False:
            print("Can not read image in Frame : {}".format(index))
            break

        if index < args.skip_count:
            continue

        # 5. detect
        _labels, _scores, _coords, times = object_detector.predict(image, check_time=True)
        total_time += times[-1]
        # 6. draw bounding box on the image
        for labels, scores, coords in zip(_labels, _scores, _coords):
            cv2.rectangle(image, (int(coords[0]), int(coords[1])), (int(coords[2]), int(coords[3])), COLORS[labels % 3], 2)
            cv2.putText(image, '{label}: {score:.3f}'.format(label=CONE_CLASSES[labels], score=scores), (int(coords[0]), int(coords[1])), FONT, 0.5, COLORS[labels % 3], 2)

        # 7. visualize result
        if args.display is True:
            cv2.imshow('result', image)
            cv2.waitKey(33)

        # 8. write result
        if args.save is True:
            cv2.imwrite(output_path + '/{}.jpg'.format(index), image)
    print("Post-process time", total_time / (index+1))
    if args.save:
        print("Images saved to", output_path)


def time_benchmark(args, image_path):
    # 1. load the configure file
    cfg_from_file(args.confg_file)

    # 2. load detector based on the configure file
    object_detector = ObjectDetector()

    # 3. load image
    image = cv2.imread(image_path)

    # 4. time test
    warmup = 20
    time_iter = 100
    print('Warmup the detector...')
    _t = list()
    for i in range(warmup+time_iter):
        _, _, _, (total_time, preprocess_time, net_forward_time, detect_time, output_time) \
            = object_detector.predict(image, check_time=True)
        if i > warmup:
            _t.append([total_time, preprocess_time, net_forward_time, detect_time, output_time])
            if i % 20 == 0: 
                print('In {}\{}, total time: {} \n preprocess: {} \n net_forward: {} \n detect: {} \n output: {}'.format(
                    i-warmup, time_iter, total_time, preprocess_time, net_forward_time, detect_time, output_time
                ))
    total_time, preprocess_time, net_forward_time, detect_time, output_time = np.sum(_t, axis=0)/time_iter * 1000 # 1000ms to 1s
    print('In average, total time: {}ms \n preprocess: {}ms \n net_forward: {}ms \n detect: {}ms \n output: {}ms'.format(
        total_time, preprocess_time, net_forward_time, detect_time, output_time
    ))
    with open('./time_benchmark.csv', 'a') as f:
        f.write("{:s},{:.2f}ms,{:.2f}ms,{:.2f}ms,{:.2f}ms,{:.2f}ms\n".format(args.confg_file, total_time, preprocess_time, net_forward_time, detect_time, output_time))


    
if __name__ == '__main__':
    args = parse_args()
    # 1. load the configure file
    cfg_from_file(args.confg_file)

    logger = setup_logger(cfg)

    if args.type == 'image':
        demo(args, args.demo_file)
    elif args.type == 'video':
        demo_live(args, args.demo_file)
    elif args.type == 'camera':
        demo_live(args, int(args.demo_file))
    elif args.type == 'time':
        time_benchmark(args, args.demo_file)
    else:
        AssertionError('type is not correct')
