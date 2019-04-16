# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import numpy as np

import torch
from torch.autograd import Variable
import torch.backends.cudnn as cudnn

from lib.layers import *
from lib.utils.timer import Timer
from lib.utils.data_augment import preproc
from lib.modeling.model_builder import create_model
from lib.utils.config_parse import cfg
from lib.utils.fp16_utils import BN_convert_float

import logging
logger = logging.getLogger(__name__)

class ObjectDetector:
    def __init__(self, input_cfg=None, root_dir=None, viz_arch=False):
        global cfg
        self.cfg = cfg if input_cfg is None else input_cfg
        cfg = self.cfg
        # Build model
        logger.info('===> Building model')
        self.model, self.priorbox = create_model(cfg.MODEL)
        self.priors = self.priorbox.forward()

        # Print the model architecture and parameters
        if viz_arch is True:
            logger.info('Model architectures:\n{}\n'.format(self.model))

        # Utilize GPUs for computation
        self.use_gpu = torch.cuda.is_available()
        #self.half = False
        self.half = cfg.MODEL.HALF_PRECISION
        if self.half:
            self.model = BN_convert_float(self.model.half())
            self.priors = self.priors.half()
        if self.use_gpu:
            logger.info('Utilize GPUs for computation')
            logger.info('Number of GPU available: {}'.format(torch.cuda.device_count()))
            self.model.cuda()
            self.priors.cuda()
            cudnn.benchmark = True
            # self.model = torch.nn.DataParallel(self.model).module
            # Utilize half precision

        # Build preprocessor and detector
        self.preprocessor = preproc(cfg.MODEL.IMAGE_SIZE, cfg.DATASET.PIXEL_MEANS, -2)
        self.detector = Detect(cfg.POST_PROCESS, self.priors)

        # Load weight:
        if cfg.RESUME_CHECKPOINT == '':
            AssertionError('RESUME_CHECKPOINT can not be empty')
        
        if cfg.RESUME_CHECKPOINT == 'latest':
            checkpoint_file = os.path.join(cfg.EXP_DIR, 'checkpoint_list.txt')
            assert os.path.exists(checkpoint_file), \
                'RESUME_CHECKPOINT set to \'latest\' but {} does not exist'.format(checkpoint_file)
            with open(checkpoint_file, 'r') as f:
                last_line = f.readlines()[-1]
            checkpoint_file = last_line[last_line.find(':') + 2:-1]
        else:
            checkpoint_file = cfg.RESUME_CHECKPOINT

        if root_dir is not None:
            checkpoint_file = os.path.join(root_dir, checkpoint_file)

        logger.info('=> loading checkpoint {:s}'.format(checkpoint_file))

        checkpoint = torch.load(checkpoint_file)
        self.model.load_state_dict(checkpoint)

        # test only
        self.model.eval()


    def predict(self, img, threshold=0.6, check_time=False):
        # make sure the input channel is 3 
        assert img.shape[2] == 3

        _t = {'preprocess': Timer(), 'net_forward': Timer(), 'detect': Timer(), 'output': Timer()}

        # preprocess image
        _t['preprocess'].tic()
        x = self.preprocessor(img)[0].unsqueeze(0)
        if self.half:
            x = x.half()
        if self.use_gpu:
            x = x.cuda()
        preprocess_time = _t['preprocess'].toc()

        # forward
        _t['net_forward'].tic()
        out = self.model(x)  # forward pass
        net_forward_time = _t['net_forward'].toc()

        # detect
        _t['detect'].tic()
        detections = self.detector.forward(out)
        detect_time = _t['detect'].toc()

        scale = torch.tensor([img.shape[1::-1], img.shape[1::-1]],
                             dtype=detections.dtype).view(-1)

        # output
        _t['output'].tic()
        labels, scores, coords = [], [], []

        batch=0

        for classes in range(detections.size(1)):
            # ALTERNATIVE 1
#            filtered_detections = detections[batch, classes]
#            filtered_detections = filtered_detections[filtered_detections[:, 0] >= threshold]

#            det_coords = filtered_detections[:, 1:]*scale

#            scores.extend(filtered_detections[:, 0].flatten().tolist())
#            labels.extend([classes-1]*filtered_detections[:, 0].numel())
#            coords.extend(det_coords.tolist())

            num = 0
            while num < detections.size(3) and detections[batch,classes,num,0] >= threshold:
                scores.append(detections[batch,classes,num,0])
                labels.append(classes-1)
                #logger.info(batch, classes, num)
                detection = detections[batch, classes, num, 1:]
            #    #logger.info(detection)
                #logger.info(detection.shape)
                #logger.info(scale)
                coords.append(detection*scale)
                num+=1
        output_time = _t['output'].toc()
        total_time = preprocess_time + net_forward_time + detect_time + output_time
        
        if check_time is True:
            return labels, scores, coords, (total_time, preprocess_time, net_forward_time, detect_time, output_time)
            # total_time = preprocess_time + net_forward_time + detect_time + output_time
            # logger.info('total time: {} \n preprocess: {} \n net_forward: {} \n detect: {} \n output: {}'.format(
            #     total_time, preprocess_time, net_forward_time, detect_time, output_time
            # ))
        return labels, scores, coords
