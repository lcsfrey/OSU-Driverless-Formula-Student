# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Function
from torch.autograd import Variable
from lib.utils.box_utils import decode,nms
# from lib.utils.nms.nms_wrapper import nms
from lib.utils.timer import Timer

import logging
logger = logging.getLogger(__name__)

class Detect(Function):
    """At test time, Detect is the final layer of SSD.  Decode location preds,
    apply non-maximum suppression to location predictions based on conf
    scores and threshold to a top_k number of output predictions for both
    confidence score and locations.
    """
    def __init__(self, cfg, priors):
        self.num_classes = cfg.NUM_CLASSES
        self.background_label = cfg.BACKGROUND_LABEL
        self.conf_thresh = cfg.SCORE_THRESHOLD
        self.nms_thresh = cfg.IOU_THRESHOLD 
        self.top_k = cfg.MAX_DETECTIONS 
        self.variance = cfg.VARIANCE
        self.priors = priors

    # def forward(self, predictions, prior):
    #     """
    #     Args:
    #         loc_data: (tensor) Loc preds from loc layers
    #             Shape: [batch,num_priors*4]
    #         conf_data: (tensor) Shape: Conf preds from conf layers
    #             Shape: [batch*num_priors,num_classes]
    #         prior_data: (tensor) Prior boxes and variances from priorbox layers
    #             Shape: [1,num_priors,4]
    #     """
    #     loc, conf = predictions

    #     loc_data = loc.data
    #     conf_data = conf.data
    #     prior_data = prior.data

    #     num = loc_data.size(0)  # batch size
    #     num_priors = prior_data.size(0)
    #     self.boxes = torch.zeros(1, num_priors, 4)
    #     self.scores = torch.zeros(1, num_priors, self.num_classes)

    #     if num == 1:
    #         # size batch x num_classes x num_priors
    #         conf_preds = conf_data.unsqueeze(0)

    #     else:
    #         conf_preds = conf_data.view(num, num_priors,
    #                                     self.num_classes)
    #         self.boxes.expand_(num, num_priors, 4)
    #         self.scores.expand_(num, num_priors, self.num_classes)

    #     # Decode predictions into bboxes.
    #     for i in range(num):
    #         decoded_boxes = decode(loc_data[i], prior_data, self.variance)
    #         # For each class, perform nms
    #         conf_scores = conf_preds[i].clone()
    #         '''
    #         c_mask = conf_scores.gt(self.thresh)
    #         decoded_boxes = decoded_boxes[c_mask]
    #         conf_scores = conf_scores[c_mask]
    #         '''

    #         conf_scores = conf_preds[i].clone()
    #         num_det = 0
    #         for cl in range(1, self.num_classes):
    #             c_mask = conf_scores[cl].gt(self.conf_thresh)
    #             scores = conf_scores[cl][c_mask]
    #             if scores.dim() == 0:
    #                 continue
    #             l_mask = c_mask.unsqueeze(1).expand_as(decoded_boxes)
    #             boxes = decoded_boxes[l_mask].view(-1, 4)
    #             ids, count = nms(boxes, scores, self.nms_thresh, self.top_k)
    #             self.output[i, cl, :count] = \
    #                 torch.cat((scores[ids[:count]].unsqueeze(1),
    #                            boxes[ids[:count]]), 1)

    #     return self.output


    def forward(self, predictions, return_time=False):
        """
        Args:
            loc_data: (tensor) Loc preds from loc layers
                Shape: [batch,num_priors*4]
            conf_data: (tensor) Shape: Conf preds from conf layers
                Shape: [batch*num_priors,num_classes]
            prior_data: (tensor) Prior boxes and variances from priorbox layers
                Shape: [1,num_priors,4]
        """
        loc, conf = predictions

        loc_data = loc.data
        conf_data = conf.data
        prior_data = self.priors.data

        num = loc_data.size(0)  # batch size
        num_priors = prior_data.size(0)
        #self.output.zero_()
        if num == 1:
            # size batch x num_classes x num_priors
            conf_preds = conf_data.t().contiguous().unsqueeze(0)
        else:
            conf_preds = conf_data.view(num, num_priors,
                                        self.num_classes).transpose(2, 1)
            #self.output.expand_(num, self.num_classes, self.top_k, 5)
        output = torch.zeros(num, self.num_classes, self.top_k, 5)

        _t = {'decode': Timer(), 'misc': Timer(), 'box_mask':Timer(), 'score_mask':Timer(),'nms':Timer(), 'cpu':Timer(),'sort':Timer()}
        from collections import defaultdict
        time_taken = defaultdict(float)
        _t['misc'].tic()
        # Decode predictions into bboxes.
        for i in range(num):
            _t['decode'].tic()
            decoded_boxes = decode(loc_data[i], prior_data, self.variance)
            time_taken['decode'] += _t['decode'].toc()

            # For each class, perform nms
            conf_scores = conf_preds[i].clone()
            num_det = 0
            for cl in range(1, self.num_classes):

                _t['cpu'].tic()
                c_mask = conf_scores[cl].gt(self.conf_thresh).nonzero().view(-1)
                time_taken['cpu'] +=_t['cpu'].toc()

                if c_mask.dim() == 0:
                    continue

                _t['score_mask'].tic()
                scores = conf_scores[cl][c_mask]
                time_taken['score_mask'] +=_t['score_mask'].toc()

                if scores.dim() == 0:
                    continue

                _t['box_mask'].tic()
                # l_mask = c_mask.unsqueeze(1).expand_as(decoded_boxes)
                # boxes = decoded_boxes[l_mask].view(-1, 4)
                boxes = decoded_boxes[c_mask, :]
                time_taken['box_mask'] +=_t['box_mask'].toc()

                # idx of highest scoring and non-overlapping boxes per class
                _t['nms'].tic()
                # cls_dets = torch.cat((boxes, scores), 1)
                # _, order = torch.sort(scores, 0, True)
                # cls_dets = cls_dets[order]
                # keep = nms(cls_dets, self.nms_thresh)
                # cls_dets = cls_dets[keep.view(-1).long()]
                try:
                    ids, count = nms(boxes, scores, self.nms_thresh, self.top_k)
                except Error as e:
                    logger.error(e)
                    logger.debug("BOXES: \n{} \nSCORES: {}\nTHRESH: {}\nTOP_K: {}".format(
                                  boxes, scores, self.nms_thresh, self.top_k))

                time_taken['gpu_nms'] += _t['nms'].toc()

                output[i, cl, :count] = \
                    torch.cat((scores[ids[:count]].unsqueeze(1), boxes[ids[:count]]), 1) \
       
        time_taken['nms_total'] = _t['misc'].toc()
        #keys = time_taken.keys()
        #time_str_format = "{}: {:.4f}s | "

        #str_stats = 'NMS TIME TAKEN - ' + ''.join([
        #    time_str_format.format(k, time_taken[k]) 
        #    for k in ['cpu', 'gpu_nms', 'nms_total']
        #])
        #logger.debug(str_stats)

        # print(nms_time, cpu_tims, scores_time,box_time,gpunms_time)
        # flt = self.output.view(-1, 5)
        # _, idx = flt[:, 0].sort(0)
        # _, rank = idx.sort(0)
        # flt[(rank >= self.top_k).unsqueeze(1).expand_as(flt)].fill_(0)
        if return_time:
            return output, time_taken
        else:
            return output
