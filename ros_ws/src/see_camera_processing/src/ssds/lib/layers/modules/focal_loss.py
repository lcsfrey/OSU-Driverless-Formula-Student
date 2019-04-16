# -*- coding: utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import numpy as np

from lib.utils.box_utils import match, log_sum_exp, one_hot_embedding

import logging
logger = logging.getLogger(__name__)

# I do not fully understand this part, It completely based on https://github.com/kuangliu/pytorch-retinanet/blob/master/loss.py

class FocalLoss(nn.Module):
    """SSD Weighted Loss Function
    Focal Loss for Dense Object Detection.
        
        Loss(x, class) = - \alpha (1-softmax(x)[class])^gamma \log(softmax(x)[class])

    The losses are averaged across observations for each minibatch.
    Args:
        alpha(1D Tensor, Variable) : the scalar factor for this criterion
        gamma(float, double) : gamma > 0; reduces the relative loss for well-classiﬁed examples (p > .5), 
                                putting more focus on hard, misclassiﬁed examples
        size_average(bool): size_average(bool): By default, the losses are averaged over observations for each minibatch.
                            However, if the field size_average is set to False, the losses are
                            instead summed for each minibatch.
    """

    def __init__(self, cfg, priors, use_gpu=True, size_average=False):
        super(FocalLoss, self).__init__()
        self.use_gpu = use_gpu
        self.num_classes = cfg.NUM_CLASSES
        self.background_label = cfg.BACKGROUND_LABEL
        self.negpos_ratio = cfg.NEGPOS_RATIO
        self.threshold = cfg.MATCHED_THRESHOLD
        self.unmatched_threshold = cfg.UNMATCHED_THRESHOLD
        self.variance = cfg.VARIANCE
        self.priors = priors

        #self.alpha = Variable(torch.ones(self.num_classes, 1) * cfg.alpha)
        self.gamma = cfg.FOCAL_LOSS_GAMMA
        self.size_average = size_average


    def forward(self, predictions, targets, size_average=False):
        """Multibox Loss
        Args:
            predictions (tuple): A tuple containing loc preds, conf preds,
            and prior boxes from SSD net.
                conf shape: torch.size(batch_size,num_priors,num_classes)
                loc shape: torch.size(batch_size,num_priors,4)
                priors shape: torch.size(num_priors,4)
            ground_truth (tensor): Ground truth boxes and labels for a batch,
                shape: [batch_size,num_objs,5] (last idx is the label).
        """
        loc_data, conf_data = predictions
        num = loc_data.size(0)
        priors = self.priors
        # priors = priors[:loc_data.size(1), :]
        num_priors = (priors.size(0))
        
        # match priors (default boxes) and ground truth boxes
        loc_t = torch.tensor(num, num_priors, 4)
        conf_t = torch.tensor(num, num_priors)
        for idx in range(num):
            truths = targets[idx][:,:-1].data
            labels = targets[idx][:,-1].data
            defaults = priors.data
            match(self.threshold,truths,defaults,self.variance,labels,loc_t,conf_t,idx)
        if self.use_gpu:
            loc_t = loc_t.cuda()
            conf_t = conf_t.cuda()
        # wrap targets
        #loc_t = (loc_t, requires_grad=False)
        #conf_t = Variable(conf_t,requires_grad=False)

        pos = conf_t > 0
        num_pos = pos.sum()

        # Localization Loss (Smooth L1)
        # Shape: [batch,num_priors,4]
        pos_idx = pos.unsqueeze(pos.dim()).expand_as(loc_data)
        loc_p = loc_data[pos_idx].view(-1,4)
        loc_t = loc_t[pos_idx].view(-1,4)
        loss_l = F.smooth_l1_loss(loc_p, loc_t, size_average=size_average)
        loss_l/=num_pos.data.sum()

        # Confidence Loss (Focal loss)
        # Shape: [batch,num_priors,1]
        loss_c = self.focal_loss(conf_data.view(-1, self.num_classes), conf_t.view(-1,1))

        return loss_l,loss_c

    def focal_loss(self, inputs, targets):
        '''Focal loss.
        mean of losses: L(x,c,l,g) = (Lconf(x, c) + αLloc(x,l,g)) / N
        '''
        N = inputs.size(0)
        C = inputs.size(1)
        P = F.softmax(inputs)
        
        class_mask = inputs.data.new(N, C).fill_(0)
        class_mask = Variable(class_mask)
        ids = targets.view(-1, 1)
        class_mask.scatter_(1, ids.data, 1.)
        labels =  targets[:, -1].detach().cpu().clone().numpy().round().astype(np.int32)
        class_sample_count = np.unique(labels, return_counts=True)[1]
        weight = 1. / class_sample_count
        logger.debug("weight:", weight)
        samples_weight = weight[labels]
        logger.debug("sample weight:", samples_weight)
        alpha = torch.from_numpy(weight)

        if inputs.is_cuda and not alpha.is_cuda:
            logger.debug("input is cuda")
            alpha = alpha.type(inputs.dtype).cuda()
        logger.debug(type(alpha))
        logger.debug(type(inputs))
        alpha = alpha[ids.data.view(-1)]
        probs = (P*class_mask).sum(1).view(-1,1)
        log_p = probs.log()
        logger.debug(type(log_p))
        
        step1 = torch.pow((1-probs), self.gamma)
        logger.debug("here")
        step2 = step1 * log_p
        logger.debug("here")
        step3 = step2 * -alpha
        logger.debug("here")
        batch_loss = -alpha*(torch.pow((1-probs), self.gamma))*log_p 
        logger.debug(batch_loss.dtype)

        loss = batch_loss.mean()
        return loss
