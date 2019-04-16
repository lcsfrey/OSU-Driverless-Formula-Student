# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import os
import sys
import cv2
import random
import pickle
import time
import logging
logger = logging.getLogger(__name__)

import torch
import torch.backends.cudnn as cudnn
import torch.optim as optim
from torch.optim import lr_scheduler
import torch.utils.data as data
import torch.nn.init as init

from tensorboardX import SummaryWriter

from lib.layers import *
from lib.utils.timer import Timer
from lib.utils.data_augment import preproc
from lib.modeling.model_builder import create_model
from lib.dataset.dataset_factory import load_data
from lib.utils.config_parse import cfg
from lib.utils.eval_utils import *
from lib.utils.visualize_utils import *


class Solver(object):
    """
    A wrapper class for the training process
    """
    def __init__(self):
        self.cfg = cfg

        # Load data
        logger.info('Loading data')
        self.loaders = {}
        if isinstance(cfg.PHASE, (str, unicode)):
            cfg.PHASE = [cfg.PHASE]

        for phase in cfg.PHASE: 
            self.loaders[phase] = load_data(cfg.DATASET, phase)

        # Build model
        logger.info('===> Building model')
        self.model, self.priorbox = create_model(cfg.MODEL)
        with torch.no_grad():
            self.priors = torch.tensor(self.priorbox.forward())
        self.detector = Detect(cfg.POST_PROCESS, self.priors)

        # Utilize GPUs for computation
        self.use_gpu = torch.cuda.is_available()
        if self.use_gpu:
            logger.info('Utilize GPUs for computation')
            logger.info('Number of GPU available: {}'.format(torch.cuda.device_count()))
            self.model.cuda()
            self.priors.cuda()
            cudnn.benchmark = True
            # if torch.cuda.device_count() > 1:
                # self.model = torch.nn.DataParallel(self.model).module

        # Print the model architecture and parameters
        logger.info('Model architectures:\n{}\n'.format(self.model))

        logger.info('Parameters and size:')
        for name, param in self.model.named_parameters():
            logger.info('{}: {}'.format(name, list(param.size())))

        # print trainable scope
        logger.debug('Trainable scope: {}'.format(cfg.TRAIN.TRAINABLE_SCOPE))
        trainable_param = self.trainable_param(cfg.TRAIN.TRAINABLE_SCOPE)
        self.optimizer = self.configure_optimizer(trainable_param, cfg.TRAIN.OPTIMIZER)
        self.exp_lr_scheduler = self.configure_lr_scheduler(self.optimizer, cfg.TRAIN.LR_SCHEDULER)
        self.max_epochs = cfg.TRAIN.MAX_EPOCHS

        # metric
        loss_func = FocalLoss if cfg.MATCHER.USE_FOCAL_LOSS else MultiBoxLoss
        self.criterion = loss_func(cfg.MATCHER, self.priors, self.use_gpu) 
 
        # Set the logger
        self.writer = SummaryWriter(log_dir=cfg.LOG_DIR)
        self.output_dir = cfg.EXP_DIR
        self.checkpoint = cfg.RESUME_CHECKPOINT
        self.checkpoints_kept = cfg.TRAIN.CHECKPOINTS_KEPT
        self.checkpoint_prefix = cfg.CHECKPOINTS_PREFIX


    def save_checkpoints(self, epochs, iters=None):
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        if iters:
            filename = self.checkpoint_prefix + '_epoch_{:d}_iter_{:d}'.format(epochs, iters) + '.pth'
        else:
            filename = self.checkpoint_prefix + '_epoch_{:d}'.format(epochs) + '.pth'
        filename = os.path.join(self.output_dir, filename)
        torch.save(self.model.state_dict(), filename)

        checkpoints_list_file = os.path.join(self.output_dir, 'checkpoints_list.txt')

        # count the number of checkpoints
        try:
            with open(checkpoints_list_file, 'r') as fin:
                checkpoint_lines = fin.read().splitlines(True)
                num_checkpoints = len(data)
        except IOError:
            num_checkpoints = 0

        if num_checkpoints > self.checkpoints_kept:
            # delete oldest checkpoint
            checkpoint_file = line[line.find(':') + 2:-1]
            os.remove(os.path.join(self.output_dir, checkpoint_file))

            # rewrite checkpoint list without oldest checkpoint
            with open(checkpoints_list_file, 'w') as fout:
                fout.writelines(checkpoint_lines[1:])

        with open(os.path.join(self.output_dir, 'checkpoint_list.txt'), 'a') as f:
            f.write('epoch {epoch:d}: {filename}\n'.format(epoch=epochs, filename=filename))
        logger.debug('Wrote snapshot to: {:s}'.format(filename))

        # TODO: write relative cfg under the same page

    def resume_checkpoint(self, resume_checkpoint):
        if resume_checkpoint == '' or not os.path.isfile(resume_checkpoint):
            logger.info(("=> no checkpoint found at '{}'".format(resume_checkpoint)))
            return False
        logger.info(("=> loading checkpoint '{:s}'".format(resume_checkpoint)))
        if resume_checkpoint == 'latest':
            previous = self.find_previous()
            if previous:
                start_epoch = previous[0][-1]
                self.resume_checkpoint(previous[1][-1])
            else:
                return False

        checkpoint = torch.load(resume_checkpoint)

        # print("=> Weigths in the checkpoints:")
        # print([k for k, v in list(checkpoint.items())])

        # remove the module in the parrallel model
        if 'module.' in list(checkpoint.items())[0][0]:
            pretrained_dict = {'.'.join(k.split('.')[1:]): v for k, v in list(checkpoint.items())}
            checkpoint = pretrained_dict

        # change the name of the weights which exists in other model
        # change_dict = {
        #         'conv1.weight':'base.0.weight',
        #         'bn1.running_mean':'base.1.running_mean',
        #         'bn1.running_var':'base.1.running_var',
        #         'bn1.bias':'base.1.bias',
        #         'bn1.weight':'base.1.weight',
        #         }
        # for k, v in list(checkpoint.items()):
        #     for _k, _v in list(change_dict.items()):
        #         if _k == k:
        #             new_key = k.replace(_k, _v)
        #             checkpoint[new_key] = checkpoint.pop(k)
        # change_dict = {'layer1.{:d}.'.format(i):'base.{:d}.'.format(i+4) for i in range(20)}
        # change_dict.update({'layer2.{:d}.'.format(i):'base.{:d}.'.format(i+7) for i in range(20)})
        # change_dict.update({'layer3.{:d}.'.format(i):'base.{:d}.'.format(i+11) for i in range(30)})
        # for k, v in list(checkpoint.items()):
        #     for _k, _v in list(change_dict.items()):
        #         if _k in k:
        #             new_key = k.replace(_k, _v)
        #             checkpoint[new_key] = checkpoint.pop(k)

        resume_scope = self.cfg.TRAIN.RESUME_SCOPE
        # extract the weights based on the resume scope
        if resume_scope != '':
            pretrained_dict = {}
            for k, v in list(checkpoint.items()):
                for resume_key in resume_scope.split(','):
                    if resume_key in k:
                        pretrained_dict[k] = v
                        break
            checkpoint = pretrained_dict

        pretrained_dict = {k: v for k, v in checkpoint.items() if k in self.model.state_dict()}
        # print("=> Resume weigths:")
        # print([k for k, v in list(pretrained_dict.items())])

        checkpoint = self.model.state_dict()

        unresume_dict = set(checkpoint)-set(pretrained_dict)
        if len(unresume_dict) != 0:
            logger.debug("=> UNResume weigths:")
            logger.debug(unresume_dict)

        checkpoint.update(pretrained_dict)

        return self.model.load_state_dict(checkpoint)


    def find_previous(self):
        if not os.path.exists(os.path.join(self.output_dir, 'checkpoint_list.txt')):
            return False
        with open(os.path.join(self.output_dir, 'checkpoint_list.txt'), 'r') as f:
            lineList = f.readlines()
        epoches, resume_checkpoints = [list() for _ in range(2)]
        for line in lineList:
            epoch = int(line[line.find('epoch ') + len('epoch '): line.find(':')])
            checkpoint = line[line.find(':') + 2:-1]
            epoches.append(epoch)
            resume_checkpoints.append(checkpoint)
        return epoches, resume_checkpoints

    def weights_init(self, m):
        for key in m.state_dict():
            if key.split('.')[-1] == 'weight':
                if 'conv' in key:
                    init.kaiming_normal(m.state_dict()[key], mode='fan_out')
                if 'bn' in key:
                    m.state_dict()[key][...] = 1
            elif key.split('.')[-1] == 'bias':
                m.state_dict()[key][...] = 0


    def initialize(self):
        # TODO: ADD INIT ways
        # raise ValueError("Fan in and fan out can not be computed for tensor with less than 2 dimensions")
        # for module in self.cfg.TRAIN.TRAINABLE_SCOPE.split(','):
        #     if hasattr(self.model, module):
        #         getattr(self.model, module).apply(self.weights_init)
        if self.checkpoint:
            logger.info('Loading initial model weights from {:s}'.format(self.checkpoint))
            self.resume_checkpoint(self.checkpoint)

        start_epoch = 0
        return start_epoch

    def trainable_param(self, trainable_scope):
        for param in self.model.parameters():
            param.requires_grad = False

        trainable_param = []
        for module in trainable_scope.split(','):
            if hasattr(self.model, module):
                # print(getattr(self.model, module))
                for param in getattr(self.model, module).parameters():
                    param.requires_grad = True
                trainable_param.extend(getattr(self.model, module).parameters())

        return trainable_param

    def train_model(self):
        previous = self.find_previous()
        if previous:
            start_epoch = previous[0][-1]
            self.resume_checkpoint(previous[1][-1])
        else:
            start_epoch = self.initialize()

        # export graph for the model, onnx always not works
        self.export_graph()

        # warm_up epoch
        warm_up = self.cfg.TRAIN.LR_SCHEDULER.WARM_UP_EPOCHS
        for epoch in iter(range(start_epoch+1, self.max_epochs+1)):
            #learning rate
            if epoch > warm_up:
                self.exp_lr_scheduler.step(epoch-warm_up)
            for phase in self.cfg.PHASES:
                self.run_epoch(self.model, self.loaders[phase], self.detector, phase, epoch)

            if epoch % cfg.TRAIN.CHECKPOINTS_EPOCHS == 0:
                self.save_checkpoints(epoch)

    def test_model(self):
        pass

    def compute_losses(self, criterion, preds, targets):
        try:
            loss_l, loss_c = self.criterion(out, targets)
        except Exception as e:
            logger.error("ERROR")
            logger.error(e)
            logger.error("MODEL OUTPUT:")
            logger.error(out)
            logger.error("TARGET")
            logger.error(targets)
        raise e

        # some bugs in coco train2017. maybe the annonation bug.
        if loss_l.data.item() == float("Inf"):
            logger.error("{} loss_l is inf/nan".format(phase))
            continue
        if loss_c.data.item() == float("Inf"):
            logger.error("{} loss_c is inf/nan".format(phase))
            continue
        return {'loss_l': loss_l, 'loss_c': loss_c}


    def run_epoch(self, model, loader, detector, phase, epoch):
        if phase == 'train':
            model.train()
        else:
            model.eval()
            if phase == 'test':
                label    = [list() for _ in range(model.num_classes)]
                gt_label = [list() for _ in range(model.num_classes)]
                score    = [list() for _ in range(model.num_classes)]
                size     = [list() for _ in range(model.num_classes)]
                npos     = [ 0     for _ in range(model.num_classes)]

        if phase == 'visualize':
            self.loaders[phase].preproc.add_writer(self.writer, epoch)
        if phase in ['test', 'visualize']:
            all_boxes = []

        epoch_size     = len(loader)
        batch_iterator = iter(loader)

        loc_loss  = 0
        conf_loss = 0

        _t = {'forward': Timer(), 'loss_calc': Timer(),
              'backward':Timer(), 'total_time': Timer()}
        time_taken = defaultdict(float)
        _t['total_time'].tic()

        for batch_num in range(epoch_size):
            images, targets = next(batch_iterator)

            if self.use_gpu:
                images  = images.cuda()
                targets = targets.cuda()

            # forward
            _t['forward'].tic()
            out = model(images, phase=phase)
            time_taken['model_forward'] += _t['forward'].toc()

            optimizer.zero_grad()

            with torch.set_grad_enabled(phase == 'train'):
                _t['loss_calc'].tic()
                losses = self.compute_loss(criterion, preds, targets)
                loss = sum(losses.values())
                time_taken['loss_calc'] += _t['loss_calc'].toc()

                loc_loss  += losses['loss_l'].data.item()
                conf_loss += losses['loss_c'].data.item()

                if phase == 'train':
                    # backward
                    _t['backwark'].tic()
                    loss.backward()
                    optimizer.step()
                    time_taken['model_backward'] += _t['backwark'].toc()

                # log per batch
                log = '\r|| {phase} || {iters:d}/{epoch_size:d} in {toc_time:.3f}s [{prograss}] || loc_loss: {loc_loss:.4f} cls_loss: {cls_loss:.4f}\r'.format(
                    phase=phase, prograss='#'*int(round(10*i/epoch_size)) + '-'*int(round(10*(1-(i/epoch_size)))), 
                    iters=i, epoch_size=epoch_size, toc_time=time_taken['total_time'], 
                    loc_loss=loc_loss/(i+1), cls_loss=conf_loss/(i+1))
                logger.info(log)

                if phase != 'train':
                    # run nms and get thresholded class bounding boxes
                    out = (out[0], model.softmax(out[1].view(-1, model.num_classes)))
                    detections = detector.forward(out)

                if phase == 'eval':
                    label, score, npos, gt_label = cal_tp_fp(
                        detections, targets, label, score, npos, gt_label)
                    size = cal_size(detections, targets, size

                if phase in ['test', 'visualize']:
                    scale = [images[0].shape[1], images[0].shape[0], 
                             images[0].shape[1], images[0].shape[0]]
                
                if phase == 'visualize':
                    # TODO: DISPLAY ALL IMAGES
                    images = images[0]
                    for j in range(1, len(detections[0])):
                        raw_cls_dets = [d.cpu().numpy() for d in detections[0][j]]
                        cls_dets = [np.append(d[1:]*scale, d[0]) for d in raw_cls_dets
                                    if d[0] > self.cfg.POST_PROCESS.SCORE_THRESHOLD]
                        all_boxes.append(cls_dets)

    time_taken['total_time'] = _t['total_time'].toc()

    # log per epoch
    lr = optimizer.param_groups[0]['lr']
    log = '\r|| {phase} || Total_time: {time:.3f}s || loc_loss: {loc_loss:.4f} conf_loss: {conf_loss:.4f} || lr: {lr:.6f}\n'.format(
        phase=phase, time=time_taken['total_time'], 
        loc_loss=loc_loss/epoch_size, conf_loss=conf_loss/epoch_size, lr=lr)
    logger.info(log)

    # log for tensorboard
    for time_name, t in time_taken.items()
        self.writer.add_scalar('{phase}/time/{t}'.format(phase=phase, t=t/epoch_size), epoch)

    if phase in ['train', 'eval']:
        self.writer.add_scalar('{phase}/loc_loss', loc_loss/epoch_size, epoch)
        self.writer.add_scalar('{phase}/conf_loss', conf_loss/epoch_size, epoch)

    if phase == 'train':
        self.writer.add_scalar('{phase}/lr', lr, epoch)

    if phase == 'eval':
        prec, rec, ap = cal_pr(label, score, npos)
        writer.add_scalar('Eval/mAP', ap, epoch)
        viz_pr_curve(writer, prec, rec, epoch)
        #viz_archor_strategy(writer, size, gt_label, epoch)

    if phase == 'test':
        # write result to pkl
        with open(os.path.join(output_dir, 'detections.pkl'), 'wb') as f:
            pickle.dump(all_boxes, f, pickle.HIGHEST_PROTOCOL)

        # currently the COCO dataset do not return the mean ap or ap 0.5:0.95 values
        logger.info('Evaluating detections')
        data_loader.dataset.evaluate_detections(all_boxes, output_dir)

    if phase == 'visualize':
        # BUG
        image = images.cpu().numpy()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        viz_detections(self.writer, [image], [all_boxes], epoch=epoch, name='eval')

        # visualize feature map in base and extras
        base_out = viz_module_feature_maps(self.writer, model.base, prepped_images, module_name='base', epoch=epoch)
        extras_out = viz_module_feature_maps(self.writer, model.extras, base_out, module_name='extras', epoch=epoch)

        # visualize feature map in feature_extractors
        features = model(image, 'feature')
        viz_feature_maps(self.writer, features, module_name='feature_extractors', epoch=epoch)
        
        #model.train()
        #prepped_images.requires_grad = True
        #base_out = viz_module_grads(writer, model, model.base, prepped_images, prepped_images, preproc.means, module_name='base',

    def train_epoch(self, model, data_loader, optimizer, criterion, writer, epoch, use_gpu):
        pass

    def eval_epoch(self, model, data_loader, detector, criterion, writer, epoch, use_gpu):
        pass

    def test_epoch(self, model, data_loader, detector, output_dir, use_gpu):
        pass

    def visualize_epoch(self, model, data_loader, priorbox, detector, writer, epoch, use_gpu):
        pass


    def configure_optimizer(self, trainable_param, cfg):
        if cfg.OPTIMIZER == 'sgd':
            optimizer = optim.SGD(trainable_param, lr=cfg.LEARNING_RATE,
                        momentum=cfg.MOMENTUM, weight_decay=cfg.WEIGHT_DECAY)
        elif cfg.OPTIMIZER == 'rmsprop':
            optimizer = optim.RMSprop(trainable_param, lr=cfg.LEARNING_RATE,
                        momentum=cfg.MOMENTUM, alpha=cfg.MOMENTUM_2, eps=cfg.EPS, weight_decay=cfg.WEIGHT_DECAY)
        elif cfg.OPTIMIZER == 'adam':
            optimizer = optim.Adam(trainable_param, lr=cfg.LEARNING_RATE,
                        betas=(cfg.MOMENTUM, cfg.MOMENTUM_2), eps=cfg.EPS, weight_decay=cfg.WEIGHT_DECAY)
        else:
            AssertionError('optimizer can not be recognized.')
        return optimizer


    def configure_lr_scheduler(self, optimizer, cfg):
        if cfg.SCHEDULER == 'step':
            scheduler = lr_scheduler.StepLR(optimizer, step_size=cfg.STEPS[0], gamma=cfg.GAMMA)
        elif cfg.SCHEDULER == 'multi_step':
            scheduler = lr_scheduler.MultiStepLR(optimizer, milestones=cfg.STEPS, gamma=cfg.GAMMA)
        elif cfg.SCHEDULER == 'exponential':
            scheduler = lr_scheduler.ExponentialLR(optimizer, gamma=cfg.GAMMA)
        elif cfg.SCHEDULER == 'SGDR':
            scheduler = lr_scheduler.CosineAnnealingLR(optimizer, T_max=cfg.MAX_EPOCHS)
        else:
            AssertionError('scheduler can not be recognized.')
        return scheduler


    def export_graph(self):
        self.model.eval()
        dummy_input = torch.tensor(torch.randn(1, 3, cfg.MODEL.IMAGE_SIZE[0], cfg.MODEL.IMAGE_SIZE[1])).cuda()
        self.writer.add_graph(self.model, (dummy_input, ))

        # Export the model
        #torch_out = torch.onnx._export(self.model,             # model being run
        #                               dummy_input,            # model input (or a tuple for multiple inputs)
        #                               "graph.onnx",           # where to save the model (can be a file or file-like object)
        #                               export_params=True)     # store the trained parameter weights inside the model file
        # if not os.path.exists(cfg.EXP_DIR):
        #     os.makedirs(cfg.EXP_DIR)


def train_model():
    s = Solver()
    s.train_model()
    return True
