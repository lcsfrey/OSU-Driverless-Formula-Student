# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import os
import sys
import cv2
import random
import pickle
import time
from collections import defaultdict
import logging


import torch
import torch.backends.cudnn as cudnn
import torch.optim as optim
from torch.optim import lr_scheduler
#import torch.utils.data as data
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

logger = logging.getLogger(__name__)

try:
    unicode
except NameError:
    unicode = str

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
        logger.info("Building model...")
        self.model, self.priorbox = create_model(cfg.MODEL)
        with torch.no_grad():
            self.priors = self.priorbox.forward()
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

        logger.debug('Parameters and size:')
        for name, param in self.model.named_parameters():
            logger.debug('{}: {}'.format(name, list(param.size())))

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
            logger.info("no checkpoint found at \'{}\'".format(resume_checkpoint))
            return False
        logger.info("loading checkpoint \'{:s}\'".format(resume_checkpoint))
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
        #self.export_graph()

        # warm_up epoch
        warm_up = self.cfg.TRAIN.LR_SCHEDULER.WARM_UP_EPOCHS
        for epoch in iter(range(start_epoch+1, self.max_epochs+1)):
            #learning rate
            if epoch > warm_up:
                self.exp_lr_scheduler.step(epoch-warm_up)
            for phase in self.cfg.PHASE:
                self.run_epoch(self.model, self.loaders[phase], self.detector, 
                               self.optimizer, self.criterion, phase, epoch)

            if epoch % cfg.TRAIN.CHECKPOINTS_EPOCHS == 0:
                self.save_checkpoints(epoch)

    def compute_losses(self, criterion, preds, targets):
        try:
            loss_l, loss_c = criterion(preds, targets)
        except Exception as e:
            logger.error("ERROR")
            logger.error(e)
            logger.error("MODEL OUTPUT:")
            logger.error(preds)
            logger.error([t.size() for t in preds[0]])
            logger.error([t.size() for t in preds[1]])
            logger.error("TARGET")
            logger.error(targets)
            logger.error([t.size() for t in targets])
            raise e

        # some bugs in coco train2017. maybe the annonation bug.
        if loss_l.data.item() == float("Inf"):
            logger.error("{} loss_l is inf/nan".format(phase))
        if loss_c.data.item() == float("Inf"):
            logger.error("{} loss_c is inf/nan".format(phase))
        return {'loss_l': loss_l, 'loss_c': loss_c}


    def run_epoch(self, model, loader, detector, optimizer, criterion, phase, epoch):
        logger.info("Current phase: {}".format(phase))
        if phase == 'train':
            model.train()
        else:
            model.eval()
            label     = [list() for _ in range(model.num_classes)]
            gt_label  = [list() for _ in range(model.num_classes)]
            score     = [list() for _ in range(model.num_classes)]
            size      = [list() for _ in range(model.num_classes)]
            npos      = [ 0     for _ in range(model.num_classes)]
            all_boxes = []

        if phase == 'visualize':
            loader.dataset.preproc.add_writer(self.writer, epoch)

        epoch_size     = len(loader)
        batch_iterator = iter(loader)

        loc_loss  = 0
        conf_loss = 0

        _t = defaultdict(Timer)
        #_t = {'data_augmentation': Timer(), 'model_forward': Timer(), 'model_backward':Timer(), 
        #      'loss_calc': Timer(), 'total_time': Timer(), 'current_batch': Timer()}
        _t['total_time'].tic()

        for batch_num in range(epoch_size):
            _t['current_batch'].clear()
            _t['current_batch'].tic()
            _t['data_augmentation'].tic()
            images, targets = next(batch_iterator)
            _t['data_augmentation'].toc()

            if self.use_gpu:
                images  = images.cuda()
                targets = [target.cuda() for target in targets]

            self.optimizer.zero_grad()

            with torch.set_grad_enabled(phase == 'train'):

                # forward
                _t['model_forward'].tic()
                out = model(images, phase=('train' if phase in ['train', 'eval'] else 'eval'))
                _t['model_forward'].toc()

                if phase in ['train', 'eval']:

                    _t['loss_calc'].tic()
                    losses = self.compute_losses(criterion, out, targets)
                    loss = sum(losses.values())
                    _t['loss_calc'].toc()

                    loc_loss  += losses['loss_l'].data.item()
                    conf_loss += losses['loss_c'].data.item()

                if phase == 'train':
                    # backward
                    _t['model_backward'].tic()
                    loss.backward()
                    self.optimizer.step()
                    _t['model_backward'].toc()
                else:
                    if phase == 'eval':
                        _t['model_forward'].tic()
                        out = (out[0], model.softmax(out[1].view(-1, model.num_classes)))
                        _t['model_forward'].toc()
                        
                    # run nms and get thresholded class bounding boxes
                    detections, det_times = detector.forward(out, return_time=True)
                    logger.debug(det_times)
                    
                    if phase == 'eval':
                        _t['calc_tp_fp'].tic()
                        label, score, npos, gt_label = cal_tp_fp(
                            detections, targets, label, score, npos, gt_label)
                        #size = cal_size(detections, targets, size)
                        _t['calc_tp_fp'].toc()
                        
                    if phase in ['test', 'visualize']:
                        _t['box_conversion'].tic()
                        scale = [images[0].shape[1], images[0].shape[0], 
                                 images[0].shape[1], images[0].shape[0]]
                        all_boxes.append([])
                        # TODO: DISPLAY ALL IMAGES
                        for j in range(1, len(detections[0])):
                            raw_cls_dets = detections[0][j].cpu().numpy()
                            cls_dets = [np.append(d[1:]*scale, d[0]) for d in raw_cls_dets
                                        if d[0] > self.cfg.POST_PROCESS.SCORE_THRESHOLD]
                            all_boxes[-1].append(cls_dets)
                        _t['box_conversion'].toc()
                progress = '#'*int(round(10*batch_num/epoch_size)) \
                           + '-'*int(round(10*(1-(batch_num/epoch_size))))

                _t['current_batch'].toc()
                log = '\r|| {phase} || epoch {epoch:}/{max_epochs} | '\
                      'batch {batch_num:d}/{epoch_size:d} | {toc_time:.3f}s | '\
                      '[{progress}]'

                log_stats = dict(
                    phase=phase, epoch=epoch, max_epochs=self.max_epochs,
                    batch_num=batch_num, epoch_size=epoch_size,
                    toc_time=_t['current_batch'].total_time, progress=progress)

                if phase in ['train', 'eval']:
                    log += '|| loc_loss: {loc_loss:.4f} conf_loss: {conf_loss:.4f}'
                    log_stats['loc_loss']  = loc_loss/(batch_num+1)
                    log_stats['conf_loss'] = conf_loss/(batch_num+1)
                logger.info(log.format(**log_stats))

                if phase == 'visualize':
                    break # only visualize one set of images

        _t['total_time'].toc()

        # log per epoch
        log = '\r|| {phase} || Total_time: {time:.3f}s '
        log_stats = dict(phase=phase, time=_t['total_time'].total_time)
        # log for tensorboard
        if phase in ['train', 'eval']:
            self.writer.add_scalar('{phase}/loc_loss'.format(phase=phase), loc_loss/epoch_size, epoch)
            self.writer.add_scalar('{phase}/conf_loss'.format(phase=phase), conf_loss/epoch_size, epoch)
            log += '|| loc_loss: {loc_loss:.4f} conf_loss: {conf_loss:.4f} '
            log_stats['loc_loss']  = loc_loss/(batch_num+1)
            log_stats['conf_loss'] = conf_loss/(batch_num+1)

        if phase == 'train':
            lr = self.optimizer.param_groups[0]['lr']
            self.writer.add_scalar('train/lr', lr, epoch)
            log += '|| lr: {lr:.6f} '
            log_stats['lr'] = lr

        if phase == 'eval':
            prec, rec, ap = cal_pr(label, score, npos)
            self.writer.add_scalar('eval/mAP', ap, epoch)
            viz_pr_curve(self.writer, prec, rec, epoch)
            #viz_archor_strategy(writer, size, gt_label, epoch)
            log += '|| ap: {ap: .4f} '
            log_stats['ap'] = ap

        if phase == 'test':
            # write result to pkl
            with open(os.path.join(output_dir, 'detections.pkl'), 'wb') as f:
                pickle.dump(all_boxes, f, pickle.HIGHEST_PROTOCOL)

            # currently the COCO dataset do not return the mean ap or ap 0.5:0.95 values
            logger.info('Evaluating detections')
            loader.dataset.evaluate_detections(all_boxes, output_dir)

        if phase == 'visualize':
            # BUG
            #print(images.size())
            image = images[0].cpu().numpy().transpose(1,2,0)
            #print(image.shape)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            #_t['viz_detections'] = Timer()
            _t['viz_detections'].tic()
            viz_detections(self.writer, [image], [all_boxes[0]], epoch=epoch, name='eval')
            _t['viz_detections'].toc()

            _t['viz_feature_maps'] = Timer()
            _t['viz_feature_maps'].tic()
            with torch.no_grad():
                # visualize feature map in base and extras
                base_out = viz_module_feature_maps(
                    self.writer, model.base, images[0].unsqueeze(0),
                    module_name='base', epoch=epoch)
                extras_out = viz_module_feature_maps(
                    self.writer, model.extras, base_out,
                    module_name='extras', epoch=epoch)

                # visualize feature map in feature_extractors
                features = model(images[0].unsqueeze(0), 'feature')
            viz_feature_maps(self.writer, features, module_name='feature_extractors', epoch=epoch)
            _t['viz_feature_maps'].toc()
            
            #model.train()
            #prepped_images.requires_grad = True
            #base_out = viz_module_grads(self.writer, model, model.base, prepped_images, prepped_images, preproc.means, module_name='base',

        for time_name, t in _t.items():
            self.writer.add_scalar('{phase}/time/{time_name}'.format(
                phase=phase, time_name=time_name), t.total_time/epoch_size, epoch)        

        # log everything for the epoch
        logger.info(log.format(**log_stats))

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
