# -*- coding: utf-8 -*-
import os
import pickle
import os.path
import sys
import torch
import torch.utils.data as data
import torchvision.transforms as transforms
import cv2
import numpy as np
import json
import uuid

from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
from pycocotools import mask as COCOmask

import logging
logger = logging.getLogger(__name__)

class CONEDetection(data.Dataset):

    """Cone Detection Dataset Object

    input is image, target is annotation

    Arguments:
        root (string): filepath to VOCdevkit folder.
        image_set (string): imageset to use (eg. 'train', 'val', 'test')
        transform (callable, optional): transformation to perform on the
            input image
        target_transform (callable, optional): transformation to perform on the
            target `annotation`
            (eg: take in caption string, return tensor of word indices)
        dataset_name (string, optional): which dataset to load
            (default: 'VOC2007')
    """

    def __init__(self, root, image_sets, preproc=None, target_transform=None,
                 dataset_name='CONES'):
        self.root = root
        self.cache_path = os.path.join(self.root, 'cache')
        self.image_set = image_sets
        self.preproc = preproc
        self.target_transform = target_transform
        self.name = dataset_name
        self.ids = list()
        self.annotations = list()

        for (data_set, image_set) in image_sets:
            annofile = os.path.join(self.root, 'annotations', data_set + '.json')
            _COCO = COCO(annofile)
            self._COCO = _COCO
            self.coco_name = data_set
            cats = _COCO.loadCats(_COCO.getCatIds())
            self._classes = tuple(['__background__'] + [c['name'] for c in cats])
            self.num_classes = len(self._classes)
            self._class_to_ind = dict(zip(self._classes, range(self.num_classes)))
            self._class_to_coco_cat_id = dict(zip([c['name'] for c in cats],
                                                  _COCO.getCatIds()))
            indexes = _COCO.getImgIds()
            self.image_indexes = indexes
            # seems it will reduce the speed during the training.

            all_paths = self._load_coco_img_path(indexes)
            good_paths, good_indexes = [], []
            for i, path in zip(indexes, all_paths):
                annIds = self._COCO.getAnnIds(imgIds=i, iscrowd=None)
                if len(annIds) > 0 and image_set in path:
                    good_paths.append(path)
                    good_indexes.append(i)
            self.ids.extend(good_paths)
            if image_set.find('test') != -1:
                print('test set will not load annotations!')
            else:
                self.annotations.extend(self._load_coco_annotations(self.coco_name, good_indexes, _COCO))


    def image_path_from_index(self, index):
        """
        Construct an image path from the image's "index" identifier.
        """
        # Example image path for index=119993:
        #   images/train2014/COCO_train2014_000000119993.jpg
        # file_name = ('COCO_' + name + '_' +
        #              str(index).zfill(12) + '.jpg')
        # change Example image path for index=119993 to images/train2017/000000119993.jpg
        path = self._COCO.loadImgs(index)[0]['file_name']
        image_path = os.path.join(self.root, path)
        assert os.path.exists(image_path), \
                'Path does not exist: {}'.format(image_path)
        return image_path


    def _load_coco_annotations(self, coco_name, indexes, _COCO):
#        cache_file=os.path.join(self.cache_path,coco_name+'_gt_roidb.pkl')
#        if os.path.exists(cache_file):
#            with open(cache_file, 'rb') as fid:
#                roidb = pickle.load(fid)
#            print('{} gt roidb loaded from {}'.format(coco_name,cache_file))
#            return roidb

#        print('parsing gt roidb for {}'.format(coco_name))
        gt_roidb = [self._annotation_from_index(index) for index in indexes]
#        with open(cache_file, 'wb') as fid:
#            pickle.dump(gt_roidb,fid,pickle.HIGHEST_PROTOCOL)
#        print('wrote gt roidb to {}'.format(cache_file))
        return gt_roidb

    def _load_coco_img_path(self, indexes):
#        cache_file=os.path.join(self.cache_path, coco_name+'_img_path.pkl')
#        if os.path.exists(cache_file):
#            with open(cache_file, 'rb') as fid:
#                img_path = pickle.load(fid)
#            print('{} img path loaded from {}'.format(coco_name,cache_file))
#            return img_path

#        print('parsing img path for {}'.format(coco_name))
        img_path = [self.image_path_from_index(index) for index in indexes]
#        with open(cache_file, 'wb') as fid:
#            pickle.dump(img_path,fid,pickle.HIGHEST_PROTOCOL)
#        print('wrote img path to {}'.format(cache_file))
        return img_path

    def _annotation_from_index(self, index):
        """
        Loads COCO bounding-box instance annotations. Crowd instances are
        handled by marking their overlaps (with all categories) to -1. This
        overlap value means that crowd "instances" are excluded from training.
        """
        im_ann = self._COCO.loadImgs(index)[0]
        width = im_ann['width']
        height = im_ann['height']

        annIds = self._COCO.getAnnIds(imgIds=index, iscrowd=None)
        objs = self._COCO.loadAnns(annIds)
        # Sanitize bboxes -- some are invalid
        valid_objs = []
        for obj in objs:
            x1 = np.max((0, obj['bbox'][0]))
            y1 = np.max((0, obj['bbox'][1]))
            x2 = np.min((width - 1, x1 + np.max((0, obj['bbox'][2] - 1))))
            y2 = np.min((height - 1, y1 + np.max((0, obj['bbox'][3] - 1))))
            if obj['area'] > 0 and x2 >= x1 and y2 >= y1:
                obj['clean_bbox'] = [x1, y1, x2, y2]
                valid_objs.append(obj)
        objs = valid_objs
        num_objs = len(objs)

        res = np.zeros((num_objs, 5))

        # Lookup table to map from COCO category ids to our internal class
        # indices
        coco_cat_id_to_class_ind = dict([(self._class_to_coco_cat_id[cls],
                                          self._class_to_ind[cls])
                                         for cls in self._classes[1:]])

        for ix, obj in enumerate(objs):
            cls = coco_cat_id_to_class_ind[obj['category_id']]
            res[ix, 0:4] = obj['clean_bbox']
            res[ix, 4] = cls

        return res

    def __getitem__(self, index):
        img_path = self.ids[index]
        target = self.annotations[index]

        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        height, width, _ = img.shape

        if self.target_transform is not None:
            target = self.target_transform(target)

        if self.preproc is not None:
            img, target = self.preproc(img, target)

        return img, target

    def __len__(self):
        return len(self.ids)

    def pull_image(self, index):
        '''Returns the original image object at index in numpy form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            PIL img
        '''
        img_id = self.ids[index]
        return cv2.imread(img_id, cv2.IMREAD_COLOR)


    def pull_anno(self, index):
        '''Returns the original annotation of image at index

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to get annotation of
        Return:
            list:  [img_id, [(label, bbox coords),...]]
                eg: ('001718', [('dog', (96, 13, 438, 332))])
        '''
        anno = self.annotations[index]
        if self.target_transform is not None:
            anno = self.target_transform(anno)
        return anno


    def pull_tensor(self, index):
        '''Returns the original image at an index in tensor form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            tensorized version of img, squeezed
        '''
        return torch.from_numpy(self.pull_image(index)).unsqueeze_(0)

    def _print_detection_eval_metrics(self, coco_eval):
        IoU_lo_thresh = 0.5
        IoU_hi_thresh = 0.95
        def _get_thr_ind(coco_eval, thr):
            ind = np.where((coco_eval.params.iouThrs > thr - 1e-5) &
                           (coco_eval.params.iouThrs < thr + 1e-5))[0][0]
            iou_thr = coco_eval.params.iouThrs[ind]
            assert np.isclose(iou_thr, thr)
            return ind

        ind_lo = _get_thr_ind(coco_eval, IoU_lo_thresh)
        ind_hi = _get_thr_ind(coco_eval, IoU_hi_thresh)
        # precision has dims (iou, recall, cls, area range, max dets)
        # area range index 0: all area ranges
        # max dets index 2: 100 per image
        precision = \
            coco_eval.eval['precision'][ind_lo:(ind_hi + 1), :, :, 0, 2]
        ap_default = np.mean(precision[precision > -1])
        logger.info('~~~~ Mean and per-category AP @ IoU=[{:.2f},{:.2f}] '
               '~~~~'.format(IoU_lo_thresh, IoU_hi_thresh))
        logger.info('{:.1f}'.format(100 * ap_default))
        for cls_ind, cls in enumerate(self._classes):
            if cls == '__background__':
                continue
            # minus 1 because of __background__
            precision = coco_eval.eval['precision'][ind_lo:(ind_hi + 1), :, cls_ind - 1, 0, 2]
            ap = np.mean(precision[precision > -1])
            logger.info('{:.1f}'.format(100 * ap))

        logger.info('~~~~ Summary metrics ~~~~')
        coco_eval.summarize()

    def _do_detection_eval(self, res_file, output_dir):
        ann_type = 'bbox'
        coco_dt = self._COCO.loadRes(res_file)
        coco_eval = COCOeval(self._COCO, coco_dt)
        coco_eval.params.useSegm = (ann_type == 'segm')
        coco_eval.evaluate()
        coco_eval.accumulate()
        self._print_detection_eval_metrics(coco_eval)
        eval_file = os.path.join(output_dir, 'detection_results.pkl')
        with open(eval_file, 'wb') as fid:
            pickle.dump(coco_eval, fid, pickle.HIGHEST_PROTOCOL)
        logger.info('Wrote COCO eval results to: {}'.format(eval_file))

    def _coco_results_one_category(self, boxes, cat_id):
        results = []
        for im_ind, index in enumerate(self.image_indexes):
            dets = boxes[im_ind].astype(np.float)
            if dets == []:
                continue
            scores = dets[:, -1]
            xs = dets[:, 0]
            ys = dets[:, 1]
            ws = dets[:, 2] - xs + 1
            hs = dets[:, 3] - ys + 1
            results.extend(
              [{'image_id' : index,
                'category_id' : cat_id,
                'bbox' : [xs[k], ys[k], ws[k], hs[k]],
                'score' : scores[k]} for k in range(dets.shape[0])])
        return results

    def _write_coco_results_file(self, all_boxes, res_file):
        # [{"image_id": 42,
        #   "category_id": 18,
        #   "bbox": [258.15,41.29,348.26,243.78],
        #   "score": 0.236}, ...]
        results = []
        for cls_ind, cls in enumerate(self._classes):
            if cls == '__background__':
                continue
            logger.info('Collecting {} results ({:d}/{:d})'.format(cls, cls_ind,
                                                          self.num_classes ))
            coco_cat_id = self._class_to_coco_cat_id[cls]
            results.extend(self._coco_results_one_category(all_boxes[cls_ind],
                                                           coco_cat_id))
            '''
            if cls_ind ==30:
                res_f = res_file+ '_1.json'
                logger.info('Writing results json to {}'.format(res_f))
                with open(res_f, 'w') as fid:
                    json.dump(results, fid)
                results = []
            '''
        #res_f2 = res_file+'_2.json'
        logger.info('Writing results json to {}'.format(res_file))
        with open(res_file, 'w') as fid:
            json.dump(results, fid)

    def evaluate_detections(self, all_boxes, output_dir):
        res_file = os.path.join(output_dir, ('detections_' +
                                         self.coco_name +
                                         '_results'))
        res_file += '.json'
        self._write_coco_results_file(all_boxes, res_file)
        # Only do evaluation on non-test sets
        if self.coco_name.find('test') == -1:
            self._do_detection_eval(res_file, output_dir)
