
# -*- coding: utf-8 -*-
import torch
import logging
logger = logging.getLogger(__name__)

# ssds part
from lib.modeling.ssds import ssd
from lib.modeling.ssds import ssd_lite
from lib.modeling.ssds import rfb
from lib.modeling.ssds import rfb_lite
from lib.modeling.ssds import fssd
from lib.modeling.ssds import fssd_lite
from lib.modeling.ssds import yolo

ssds_map = {
    'ssd': ssd.build_ssd,
    'ssd_lite': ssd_lite.build_ssd_lite,
    'rfb': rfb.build_rfb,
    'rfb_lite': rfb_lite.build_rfb_lite,
    'fssd': fssd.build_fssd,
    'fssd_lite': fssd_lite.build_fssd_lite,
    'yolo_v2': yolo.build_yolo_v2,
    'yolo_v3': yolo.build_yolo_v3,
}

# nets part
from lib.modeling.nets import vgg
from lib.modeling.nets import resnet
from lib.modeling.nets import mobilenet
from lib.modeling.nets import darknet
networks_map = {
    'vgg16': vgg.vgg16,
    'resnet_18': resnet.resnet_18,
    'resnet_34': resnet.resnet_34,
    'resnet_50': resnet.resnet_50,
    'resnet_101': resnet.resnet_101,
    'mobilenet_v1': mobilenet.mobilenet_v1,
    'mobilenet_v1_075': mobilenet.mobilenet_v1_075,
    'mobilenet_v1_050': mobilenet.mobilenet_v1_050,
    'mobilenet_v1_025': mobilenet.mobilenet_v1_025,
    'mobilenet_v2': mobilenet.mobilenet_v2,
    'mobilenet_v2_075': mobilenet.mobilenet_v2_075,
    'mobilenet_v2_050': mobilenet.mobilenet_v2_050,
    'mobilenet_v2_025': mobilenet.mobilenet_v2_025,
    'darknet_19': darknet.darknet_19,
    'darknet_53': darknet.darknet_53,


    # ----------------------------------------------
    # Lucas' implementations
    # ----------------------------------------------
    'resnet_se_a': resnet.resnet_spatial_expansion_a,
    'resnet_se_b': resnet.resnet_spatial_expansion_b,
}
 
from lib.layers.functions.prior_box import PriorBox

def _forward_features_size(model, img_size):
    model.eval()
    x = torch.rand(1, 3, img_size[0], img_size[1])
    feature_maps = model(x, phase='feature')
    return [(o.size()[2], o.size()[3]) for o in feature_maps]


def create_model(cfg):
    '''
    '''
    # Base of network takes in image and outputs
    # a list of feautre maps at different scales
    net_base = networks_map[cfg.NETS]

    # Head of network takes in a list of feature maps
    # and outputs a list of tensors corresponding to
    # the location and class predictions of objects.
    ssd_head = ssds_map[cfg.SSDS]

    number_box = [
        (2*len(aspect_ratios)
         if isinstance(aspect_ratios[0], int)
         else len(aspect_ratios))
        for aspect_ratios in cfg.ASPECT_RATIOS
    ]

    # Combine base and head of network
    model = ssd_head(
        base=net_base,
        feature_layer=cfg.FEATURE_LAYER,
        mbox=number_box,
        num_classes=cfg.NUM_CLASSES)

    logger.info('Model blocks:')
    logger.info(model)

    feature_maps = _forward_features_size(model, cfg.IMAGE_SIZE)
    logger.info('Feature map size:')
    logger.info(feature_maps)

    # PriorBox takes in a list of feature maps and outputs
    # a tensor containing a list of bounding box priors used
    # to match location and class predictions
    priorbox = PriorBox(image_size=cfg.IMAGE_SIZE, feature_maps=feature_maps,
                        aspect_ratios=cfg.ASPECT_RATIOS, scale=cfg.SIZES,
                        archor_stride=cfg.STEPS, clip=cfg.CLIP)
    return model, priorbox
