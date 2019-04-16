MODEL:
  SSDS: fssd
  NETS: resnet_se_a
  HALF_PRECISION: False
  IMAGE_SIZE: [512, 512]
  NUM_CLASSES: 4
  FEATURE_LAYER: [[[10, 14, 'S'], [512, 1024, 512]],
                  [['', 'S', 'S'], [512, 512, 256]]]
 # FEATURE_LAYER: [[[10, 16, 'S'], [512, 1024, 512]],
 #                 [['', 'S', 'S', 'S', '', ''], [512, 512, 256, 256, 256, 256]]]
  
 
  # anchor stride
  STEPS: [] 
# [[4, 4], [8, 8], [16, 16], [64, 64], [100, 100]]
  # scales
  SIZES: [[15, 15], [30, 30], [60, 60], [120, 120]] #, [213, 213], [264, 264]]
  ASPECT_RATIOS: [[1, 2, 3], [1, 2, 3], [1, 2, 3]]
#, [1, 2, 3], [1, 2]]

TRAIN:
  MAX_EPOCHS: 150
  CHECKPOINTS_EPOCHS: 10
  BATCH_SIZE: 8
  TRAINABLE_SCOPE: 'base,norm,extras,transforms,pyramids,loc,conf'
  RESUME_SCOPE: 'base,norm,extras,transforms,pyramids,loc,conf'
  OPTIMIZER:
    OPTIMIZER: sgd
    LEARNING_RATE: 0.0015
    MOMENTUM: 0.9
    WEIGHT_DECAY: 0.0001
  LR_SCHEDULER:
    SCHEDULER: exponential
    STEPS: [30, 50, 90, 120]
    GAMMA: .98
    WARM_UP_EPOCHS: 10

TEST:
  BATCH_SIZE: 64
  TEST_SCOPE: [90, 100]

MATCHER:
  MATCHED_THRESHOLD: 0.5
  UNMATCHED_THRESHOLD: 0.5
  NEGPOS_RATIO: 3
  #USE_FOCAL_LOSS: True
  #FOCAL_LOSS_GAMMA: 2

POST_PROCESS:
  SCORE_THRESHOLD: 0.01
  IOU_THRESHOLD: 0.6
  MAX_DETECTIONS: 100

DATASET:
  DATASET: 'cones'
  DATASET_DIR: '/home/gfr_admin/lucas/data/cones-all'
  TRAIN_SETS: [['cones_all', 'cones-2'], ['cones_all','cones-4']]
  TEST_SETS:  [['cones_all', 'cones-3']]
  PROB: 0.6
  TEST_IMAGE_SIZE: [512, 512]

EXP_DIR: './experiments/models/fssd_resnet50_spatial_a_cones'
LOG_DIR: './experiments/models/fssd_resnet50_spatial_a_cones/logs'
RESUME_CHECKPOINT: 'latest' #'/fssd_resnet_se_a_cones_epoch_280.pth'
PHASE: ['train', 'eval', 'visualize']
