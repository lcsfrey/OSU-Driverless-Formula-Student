uint16 count # number of boxes in list

# flat array of box coordinates arranged in form
# [box1_x1, box1_y1, box1_x2, box1_y2, box2_x1, box2_y1, ...]
uint16[] boxes

# confidence scores of each box. higher confidence means higher likelihood
# that the prediction is correct
float32[] confidences

# integer corresponding to the color of the cone
# 0: Orange
# 1: Yellow 
# 2: Blue 
uint16[] classes

# time when image was originally taken
time time_stamp
