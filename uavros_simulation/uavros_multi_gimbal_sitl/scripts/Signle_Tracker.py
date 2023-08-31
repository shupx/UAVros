# This file is part of OpenCV Zoo project.
# It is subject to the license terms in the LICENSE file found in the same directory.
#
# Copyright (C) 2021, Shenzhen Institute of Artificial Intelligence and Robotics for Society, all rights reserved.
# Third party copyrights are property of their respective owners.

import argparse

import numpy as np
import cv2 as cv
import os
from dasiamrpn import DaSiamRPN
file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
# Check OpenCV version
assert cv.__version__ >= "4.7.0", \
       "Please install latest opencv-python to try this demo: python3 -m pip install --upgrade opencv-python"

# Valid combinations of backends and targets
backend_target_pairs = [
    [cv.dnn.DNN_BACKEND_OPENCV, cv.dnn.DNN_TARGET_CPU],
    [cv.dnn.DNN_BACKEND_CUDA,   cv.dnn.DNN_TARGET_CUDA],
    [cv.dnn.DNN_BACKEND_CUDA,   cv.dnn.DNN_TARGET_CUDA_FP16],
    [cv.dnn.DNN_BACKEND_TIMVX,  cv.dnn.DNN_TARGET_NPU],
    [cv.dnn.DNN_BACKEND_CANN,   cv.dnn.DNN_TARGET_NPU]
]

def visualize(image, bbox, score, isLocated, fps=None, box_color=(0, 255, 0),text_color=(0, 255, 0), fontScale = 1, fontSize = 1):
    output = image.copy()
    h, w, _ = output.shape

    if fps is not None:
        cv.putText(output, 'FPS: {:.2f}'.format(fps), (0, 30), cv.FONT_HERSHEY_DUPLEX, fontScale, text_color, fontSize)

    if isLocated and score >= 0.6:
        # bbox: Tuple of length 4
        x, y, w, h = bbox
        cv.rectangle(output, (x, y), (x+w, y+h), box_color, 2)
        cv.putText(output, '{:.2f}'.format(score), (x, y+20), cv.FONT_HERSHEY_DUPLEX, fontScale, text_color, fontSize)
    else:
        text_size, baseline = cv.getTextSize('Target lost!', cv.FONT_HERSHEY_DUPLEX, fontScale, fontSize)
        text_x = int((w - text_size[0]) / 2)
        text_y = int((h - text_size[1]) / 2)
        cv.putText(output, 'Target lost!', (text_x, text_y), cv.FONT_HERSHEY_DUPLEX, fontScale, (0, 0, 255), fontSize)

    return output

class Signle_Tracker:
    def __init__(self):
        backend_id = backend_target_pairs[1][0]
        target_id = backend_target_pairs[1][1]

    # Instantiate DaSiamRPN
        self.model = DaSiamRPN(
            kernel_cls1_path=os.path.join(dir_path,"object_tracking_dasiamrpn_kernel_cls1_2021nov.onnx"),
            kernel_r1_path=os.path.join(dir_path,"object_tracking_dasiamrpn_kernel_r1_2021nov.onnx"),
            model_path=os.path.join(dir_path,"object_tracking_dasiamrpn_model_2021nov.onnx"),
            backend_id=backend_id,
            target_id=target_id)
    def init_model(self,image,roi):
        # Init tracker with ROI
        self.model.init(image, roi)

    def update(self,image):
        isLocated, bbox, score = self.model.infer(image) #左上宽高
        return isLocated, bbox, score

