#!/usr/bin/env python
# -*- coding: utf8 *-*

from __future__ import absolute_import, division, print_function

import os
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from omegaconf import OmegaConf

from libs.models import *
from libs.utils import DenseCRF

def get_device(cuda):
    cuda = cuda and torch.cuda.is_available()
    
    device = torch.device("cuda" if cuda else "cpu")

    if cuda:
        current_device = torch.cuda.current_device()
        print("Device:", torch.cuda.get_device_name(current_device))
    else:
        print("Device: CPU")
    return device


def get_classtable(CONFIG):
    with open(CONFIG.DATASET.LABELS) as f:
        classes = {}
        for label in f:
            label = label.rstrip().split("\t")
            classes[int(label[0])] = label[1].split(",")[0]
    return classes


def setup_postprocessor(CONFIG):
    # CRF post-processor
    postprocessor = DenseCRF(
        iter_max=CONFIG.CRF.ITER_MAX,
        pos_xy_std=CONFIG.CRF.POS_XY_STD,
        pos_w=CONFIG.CRF.POS_W,
        bi_xy_std=CONFIG.CRF.BI_XY_STD,
        bi_rgb_std=CONFIG.CRF.BI_RGB_STD,
        bi_w=CONFIG.CRF.BI_W,
    )
    return postprocessor


def preprocessing(image, device, CONFIG):
    # Resize （推断时不进行resize）
    # scale = CONFIG.IMAGE.SIZE.TEST / max(image.shape[:2])
    # image = cv2.resize(image, dsize=None, fx=scale, fy=scale)

    
    raw_image = image.astype(np.uint8)

    # Subtract mean values
    image = image.astype(np.float32)
    image -= np.array(
        [
            float(CONFIG.IMAGE.MEAN.B),
            float(CONFIG.IMAGE.MEAN.G),
            float(CONFIG.IMAGE.MEAN.R),
        ]
    )
    
    # Convert to torch.Tensor and add "batch" axis
    image = torch.from_numpy(image.transpose(2, 0, 1)).float().unsqueeze(0)
    image = image.to(device)
    return image, raw_image


def inference(model, image, raw_image=None, postprocessor=None):
    _, _, H, W = image.shape

    # Image -> Probability map
    # print(image.shape)
    logits = model(image)
    # print(logits.shape)
    logits = F.interpolate(logits, size=(H, W), mode="bilinear", align_corners=False)
    probs = F.softmax(logits, dim=1)[0]
    probs = probs.cpu().numpy()
    
    # np.save("probmap.npy",probs)
    # Refine the prob map with CRF
    if postprocessor and raw_image is not None:
        probs = postprocessor(raw_image, probs)   # crf.py

    labelmap = np.argmax(probs, axis=0)

    return probs, labelmap

class SegModel():
    def __init__(self):
        config_path = "configs/cocostuff164k.yaml"
        model_path = "data/models/deeplabv2_resnet101_msc-cocostuff164k-100000.pth"
        cuda = True
        crf = True  

        print(os.getcwd())
        self.CONFIG = OmegaConf.load(config_path) 
        self.device = get_device(cuda)
        torch.set_grad_enabled(False)
        classes = get_classtable(self.CONFIG)
        self.postprocessor = setup_postprocessor(self.CONFIG) if crf else None
        
        self.model = eval(self.CONFIG.MODEL.NAME)(n_classes=self.CONFIG.DATASET.N_CLASSES)
        state_dict = torch.load(model_path, map_location=lambda storage, loc: storage) 
                
        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.model.to(self.device)
        print("Model:", self.CONFIG.MODEL.NAME)

    def runModel(self, image):
        image, raw_image = preprocessing(image, self.device, self.CONFIG)  
        probs, labelmap = inference(self.model, image, raw_image, self.postprocessor)
        return probs

def init():
    global seg_model 
    seg_model =  SegModel()

def runSegModel(image):
    return seg_model.runModel(image)

if __name__ == "__main__":
    image_path = "test"
    init()
    if os.path.isdir(image_path):
        for filename in os.listdir(image_path):
            filename = image_path+filename if str(image_path).endswith('/') else image_path+'/'+filename
            image = cv2.imread(filename, cv2.IMREAD_COLOR)
            probmap = seg_model.runModel(image)
            print(probmap)
