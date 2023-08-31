#!/home/kxx/miniconda3/envs/wurenzhengfeng/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import onnxruntime

class YOLOV5():
    def __init__(self, onnxpath):
        self.onnx_session = onnxruntime.InferenceSession(onnxpath,providers=['CUDAExecutionProvider'])
        # print(self.onnx_session.get_providers())
        self.input_name = self.get_input_name()
        self.output_name = self.get_output_name()
    def get_input_name(self):
        input_name = []
        for node in self.onnx_session.get_inputs():
            input_name.append(node.name)
        return input_name

    def get_output_name(self):
        output_name = []
        for node in self.onnx_session.get_outputs():
            output_name.append(node.name)
        return output_name

    def get_input_feed(self, img_tensor):
        input_feed = {}
        for name in self.input_name:
            input_feed[name] = img_tensor
        return input_feed

    def nms(self,dets, thresh):
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        # -------------------------------------------------------
        #   计算框的面积
        #	置信度从大到小排序
        # -------------------------------------------------------
        areas = (y2 - y1 + 1) * (x2 - x1 + 1)
        scores = dets[:, 4]
        keep = []
        index = scores.argsort()[::-1]

        while index.size > 0:
            i = index[0]
            keep.append(i)
            # -------------------------------------------------------
            #   计算相交面积
            #	1.相交
            #	2.不相交
            # -------------------------------------------------------
            x11 = np.maximum(x1[i], x1[index[1:]])
            y11 = np.maximum(y1[i], y1[index[1:]])
            x22 = np.minimum(x2[i], x2[index[1:]])
            y22 = np.minimum(y2[i], y2[index[1:]])

            w = np.maximum(0, x22 - x11 + 1)
            h = np.maximum(0, y22 - y11 + 1)

            overlaps = w * h
            # -------------------------------------------------------
            #   计算该框与其它框的IOU，去除掉重复的框，即IOU值大的框
            #	IOU小于thresh的框保留下来
            # -------------------------------------------------------
            ious = overlaps / (areas[i] + areas[index[1:]] - overlaps)
            idx = np.where(ious <= thresh)[0]
            index = index[idx + 1]
        return keep

    def xywh2xyxy(self, x):
        # [x, y, w, h] to [x1, y1, x2, y2]
        y = np.copy(x)
        y[:, 0] = (x[:, 0] - x[:, 2] / 2)
        y[:, 1] = (x[:, 1] - x[:, 3] / 2)
        y[:, 2] = (x[:, 0] + x[:, 2] / 2)
        y[:, 3] = (x[:, 1] + x[:, 3] / 2)
        return y

    def filter_box(self, org_box, conf_thres, iou_thres):  # 过滤掉无用的框
        # -------------------------------------------------------
        #   删除为1的维度
        #	删除置信度小于conf_thres的BOX
        # -------------------------------------------------------
        org_box = np.squeeze(org_box)
        conf = org_box[..., 4] > conf_thres
        box = org_box[conf == True]
        # -------------------------------------------------------
        #	通过argmax获取置信度最大的类别
        # -------------------------------------------------------
        cls_cinf = box[..., 5:]
        cls = []
        for i in range(len(cls_cinf)):
            cls.append(int(np.argmax(cls_cinf[i])))
        all_cls = list(set(cls))
        output = []
        for i in range(len(all_cls)):
            curr_cls = all_cls[i]
            curr_cls_box = []
            curr_out_box = []
            for j in range(len(cls)):
                if cls[j] == curr_cls:
                    box[j][5] = curr_cls
                    curr_cls_box.append(box[j][:6])
            curr_cls_box = np.array(curr_cls_box)
            # curr_cls_box_old = np.copy(curr_cls_box)
            curr_cls_box = self.xywh2xyxy(curr_cls_box)
            curr_out_box = self.nms(curr_cls_box, iou_thres)
            for k in curr_out_box:
                output.append(curr_cls_box[k])
        output = np.array(output)
        return output

    def inference_img(self, img,width,height,conf_thres, iou_thres):
        or_img = cv2.resize(img, (width, height))   #resize后的原图
        img = or_img[:, :, ::-1].transpose(2, 0, 1)  # 将opencv格式的图片转换为rgb格式的图片
        img = img.astype(dtype=np.float32)
        img /= 255.0
        img = np.expand_dims(img, axis=0)
        input_feed = self.get_input_feed(img)
        pred = self.onnx_session.run(None, input_feed)[0]
        outbox_jc = self.filter_box(pred, conf_thres, iou_thres)
        return outbox_jc, or_img


