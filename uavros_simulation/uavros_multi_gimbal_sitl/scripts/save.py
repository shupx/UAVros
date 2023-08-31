#!/usr/bin/env python
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
cam0_path  = '/home/qwy/Pictures/cam0/'    # 已经建立好的存储cam0 文件的目录
cam1_path  = '/home/qwy/Pictures/cam1/'
 
def callback_cam0(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.resize(cv_img,(640,640))
        timestr = "%.6f" %  data.header.stamp.to_sec()
              #%.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = "cam0"+timestr+ ".jpg" #图像命名：时间戳.jpg
        cv2.imwrite(cam0_path + image_name, img)  #保存；
        # cv2.imshow("frame" , cv_img)
        # cv2.waitKey(3)
    else:
        pass
    
def callback_cam1(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.resize(cv_img,(640,640))
        timestr = "%.6f" %  data.header.stamp.to_sec()
              #%.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = "cam1"+timestr+ ".jpg" #图像命名：时间戳.jpg
        cv2.imwrite(cam1_path + image_name, img)  #保存；
        # cv2.imshow("frame" , cv_img)
        # cv2.waitKey(3)
    else:
        pass
 
def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/uav0/amov_gimbal_ros/gimbal_image', Image, callback_cam0)
    rospy.Subscriber('/uav1/amov_gimbal_ros/gimbal_image', Image, callback_cam1)
    rospy.spin()
 
if __name__ == '__main__':
    displayWebcam()
 