#!/usr/bin/python3
# -*- coding: utf-8 -*-


'''
Created on 2023-07-13

@author: <qinwenyuan, quanheyi, zhouzhiyan, kongxiangxi>

1、从吊舱话题中读到图像/直接从串口读取图像
2、调用目标检测算法检测多辆车
3、将车抠出来检测数字,计算目标框的中心点坐标并转化到世界坐标系中,进行定位
4、将数字的属性添加到车上
5、将添加了数字属性车的边界框给吊舱控制
'''

import os
import math
import lap
import _thread
import sys
file_path = os.path.abspath(__file__)                  # 读取当前脚本文件路径   __file__： Python内置变量，当前脚本文件的路径
dir_path = os.path.dirname(file_path)                  # 读取当前脚本文件所在的上一级路径
sys.path.append(dir_path)                              # 将上级目录路径添加到系统路径sys.path中   Python解释器能够在运行脚本时查找并导入该目录下的模块
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from picture2world import picture2world  # 像素坐标转世界坐标
from Signle_Tracker import Signle_Tracker  # 单目标跟踪
from detect_onnx import YOLOV5 
from amov_gimbal_sdk_ros.msg import GimbalControl, GimbalState

from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float32MultiArray
# std_msgs：基本数据消息
# Int8：有符号的8位整数，取值范围为-128到127
# Float32MultiArray：多维的浮点数数组，包含一个data字段，可以用来存储多个浮点数值
from sensor_msgs.msg import Image
# sensor_msgs：传感器数据的消息类型
# Image：图像数据，包含图像的宽度、高度、像素编码格式以及像素数据等信息
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseArray, Quaternion,Point  # w >0:angle <0: angle_rate x:roll y:pitch z:yaw
# geometry_msgs：用于传递和交换几何相关的数据，描述机器人的位置、姿态、速度等几何信息
# Twist: 用于描述刚体在三维空间中的运动状态，包括线速度(linear.x,linear.y,linear.z)、角速度(angular.x,angular.y,angular.z)
# PoseArray：
# Pose：用于表示3D空间中的位姿信息，包括position（位置：geometry_msgs.msg.Point）和orientation（方向：geometry_msgs.msg.Quaternion）
# Point：
# Quaternion：表示四元数，四元数是一种用于表示旋转的数学工具，在机器人系统中常用于描述物体或机器人的姿态，q=w+xi+yj+zk，w是实部，(x,y,z)是虚部的三维向量
# PoseStamped：表示带时间戳的位姿信息，包含了位置和姿态信息，并附带了时间戳，用于标识该位姿信息的采集时间
from amov_gimbal_sdk_ros.msg import GimbalState

# amov_gimbal_sdk_ros：G1吊舱消息类型
# GimbalControl：控制模式：0:不动 1:角速度 2:角度和角速度 3:回到初始位置   横滚(Roll)，偏航(Yaw)，俯仰(Pitch)角 大小(angle)和角速度(rate)控制  
# GimbalState：控制模式：0：home 1：tracking 2：yaw follow  imu_angle：相机IMU的横滚、俯仰和偏航角度（每秒发布30次） rotor_angle：电机各关节的角度（3个电机每秒发布30次）
# 控制俯仰(Pitch)角和横滚(Roll)角是imu  控制偏航(Yaw)角是rotor(IMU在相机里)


'''global variable'''

JC_CLASSES = ['car','0','1','2','3']               # 检测类别索引
YOLO_DIR = os.path.join(dir_path,"models/best.onnx")  # 将当前脚本文件的上一级路径和BS.onnx进行拼接，形成完整的文件路径


class MainVision:
    def __init__(self, classes, dir, img_width=640, img_heigh=640, camera_img_width=1280, camera_img_heigh=720,
                 conf_thres=0.2):

        '''database'''
        self.images = None                              # 缓存吊舱接收到的图像
        self.car_number_dets = []                       # 储存处理好的车的检测框信息
        self.classes = classes                          # 检测类别
        self.dir = dir                                  # 权重文件路径
        self.flag = 1                                   # 任务阶段标志位
        self.camera_state = GimbalState()               # 吊舱状态信息
        self.prev_target_position_number = []           # 储存过去图片目标位置信息和数字信息
        self.GPS = PoseStamped()                        # 存储带时间戳的GPS信息
        self.img_width = img_width                      # resize后的宽度
        self.img_heigh = img_heigh                      # resize后的高度
        self.camera_img_width = camera_img_width        # 吊舱原始宽度
        self.camera_img_heigh = camera_img_heigh        # 吊舱原始高度
        self.conf_thres = 0.3                    # 置信度阈值
        self.sing_track_flag = False                    # 单目标跟踪标志位
        self.track_score = 0.6                          # 跟踪器置信度阈值
        self.f = 0.01543
        self.car_z = 0                                # 车高
        self.FOV_V = 70/2
        self.FOV_H = 125/2
        # self.model_name = rospy.get_param("~model_name","uav0")
        self.car_name = rospy.get_param("~car_name","ugv0")

        '''detector'''
        # self.yolo = YOLOV5_TRT(self.dir, self.classes)  # 检测器
        # _ = self.yolo.infer(cv2.resize(cv2.imread(os.path.join(dir_path,"demp.jpeg")), (self.img_width, self.img_heigh)))
        self.yolo = YOLOV5(self.dir)
        _, _ = self.yolo.inference_img(cv2.imread(os.path.join(dir_path,"demp.jpeg")),self.img_width,self.img_heigh,0.1,0.1)

        self.sing_tracker = Signle_Tracker()            # 跟踪器

        '''tools'''
        self.bridge = CvBridge()                        # OpenCV图像转换ROS图像
        self.plane_position_gazebo = Odometry()
        self.car_position_gazebo = Odometry()
        self.pre_pixel = [None,None]
        self.car_position = Odometry()
        '''ros'''
        # node
        rospy.init_node('vision')
        # rospy.loginfo("vision node is initialized!")

        # subscriber
        rospy.Subscriber("task_stage", Int8, callback=self.call_back_flight_stage)  # 订阅规划flag
        # rospy.Subscriber("amov_gimbal_ros/gimbal_image", Image, queue_size=1,
        #                  callback=self.call_back_gimbal_image)  # 订阅吊舱图像
        # rospy.Subscriber("amov_gimbal_ros/gimbal_state", GimbalState, queue_size=1,
        #                  callback=self.call_back_state)  # 订阅吊舱状态
        # rospy.Subscriber("mavros/local_position/pose", PoseStamped, queue_size=1,
        #                  callback=self.gps_call_back)  # 订阅mavros的数据  gps：x，y 激光定高：z
        rospy.Subscriber("modelpos_gazebo/", Odometry, queue_size=1,
                         callback=self.plane_position_call_back)  
        rospy.Subscriber("amov_gimbal_ros/gimbal_image", Image, queue_size=1,
                         callback=self.call_back_gimbal_image)  # 订阅吊舱图像
        rospy.Subscriber("amov_gimbal_ros/gimbal_state", GimbalState, queue_size=1,
                         callback=self.call_back_state)  # 订阅吊舱状态
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, queue_size=1,
                         callback=self.gps_call_back)  # 订阅mavros的数据  gps：x，y 激光定高：z
        # rospy.Subscriber("/{}/modelpos_gazebo/".format(self.model_name), Odometry, queue_size=1,
        #                  callback=self.plane_position_call_back)  
        # rospy.Subscriber("/{}/modelpos_gazebo/".format(self.car_name), Odometry, queue_size=1,
        #                  callback=self.car_position_call_back)

        # publisher
        # self.state_puber = rospy.Publisher("vision/g1_ctrl", Twist, queue_size=10)  # 发布吊舱控制信息
        self.target_position_puber = rospy.Publisher("vision/target_position", Pose, queue_size=10)  # 发布目标位置信息
        self.target_image_puber = rospy.Publisher("vision/target_image", Image, queue_size=10)  # 发布目标图像信息（地面站/调试可视化）
        self.car_position_puber = rospy.Publisher("vision/car_position", Odometry, queue_size=10)  # 发布目标图像信息（地面站/调试可视化）
        self.state_puber = rospy.Publisher(
            "amov_gimbal_ros/gimbal_control", GimbalControl, queue_size=10) 
        '''thread'''
        self.start()
        # _thread.start_new_thread(self.object_detection_block, ())  # 任务函数线程
        # _thread.start_new_thread(self.postion_caculate, ())  # 任务函数线程
    def start(self):
        for i in range(5):
            order = GimbalControl()
            order.mode = 2
            order.roll_angle = 0.0
            order.pitch_angle = 60.0
            order.yaw_angle = 0.0
            self.state_puber.publish(order)
            info = 'change the camera state to: pitch %.2f, roll %.2f, yaw %.2f' % (
                order.pitch_angle, order.roll_angle, order.yaw_angle)
            rospy.loginfo(info)
            rospy.sleep(0.5)

    def plane_position_call_back(self,data):
        self.plane_position_gazebo.pose.pose.position.x = data.pose.pose.position.x
        self.plane_position_gazebo.pose.pose.position.y = data.pose.pose.position.y
        self.plane_position_gazebo.pose.pose.position.z = data.pose.pose.position.z


    def car_position_call_back(self,data):
        self.car_position_gazebo.pose.pose.position.x = data.pose.pose.position.x
        self.car_position_gazebo.pose.pose.position.y = data.pose.pose.position.y
        self.car_position_gazebo.pose.pose.position.z = data.pose.pose.position.z

    def postion_caculate(self):
        xx = self.car_position_gazebo.pose.pose.position.x-self.plane_position_gazebo.pose.pose.position.x+self.GPS.pose.position.x
        yy = self.car_position_gazebo.pose.pose.position.y-self.plane_position_gazebo.pose.pose.position.y+self.GPS.pose.position.y
        zz = self.car_position_gazebo.pose.pose.position.z-self.plane_position_gazebo.pose.pose.position.z+self.GPS.pose.position.z
        # self.car_position_puber.publish(car_position)


    def linear_assignment(self, cost_matrix, thresh):
        '''
        LAPJV线性匹配i
        output:
        matches: 匹配对的索引
        unmatched_a: 未匹配的A元素的索引
        unmatched_b: 未匹配的B元素的索引
        '''
        if cost_matrix.size == 0:
            return (
                np.empty((0, 2), dtype=int),
                tuple(range(cost_matrix.shape[0])),
                tuple(range(cost_matrix.shape[1])),
            )
        matches, unmatched_a, unmatched_b = [], [], []
        cost, x, y = lap.lapjv(cost_matrix, extend_cost=True, cost_limit=thresh)
        # x：A匹配对的索引list，未匹配的元素位置为-1
        # y：B匹配对的索引list，未匹配的元素位置为-1
        # cost：匹配过程的最小总代价
        for ix, mx in enumerate(x):
            if mx >= 0:
                matches.append([ix, mx])
        unmatched_a = np.where(x < 0)[0]
        unmatched_b = np.where(y < 0)[0]
        matches = np.asarray(matches)
        return matches, unmatched_a, unmatched_b

    def distances(self, position1, position2):
        '''
        计算每一个当前目标位置到每一个预测目标位置的距离矩阵

        input:
        position1: 当前目标的坐标点集合
        position2: 预测目标的坐标点集合

        output:
        distances：当前目标i和预测目标j组成的距离矩阵
        '''
        l = len(position1)  # 当前识别的目标个数
        m = len(position2)  # 预测的目标个数
        distances = np.zeros((l, m))
        for i in range(l):
            for j in range(m):
                distances[i][j] = self.distance(position1[i], position2[j])  # 当前目标i和预测目标j之间的距离矩阵
        return distances

    def distance(self, position1, position2):
        '''
        计算两坐标点之间的距离

        input:
        position1: 第一个点的坐标(x,y,z)
        position2: 第二个点的坐标(x,y,z)

        output:
        dis: 两个坐标点之间的距离
        '''
        dis = math.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2 + (
                    position1[2] - position2[2]) ** 2)  # 两点之间的距离
        return dis


    def pixel2angle(self, center):
        '''
        中心点像素坐标转原图像素坐标和吊舱旋转角度

        input:
        center: 中心点坐标（归1化后的）

        output:
        x_pixel: 在原图中心点x像素坐标
        y_pixel: 在原图中心点y像素坐标
        yaw: 吊舱控制的偏航角
        pitch: 吊舱控制的俯仰角
        roll: 吊舱控制的横滚角
        '''
        x = center[0]
        y = center[1]
        x_pixel = (x - 0.5) * self.camera_img_width  # 放大到原图中，吊舱像素：1280*720
        y_pixel = (y - 0.5) * self.camera_img_heigh
        yaw_max=math.radians(self.FOV_H)
        yaw = math.atan(2*(x - 0.5) * math.tan(yaw_max))/math.pi*180  # 偏航角
        pitch_max = math.radians(self.FOV_V)
        pitch = math.atan(2*(y - 0.5) * math.tan(pitch_max))/math.pi*180  # 俯仰角
        roll = 0  # 横滚角
        return x_pixel, y_pixel, yaw, pitch, roll

    # def control_gimbal(self, roll, pitch, yaw):
    #     '''
    #     控制吊舱转到对应角度
        
    #     input:
    #     roll: 横滚角控制信号
    #     pitch: 俯仰角控制信号
    #     yaw: 俯仰角控制信号

    #     output:
    #     无
    #     '''
    #     # rospy.loginfo("Heading to person")
    #     order = Twist()
    #     order.angular.x = roll
    #     order.angular.y = pitch
    #     order.angular.z = yaw
    #     self.state_puber.publish(order)
    def control_gimbal(self, roll, pitch, yaw):
        '''
        控制吊舱转到对应角度
        
        input:
        roll: 横滚角控制信号
        pitch: 俯仰角控制信号
        yaw: 俯仰角控制信号

        output:
        无
        '''
        # rospy.loginfo("Heading to person")
        ctrl_mode = 2
        order = GimbalControl()
        order.mode = ctrl_mode
        # order.roll_angle = order_roll.angle
        # order.roll_rate=order_roll.angular_velocity
        # order.roll_angle=0
        # order.roll_rate=0
        order.pitch_angle = self.camera_state.imu_angle[1] + pitch
        # order.pitch_rate=order_pitch.angular_velocity
        order.yaw_angle = self.camera_state.rotor_angle[2] + yaw
        order.roll_angle = 0

        # order.yaw_rate=order_yaw.angular_velocity
        self.state_puber.publish(order)


    def if_move(self, target_position_number, prev_target_position_number):
        '''
        确定目标是否移动，将移动的目标位置发布

        input:
        target_position: 所有当前帧检测到的目标位置数字[[x1,y1,z1,number],[x2,y2,z2,number]...[xn,yn,zn,number]]
        prev_target_position: 所有上一帧检测到的目标位置数字[[x1,y1,z1,number],[x2,y2,z2,number]...[xn,yn,zn,number]]
        
        output:
        None
        '''

        xyz_positions = [xyz_list[:3] for xyz_list in target_position_number]  # 取xyz
        xyz_prev_target_positions = [xyz_list[:3] for xyz_list in prev_target_position_number]
        position1 = []  # 所有目标当前位置
        position2 = []  # 所有目标过去位置
        new_position = []  # 运动目标当前位置
        for xyz_position in xyz_positions:
            position1.append(xyz_position)
        for xyz_prev_target_position in xyz_prev_target_positions:
            position2.append(xyz_prev_target_position)
        distances = self.distances(position1, position2)  # 计算每一个当前目标位置到每一个预测目标位置的距离矩阵（作为匹配代价矩阵）
        label, _, _ = self.linear_assignment(distances, 1)  # 以当前坐标和预测坐标的距离作为匹配代价进行位置的线性匹配
        if len(label) == 0:  # 没有静止目标
            position_target = Point()
            position_target.x = 1
            position_target.y = len(label)
            position_target.z = 0
            position_number = Quaternion()
            position_number.x = target_position_number[0][0]
            position_number.y = target_position_number[0][1]
            position_number.z = target_position_number[0][2]
            position_number.w = target_position_number[0][3]
            target_pose = Pose(position = position_target, orientation = position_number)
            self.target_position_puber.publish(target_pose)  # 将所有当前帧检测到的目标位置数字发布
        else:  # 有静止目标
            filtered_indexes = [i for i in range(len(target_position_number)) if i not in [row[0] for row in label]]  # 筛选动目标索引
            for i in filtered_indexes:  # 筛选动目标位置数字
                new_position.append(target_position_number[i])
            if len(new_position)!=0:
                position_target = Point()
                position_target.x = 1
                position_target.y = 1
                position_target.z = 1
                position_number = Quaternion()
                position_number.x = new_position[0][0]
                position_number.y = new_position[0][1]
                position_number.z = new_position[0][2]
                position_number.w = new_position[0][3]
                target_pose = Pose(position = position_target, orientation = position_number)
                self.target_position_puber.publish(target_pose)  # 将当前帧检测到运动的目标位置数字发布
            else:
                position_target = Point()
                position_target.x = 1
                position_target.y = 1
                position_target.z = 0
                position_number = Quaternion()
                position_number.x = target_position_number[0][0]
                position_number.y = target_position_number[0][1]
                position_number.z = target_position_number[0][2]
                position_number.w = target_position_number[0][3]
                target_pose = Pose(position = position_target, orientation = position_number)
                self.target_position_puber.publish(target_pose)  # 发布（0,0,0,0）
                
    def static_object(self,target_position_number):
        position_target = Point()
        position_target.x = 1
        position_target.y = 1
        position_target.z = 1
        position_number = Quaternion()
        position_number.x = target_position_number[0][0]
        position_number.y = target_position_number[0][1]
        position_number.z = target_position_number[0][2]
        position_number.w = target_position_number[0][3]
        target_pose = Pose(position = position_target, orientation = position_number)
        # print(target_pose)
        self.target_position_puber.publish(target_pose)  # 将所有当前帧检测到的目标位置数字发布
   
    def car_detect(self, dets):
        '''
        车辆检测

        input:
        dets: 所有目标检测结果

        output:
        car_dets: 车辆类目标检测框的坐标
        '''
        car_dets = []
        if len(dets) > 0:
            for i in range(len(dets)):
                classes_index = int(dets[i][5])
                if classes_index == self.classes.index('car'):   # 类别为车
                # if dets[i].class_name == "car":  # 类别为车
                    car_dets.append(dets[i][:4].tolist())
                    # car_dets.append(dets[i].bbox)
        return car_dets

    def number_detect(self, dets):
        '''
        数字检测

        input:
        dets: 所有目标检测结果

        output:
        number/-1: 检测到的数字值，每检测到返回-1
        '''
        number = -1
        if len(dets) > 0:
            # if dets[0].class_name in ['0', '1', '2', '3']:  # 类别为数字
            classes_index = int(dets[0][5])
            if classes_index in [self.classes.index('0'), self.classes.index('1'), self.classes.index('2'), self.classes.index('3')]:   
                number = int(self.classes[classes_index])
                # number = int(dets[0].class_name)
        return number

    def crop_images(self, img, boxs):
        '''
        剪裁图像

        input:
        img: 要剪裁的一张图像
        boxs: 要剪裁区域的所有检测框坐标

        output:
        cropped_images: 剪裁后的所有图像
        '''
        cropped_images = []
        for box in boxs:
            # 边界检查，确保坐标不会超出图像范围
            x1, y1, x2, y2 = box[:4]
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1
            if x2 <= x1 or y2 <= y1:
                continue
            x1 = max(0, int(x1))
            y1 = max(0, int(y1))
            x2 = min(img.shape[1], int(x2))
            y2 = min(img.shape[0], int(y2))
            # 剪裁图像
            cropped_img = img[y1:y2, x1:x2]
            cropped_images.append(cropped_img)
            
        return cropped_images


    def draw_image(self, image, box_data):
        '''
        对检测框取整，在检测图像上画检测框,并将检测图像调整为原图大小并发布原图

        input:
        image: 要绘制的图像
        box_data: 检测框信息(坐标 类别 置信度)

        output:
        Noneprev_target_position_number
        '''
        left = int(box_data[0] / self.img_width * self.camera_img_width)
        top = int(box_data[1] / self.img_heigh * self.camera_img_heigh)
        right = int(box_data[2] / self.img_width * self.camera_img_width)
        bottom = int(box_data[3] / self.img_heigh * self.camera_img_heigh)
        cv2.rectangle(image, (left, top), (right, bottom), (255, 23, 13), 2)  # 画框
        img = cv2.resize(image, (self.camera_img_width, self.camera_img_heigh))  # 将图片调整为原图大小
        data = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")  # opencv转rosmsg
        self.target_image_puber.publish(data)  # 发布处理后的图像

    def max_car(self, car_dets):
        '''
        筛选图像中最大的车

        input:
        car_dets: 当前图片中车的所有检测框坐标

        output:
        car: 当前图片中最大车的检测框坐标
        index: 当前图片中最大车的检测框坐标索引

        '''
        min_aera = -1
        index = None
        car = None
        for i in range(len(car_dets)):
            if abs(car_dets[i][0] - car_dets[i][2]) * abs(car_dets[i][1] - car_dets[i][3]) > min_aera:
                min_aera = abs(car_dets[i][0] - car_dets[i][2]) * abs(car_dets[i][1] - car_dets[i][3])
                car = car_dets[i]
                index = i
            else:
                continue
        return index, car

    def resize_bbox(self, box_data):
        '''
        将检测框的像素坐标还原到原图的大小(640,640)->(1280,720)

        input:
        box_data: 当前图片中最大车的检测框坐标坐标信息

        output:
        [int(left_resize), 
        int(top_resize), 
        int(abs(right_resize-left_resize)), 
        int(abs(bottom_resize-top_resize))]: 还原后的检测框信息
        '''
        left, top, right, bottom = box_data[0], box_data[1], box_data[2], box_data[3]  # 提取检测框的左上右下角坐标
        left_resize = left / self.img_width * self.camera_img_width  # 将四个坐标值归一化
        top_resize = top / self.img_heigh * self.camera_img_heigh
        right_resize = right / self.img_width * self.camera_img_width
        bottom_resize = bottom / self.img_heigh * self.camera_img_heigh
        return [int(left_resize), int(top_resize), int(abs(right_resize - left_resize)),
                int(abs(bottom_resize - top_resize))]
    
    def get_center_coordinates(self, car_dets, width,height):
        center_coordinates_list = []
        for det in car_dets:
            x1, y1, x2, y2 = det
            center_x = 0.5 * (x1 + x2)/width
            center_y = 0.5 * (y1 + y2)/height
            center_coordinates_list.append([center_x, center_y])
        return center_coordinates_list

    def call_back_flight_stage(self, flight_stage):  # 订阅任务阶段
        self.flag = flight_stage.data
        # rospy.loginfo("task stage is %d!" % self.flag)

    def call_back_gimbal_image(self, gimbal):  # 订阅吊舱获取的图像
        # rospy.loginfo("receive gimbal images!")
        self.images = self.bridge.imgmsg_to_cv2(gimbal, "bgr8")  # rosmsg转opencv

    def call_back_state(self, state: GimbalState):  # 订阅吊舱状态信息
        self.camera_state = state

    def gps_call_back(self, data: PoseStamped):  # 订阅mavros位置信息x，y：gps  z：激光定高
        self.GPS = data
        # print("GPS",self.GPS)
        
    def main_loop(self):
        # rospy.loginfo("Ready to begin object detection block.")
        while not rospy.is_shutdown():
            or_image = self.images  # (1280,720,3)
            GPS = self.GPS
            # print(GPS)
            camera = self.camera_state
            if self.flag == 1 and or_image is not None:  # 收到flag并且吊舱接收到图像 执行任务1
                # rospy.loginfo("Task 1 start.")
                target_position_number = []  # 存储当前帧图片中目标的位置信息和数字信息
                if not self.sing_track_flag:  # 默认关闭跟踪模块
                    dets, img = self.yolo.inference_img(or_image,self.img_width,self.img_heigh,0.2,0.1)   # (640,640,3)
                    car_dets = self.car_detect(dets)  # 识别车辆  (640,640,3)  [[]]
                    # print(len(car_dets))                    
                    if len(car_dets) > 0:  # 识别到车辆
                        # rospy.loginfo("car is detected!")
                        track_car_index, track_car_det = self.max_car(car_dets)  # 找到图像中最大的车作为跟踪车辆（这里要改，接收消息，判断要跟踪哪一辆车）

                        center = self.get_center_coordinates(car_dets,self.img_width,self.img_heigh)  # (0~1)[[]]
                        # car_imgs = self.crop_images(img, car_dets)  # 剪裁 <640
                        # for i in range(len(car_imgs)):
                        #     car_img = cv2.resize(car_imgs[i], (self.img_width, self.img_heigh))  # (640,640,3)
                        #     number_dets, _ = self.yolo.inference_img(car_img,self.img_width,self.img_heigh,0.1,0.1)   # (640,640,3)
                        #     number = self.number_detect(number_dets)  # 识别数字
                        #     number = 1
                        #############################################测试代码#############################################
                        self.draw_image(or_image, track_car_det)  # 在图像上画检测框,调整为原图大小，发布检测图像（1280，720，3）
                        #################################################################################################
                        l,t,w,h  = self.resize_bbox(track_car_det)  # (1280,720)
                        # if(self.pre_pixel[0] is None):
                        #     self.pre_pixel = [l+0.5*w,t+0.5*h]
                        # else:
                        #     if(math.sqrt((l+0.5*w-self.pre_pixel[0])**2+(t+0.5*h-self.pre_pixel[1])**2)<50):
                        #         self.pre_pixel = [l+0.5*w,t+0.5*h]
                        center_position = picture2world([[l+0.5*w], [t+0.5*h]],  # 将第i辆车中心点的像素坐标转化为世界坐标(x,y,z)
                                                        GPS,
                                                        camera.rotor_angle[1],
                                                        camera.rotor_angle[0],
                                                        camera.rotor_angle[2],
                                                        self.car_z).tolist()
                            # print(center_position)
                        # print("GPS",self.GPS)
                                                            
                        center_position.append(1)  # 添加数字
                        target_position_number.append(center_position)  # 将该目标的位置和数字信息存储 [[],[]...[]]
                        self.static_object(target_position_number)
                        # if len(self.prev_target_position_number) == 0:
                        #     self.prev_target_position_number = target_position_number  # 将当前目标位置作为目标预测位置
                        # else:
                        #     self.if_move(target_position_number, self.prev_target_position_number)  # 筛选动目标，将动目标位置和数字发布、
                        roi = self.resize_bbox(track_car_det)  # 将最大检测框的坐标还原到原图坐标
                        self.sing_tracker.init_model(or_image, roi)  # 选定跟踪目标 (1280, 720, 3)
                        track_center = center[track_car_index]  # 最大检测框的中心点坐标(0~1)
                        _, _, yaw, pitch, roll = self.pixel2angle(track_center)  # 像素坐标转换为吊舱转旋转角度
                        self.control_gimbal(roll, pitch, yaw)  # 控制吊舱旋转
                        self.sing_track_flag = True  # 如果检测到和数字对应的车，在此决定是否开启跟踪模块(Ture/False)
                else:  # 跟踪模块
                    isLocated, bbox, score = self.sing_tracker.update(or_image)  # 跟踪选定的框，返回是否跟上，左上右下坐标信息，置信度
                    if isLocated and score >= self.track_score:
                        x, y, w, h = bbox  # (1280,720,3)
                        # if(math.sqrt((l+0.5*w-self.pre_pixel[0])**2+(t+0.5*h-self.pre_pixel[1])**2)<50):
                        #     self.pre_pixel = [x+0.5*w,y+0.5*h]

                        #############################################测试代码#############################################
                        cv2.rectangle(or_image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 在图上画跟踪框
                        center_position = picture2world([[x+0.5*w], [y+0.5*h]],  # 将第i辆车中心点的像素坐标转化为世界坐标(x,y,z)
                                                GPS,
                                                camera.rotor_angle[1],
                                                camera.rotor_angle[0],
                                                camera.rotor_angle[2],
                                                self.car_z).tolist()
                        center_position.append(1)  # 添加数字
                        target_position_number.append(center_position)  # 将该目标的位置和数字信息存储 [[],[]...[]]
                        # if len(self.prev_target_position_number) == 0:
                        #     self.prev_target_position_number = target_position_number  # 将当前目标位置作为目标预测位置
                        # else:
                        #     self.if_move(target_position_number, self.prev_target_position_number)  # 筛选动目标，将动目标位置和数字发布、
                        self.static_object(target_position_number)
                        data = self.bridge.cv2_to_imgmsg(or_image, encoding="bgr8")  # opencv转rosmsg
                        self.target_image_puber.publish(data)  # 发布画框后的图像
                        #################################################################################################

                        track_center = np.array([(x + 0.5 * w) / self.camera_img_width,
                                                    (y + 0.5 * h) / self.camera_img_heigh])  # 求跟踪框的中心点坐标，并归一化(0~1)
                        _, _, yaw, pitch, roll = self.pixel2angle(track_center)  # 像素坐标转换为吊舱转旋转角度
                        self.control_gimbal(roll, pitch, yaw)  # 控制吊舱旋转
                    else:
                        self.sing_track_flag = False
            elif self.flag == 2:  # 后续添加
                pass

            elif self.flag == 3:
                pass
            # else:
            #     rospy.loginfo("Waiting for the task to start!")


if __name__ == "__main__":
    main_vision = MainVision(JC_CLASSES, YOLO_DIR)
    main_vision.main_loop()
    rospy.loginfo("MainVision is running!")
    rospy.spin()  # 阻塞等待回调
