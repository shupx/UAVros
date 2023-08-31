import numpy as np
import math
import cv2
from tf.transformations import quaternion_matrix
#相机内参
camera = [333.838991, 0, 640, 0, 333.838991, 360, 0, 0, 1]

#畸变系数
camera_distCoeffs = np.float32([0, 0, 0, 0])


K = np.zeros((3, 3))
K[0, 0] = camera[0]
K[1, 1] = camera[4]
K[0, 2] = camera[2]
K[1, 2] = camera[5]
K[2, 2] = camera[8]

FOV_V = 69
FOV_H = 125

def undistored(img):
    img_undistored = cv2.undistort(img, K, camera_distCoeffs) #使用opencv去除图像畸变
    # img_vis = cv2.resize(img, (0,0), fx=0.5, fy=0.5)   #将原始图像缩放0.5倍后可视化
    # img_undistored_vis = cv2.resize(img_undistored, (0,0), fx=0.5, fy=0.5)#将去畸变图像缩放0.5倍后可视化
    # cv2.imshow('raw', img_vis)
    # cv2.imshow('undistored', img_undistored_vis)
    # cv2.waitKey()
    return img_undistored

def picture2world(pos_picture, plane_data, camera_theta, camera_phi, camera_psi, zw):# 俯仰theta，滚转phi，偏航psi(输入)
      pos_picture.append([1])
      pos_camera_divz = np.matmul(np.linalg.inv(K), pos_picture)
      R22 = np.array([[0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]])

      pos_world_LDF= np.matmul(R22, pos_camera_divz)
      R11 = np.array([[math.cos(camera_psi/180*math.pi), math.sin(camera_psi/180*math.pi), 0],
            [-math.sin(camera_psi/180*math.pi), math.cos(camera_psi/180*math.pi), 0],
            [0, 0, 1]])


      R21 = np.array([[1, 0, 0],
            [0, math.cos(camera_phi/180*math.pi), math.sin(camera_phi/180*math.pi)],
            [0, -math.sin(camera_phi/180*math.pi), math.cos(camera_phi/180*math.pi)]])
      
      R31 = np.array([[math.cos(camera_theta/180*math.pi), 0, math.sin(camera_theta/180*math.pi)],
      [0, 1, 0],
      [-math.sin(camera_theta/180*math.pi), 0, math.cos(camera_theta/180*math.pi)]])
      Le2b_1 = np.matmul(np.matmul(R11,R31),R21)

      pos_world_FLU = np.matmul(Le2b_1,pos_world_LDF)
      plan_quaternion  = [plane_data.pose.orientation.x,plane_data.pose.orientation.y,plane_data.pose.orientation.z,plane_data.pose.orientation.w]
      plan_rotation_matrix = quaternion_matrix(plan_quaternion)
      # print(plan_rotation_matrix)
      camera_position_FLU2world = np.matmul(plan_rotation_matrix[:3,:3], pos_world_FLU)
      z = -(plane_data.pose.position.z-zw-0.05)/camera_position_FLU2world[2]
      delta_y = z*camera_position_FLU2world[1]
      delta_x = z*camera_position_FLU2world[0]
      # print(delta_y,delta_x)
      target_world = np.array([delta_x[0]+plane_data.pose.position.x,delta_y[0]+plane_data.pose.position.y,0])
      # print(plane_data.pose.position.x,plane_data.pose.position.y)
      # print(target_world)
      return target_world
