#!/usr/bin/env python

'''
This script forwards the gazebo link states of gimbals to ROS topic since the mavros topic
"mavros/mount_control/orientation" has no outputs in SITL.
Peixuan Shu
'''

import rospy

from mavros_msgs.msg import MountControl

from amov_gimbal_sdk_ros.msg import GimbalControl
from amov_gimbal_sdk_ros.msg import GimbalState

from mavros_msgs.srv import MountConfigure
from gazebo_msgs.srv import GetLinkState
import std_msgs.msg

from tf.transformations import euler_from_quaternion


class GimbalForward:

    def __init__(self):
        rospy.init_node('gimbal_topic_forward', anonymous=True)
        self.rate=rospy.Rate(30)

        self.vehicle_type = rospy.get_param('~modelname','solo_gimbal')
        self.vehicle_id = rospy.get_param('~model_id',0)
        print('Gimbal Topic Forward for model: ' + self.vehicle_type + '_' + str(self.vehicle_id))

        # forward "amov_gimbal_ros/gimbal_control" to "mavros/mount_control/command"
        rospy.Subscriber('amov_gimbal_ros/gimbal_control', GimbalControl, self.gimbal_control_cb)
        self.mount_publisher = rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=1)

        # forward amov gazebo gimbal states to "amov_gimbal_ros/gimbal_state"
        self.gazebo_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.gimbal_state_pub = rospy.Publisher('amov_gimbal_ros/gimbal_state', GimbalState, queue_size=1)

        # config mavros gimbal onece
        self.mountConfig = rospy.ServiceProxy('mavros/mount_control/configure', MountConfigure)
        srvheader=std_msgs.msg.Header()
        srvheader.stamp=rospy.Time.now()
        srvheader.frame_id="map"
        # self.mountConfig(header=srvheader,mode=2,stabilize_roll=False,stabilize_yaw=False,stabilize_pitch=False)

        self.cmd_mode = 0
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.cmd_roll_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.cmd_yaw_rate = 0.0

        self.cam_imu_roll = 0.0
        self.cam_imu_pitch = 0.0
        self.cam_imu_yaw = 0.0
        self.cam_rotor_roll = 0.0
        self.cam_rotor_pitch = 0.0
        self.cam_rotor_yaw = 0.0

        self.yaw_limit_low = -60
        self.yaw_limit_high = 60
    
    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                response = self.gazebo_link_state(self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_camera_link','ground_plane::link')
                qx = response.link_state.pose.orientation.x
                qy = response.link_state.pose.orientation.y
                qz = response.link_state.pose.orientation.z
                qw = response.link_state.pose.orientation.w
                euler = euler_from_quaternion([qx, qy, qz, qw])
                self.cam_imu_roll = euler[0] * 180 / 3.1415926 # deg (Forward)
                self.cam_imu_pitch = -euler[1] * 180 / 3.1415926 # deg (Right)
                self.cam_imu_yaw = -euler[2] * 180 / 3.1415926 # deg (Down)

                response = self.gazebo_link_state(self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_camera_link', self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_horizontal_arm_link')   
                qx = response.link_state.pose.orientation.x
                qy = response.link_state.pose.orientation.y
                qz = response.link_state.pose.orientation.z
                qw = response.link_state.pose.orientation.w
                euler = euler_from_quaternion([qx, qy, qz, qw])
                # self.cam_rotor_roll = -euler[0] * 180 / 3.1415926 # deg (Back)
                self.cam_rotor_pitch = -euler[1] * 180 / 3.1415926 # deg (Right)
                # self.cam_rotor_yaw = -euler[2] * 180 / 3.1415926 # deg (Down)     

                response = self.gazebo_link_state(self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_horizontal_arm_link', self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_vertical_arm_link')   
                qx = response.link_state.pose.orientation.x
                qy = response.link_state.pose.orientation.y
                qz = response.link_state.pose.orientation.z
                qw = response.link_state.pose.orientation.w
                euler = euler_from_quaternion([qx, qy, qz, qw])
                self.cam_rotor_roll = -euler[0] * 180 / 3.1415926 # deg (Back)
                # self.cam_rotor_pitch = -euler[1] * 180 / 3.1415926 # deg (Right)
                # self.cam_rotor_yaw = -euler[2] * 180 / 3.1415926 # deg (Down)     

                response = self.gazebo_link_state(self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_vertical_arm_link', self.vehicle_type+'_'+str(self.vehicle_id)+'::cgo3_mount_link')   
                qx = response.link_state.pose.orientation.x
                qy = response.link_state.pose.orientation.y
                qz = response.link_state.pose.orientation.z
                qw = response.link_state.pose.orientation.w
                euler = euler_from_quaternion([qx, qy, qz, qw])
                # self.cam_rotor_roll = -euler[0] * 180 / 3.1415926 # deg (Back)
                # self.cam_rotor_pitch = -euler[1] * 180 / 3.1415926 # deg (Right)
                self.cam_rotor_yaw = -euler[2] * 180 / 3.1415926 # deg (Down)                                          
            except:
                print("Gazebo model state service call failed for one loop")

            state_msg = GimbalState()
            state_msg.header.stamp = rospy.Time.now()
            state_msg.mode = self.cmd_mode
            state_msg.imu_angle[0] = self.cam_imu_roll
            state_msg.imu_angle[1] = self.cam_imu_pitch
            state_msg.imu_angle[2] = self.cam_imu_yaw
            state_msg.rotor_angle[0] = self.cam_rotor_roll
            state_msg.rotor_angle[1] = self.cam_rotor_pitch
            state_msg.rotor_angle[2] = self.cam_rotor_yaw
            self.gimbal_state_pub.publish(state_msg)
            self.rate.sleep()
        
    def gimbal_control_cb(self, msg):
        self.cmd_mode = msg.mode
        self.cmd_roll = msg.roll_angle # solo_gimbal.sdf model limits rotor roll at [-45,45]deg (Forward)
        self.cmd_pitch = msg.pitch_angle  # solo_gimbal.sdf model limits rotor pitch at [-120,40]deg (Right)
        self.cmd_yaw = msg.yaw_angle  # solo_gimbal.sdf model limits rotor yaw at [-inf, inf]deg (Down)

        # limit roll imu controll angle
        if self.cmd_roll > 45.0: 
            self.cmd_roll = 45.0
        if self.cmd_roll < -45.0:
            self.cmd_roll = -45.0
        # limit pitch imu controll angle
        if self.cmd_pitch > 30.0: 
            self.cmd_pitch = 30.0
        if self.cmd_pitch < -89.9:
            self.cmd_pitch = -89.9
        # limit yaw rotor control angle
        if self.cmd_yaw > 80: 
            self.cmd_yaw = 80
        if self.cmd_yaw < -80:
            self.cmd_yaw = -80
        
        self.cmd_roll_rate = msg.roll_rate
        self.cmd_roll_rate = msg.pitch_rate
        self.cmd_yaw_rate = msg.yaw_rate    
        if self.cmd_mode == 1 or self.cmd_roll_rate != 0. or self.cmd_roll_rate != 0. or self.cmd_yaw_rate != 0.:
            print("[error]gimbal rate control not supported yet")
        control_msg = MountControl()
        control_msg.mode = self.cmd_mode # 2 is angle control
        control_msg.pitch = self.cmd_pitch # deg
        control_msg.roll = self.cmd_roll # deg
        control_msg.yaw = self.cmd_yaw # deg
        self.mount_publisher.publish(control_msg)

if __name__ == "__main__":
    node = GimbalForward()
    node.main_loop()