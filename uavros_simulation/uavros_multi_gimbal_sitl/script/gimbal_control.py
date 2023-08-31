import time
import numpy as np
import rospy
from amov_gimbal_sdk_ros.msg import GimbalControl, GimbalState
from geometry_msgs.msg import Twist,Vector3

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None, activate_thresh=3):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_interval = 0
        self.current_time = current_time if current_time is not None else rospy.get_time()
        self.last_time = self.current_time

        """Clears PID computations and coefficients"""
        self.targetPoint = 0

        self.PTerm = 0
        self.ITerm = 0
        self.DTerm = 0
        self.last_error = 0

        # Windup Guard
        # 积分误差上限
        self.windup_guard = 20

        self.output = 0
        self.activate_thresh = activate_thresh

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.targetPoint - feedback_value
        if error<self.activate_thresh:
            return 0

        self.current_time = current_time if current_time is not None else rospy.get_time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_interval:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + self.Ki * self.ITerm + self.Kd * self.DTerm
        return self.output

    def set_target(self, target_value):
        self.targetPoint = target_value
        self.last_time=rospy.get_time()

    def set_kp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def set_ki(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def set_kd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def set_windup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def set_sample_interval(self, sample_interval):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_interval = sample_interval


class Motor:
    def __init__(self, angle=0, angular_velocity=0,max_rate=20):
        # 角速度控制在[-pi,pi]
        # 角度范围[-pi,pi]
        self.angle = angle
        self.angular_velocity = angular_velocity
        self.max_rate=max_rate
    def update(self):
        if abs(self.angular_velocity) >= self.max_rate:
            sign=1 if self.angular_velocity>=0 else -1
            self.angular_velocity = self.max_rate*sign


# One Dimension Control
class Ctrl_single:
    def __init__(self,P=0.2,I=0.0,D=0.0,rate_max=20,angle_thresh=[-30,90],length=10,activate_thresh=3) -> None:
        
        # initialization
        self.target=None
        self.window=None
        self.pid_ctrl=PID(P,I,D,activate_thresh=activate_thresh)
        self.state_cur=None
        self.rate_max=rate_max
        self.angle_thresh = angle_thresh
        self.length = length
        
    def set_target(self,target:float):
        # 限制旋转角度        
        if self.angle_thresh[0]<=target<=self.angle_thresh[1]:
            cur_target=target
        elif target < self.angle_thresh[0]:
            cur_target = self.angle_thresh[0]
        elif target>self.angle_thresh[1]:
            cur_target = self.angle_thresh[1]
        
        # 窗口滤波
        if self.window is None:
            self.window=[cur_target]
        elif len(self.window) < self.length:
            self.window=self.window+[cur_target]
        else:
            self.window.pop(0)
            self.window.append(cur_target)
        
        self.target=sum(self.window)/len(self.window)
        # 控制器目标更新
        self.pid_ctrl.set_target(self.target)
    
    def set_params(self,P,I,D):
        self.pid_ctrl.set_kp(P)
        self.pid_ctrl.set_ki(I)
        self.pid_ctrl.set_kd(D)
    
    def update(self,state):
        self.state_cur=state
        if self.target is not None:
            output=self.pid_ctrl.update(self.state_cur)
            motor=Motor(self.target,output,self.rate_max)
            motor.update()
            return motor
        else:
            return Motor()

class G1_ctrl:
    def __init__(self) -> None:

        """ros"""
        node_name = 'gimbal_control'
        rospy.init_node(node_name)
        rospy.loginfo("g1_manager_node is initialized!")
        
        """filter params"""
        self.length=10
        
        """control params"""
        P = rospy.get_param("~K_p",0.5)
        I = rospy.get_param("~K_i",0)
        D = rospy.get_param("~K_d",0)
        rate_max=rospy.get_param("~max_rate",90)
        model_name=rospy.get_param("~model_name","uav0")
        rospy.loginfo("Get Control Parameters: K_P: %f  K_I: %f  K_D: %f  Max_Rate: %f"%(P,I,D,rate_max))
        self.camera_state = None
        self.ctrl_mode = 1  # 1:angle rate; 2:angle
        self.pitch_ctrl = Ctrl_single(P = P,I = I,D = D,rate_max = rate_max,angle_thresh=[-30,90],length=self.length)
        self.roll_ctrl = Ctrl_single(P = P,I = I,D = D,rate_max = rate_max,angle_thresh=[-25,25],length=self.length)
        self.yaw_ctrl = Ctrl_single(P = P,I = I,D = D,rate_max = rate_max,angle_thresh=[-40,40],length=self.length)
        self.dt = 0.1
        self.time_last=rospy.get_time()
        # pubers
        self.state_puber = rospy.Publisher(
            "/{}/amov_gimbal_ros/gimbal_control".format(model_name), GimbalControl, queue_size=10)    
        self.start()
        # subers
        self.order_suber=rospy.Subscriber("vision/g1_ctrl",Twist,queue_size=10,callback=self.call_back_order)

        self.state_suber = rospy.Subscriber(
            "/{}/amov_gimbal_ros/gimbal_state".format(model_name), GimbalState, queue_size=10, callback=self.call_back_state)
        self.PID_suber=rospy.Subscriber("/uavros_simulation/uavros_multi_gimbal_sitl/pid_params",Vector3,queue_size=10,callback=self.call_back_pid)
        # self.timer=rospy.Timer(rospy.Duration(self.dt),self.order_pub)
        
        
    def timer_F(self):
        self.timer=rospy.Timer(rospy.Duration(self.dt),self.order_pub)

    def start(self):
        for i in range(5):
            order = GimbalControl()
            order.mode = 2
            order.roll_angle = 0.0
            order.pitch_angle = 90.0
            order.yaw_angle = 0.0
            self.state_puber.publish(order)
            info = 'change the camera state to: pitch %.2f, roll %.2f, yaw %.2f' % (
                order.pitch_angle, order.roll_angle, order.yaw_angle)
            rospy.loginfo(info)
            rospy.sleep(0.5)

    def call_back_pid(self,params:Vector3):
        P=params.x
        I=params.y
        D=params.z
        self.roll_ctrl.set_params(P,I,D)
        self.pitch_ctrl.set_params(P,I,D)
        self.yaw_ctrl.set_params(P,I,D)
    
    def call_back_order(self,order:Twist):
        roll=order.angular.x
        pitch=order.angular.y
        yaw=order.angular.z
        assert roll is not None
        assert pitch is not None
        assert yaw is not None
        roll += self.camera_state.imu_angle[0]
        pitch += self.camera_state.imu_angle[1]
        yaw += self.camera_state.rotor_angle[2]
        
        self.pitch_ctrl.set_target(pitch)
        self.roll_ctrl.set_target(roll)
        self.yaw_ctrl.set_target(yaw)

    def order_pub(self,event):
        rospy.loginfo(str(self.pitch_ctrl.target))
        time_cur=rospy.get_time()
        time_delta=time_cur-self.time_last
        self.time_last=time_cur
        if self.pitch_ctrl.target is not None:
            # order_roll=self.roll_ctrl.update(state.rotor_angle[0])
            order_pitch=self.pitch_ctrl.update(self.camera_state.imu_angle[1])
            order_yaw=self.yaw_ctrl.update(self.camera_state.rotor_angle[2])
            
            
            ctrl_mode = 2
            order = GimbalControl()
            order.mode = ctrl_mode
            # order.roll_angle = order_roll.angle
            # order.roll_rate=order_roll.angular_velocity
            # order.roll_angle=0
            # order.roll_rate=0
            order.pitch_angle = self.camera_state.imu_angle[1] + order_pitch.angular_velocity*time_delta
            # order.pitch_rate=order_pitch.angular_velocity
            order.yaw_angle = self.camera_state.rotor_angle[2] + order_yaw.angular_velocity*time_delta
            # order.yaw_rate=order_yaw.angular_velocity
            self.state_puber.publish(order)
            info = 'receive camera state order: roll %.2f, pitch %.2f, yaw %.2f' %(self.roll_ctrl.target,self.pitch_ctrl.target,self.yaw_ctrl.target)
            rospy.loginfo(info)
           
            info = 'change camera rate to:      roll %.2f, pitch %.2f, yaw %.2f' % (
                0, order.pitch_angle, order.yaw_angle)
            rospy.loginfo(info)   
    
    def call_back_state(self,state:GimbalState):
        self.camera_state=state
        info = 'current camera state rotor: roll %.2f, pitch %.2f, yaw %.2f' %(state.rotor_angle[0],state.rotor_angle[1],state.rotor_angle[2])
        rospy.loginfo(info)
        info = 'current camera state imu  : roll %.2f, pitch %.2f, yaw %.2f' %(state.imu_angle[0],state.imu_angle[1],state.imu_angle[2])
        rospy.loginfo(info)
        
        # else:
        #     info="No Target Yet"
        #     rospy.loginfo(info)


if __name__ =="__main__":
    G1=G1_ctrl()
    G1.timer_F()
    rospy.spin()