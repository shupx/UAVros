/**
 * @brief UAV controller class
 *
 * Flight controller to output velocity setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.07.10
 */

#include "uav_controller.h"

using namespace Eigen;
using namespace std;

uavCtrl::uavCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
  nh_private_.param<double>("loop_sec", loop_sec_, 0.05);
  nh_private_.param<double>("arrive_alt", arrive_alt_, 10.0);
  nh_private_.param<double>("track_alt", track_alt_, 10.0);
  nh_private_.param<double>("hover_yaw_rad", hover_yaw_, 4.0);  
  nh_private_.param<double>("vxy_max", vxy_max_, 3.0);
  nh_private_.param<double>("acc_max",acc_max_, 6);
  nh_private_.param<double>("Kp", Kp_, 1.0);
  nh_private_.param<double>("Ki", Ki_, 0.0);
  nh_private_.param<double>("Kd", Kd_, 0.0);
  nh_private_.param<double>("hover_center_deviation", hover_center_deviation_, 3.0);
  nh_private_.param<double>("track_center_deviation", track_center_deviation_, 2.0);
  nh_private_.param<double>("away_center_deviation", away_center_deviation_, 6.0);

  // nh_private_.param<double>("car_initposx", car_initposx_, 1.0);
  // nh_private_.param<double>("car_initposy", car_initposy_, 1.0);
  nh_private_.param<float>("car_initLat_deg", car_initLat_, 0.0);
  nh_private_.param<float>("car_initLon_deg", car_initLon_, 0.0);
  nh_private_.param<float>("car_Lat1_", car_Lat1_, 0.0);
  nh_private_.param<float>("car_Lon1_", car_Lon1_, 0.0);
  nh_private_.param<float>("car_Lat2_", car_Lat2_, 0.0);
  nh_private_.param<float>("car_Lon2_", car_Lon2_, 0.0);
  nh_private_.param<float>("car_Lat3_", car_Lat3_, 0.0);
  nh_private_.param<float>("car_Lon3_", car_Lon3_, 0.0);
  nh_private_.param<float>("car_Lat4_", car_Lat4_, 0.0);
  nh_private_.param<float>("car_Lon4_", car_Lon4_, 0.0);
  nh_private_.param<float>("car_Lat5_", car_Lat5_, 0.0);
  nh_private_.param<float>("car_Lon5_", car_Lon5_, 0.0);
  nh_private_.param<float>("car_Lat6_", car_Lat6_, 0.0);
  nh_private_.param<float>("car_Lon6_", car_Lon6_, 0.0);
  nh_private_.param<float>("car_Lat7_", car_Lat7_, 0.0);
  nh_private_.param<float>("car_Lon7_", car_Lon7_, 0.0);
  nh_private_.param<double>("Mission_sec", Mission_sec_, 200.0);
  nh_private_.param<double>("lost_sec_thre",lost_sec_thre_, 7.0);
  nh_private_.param<double>("v_thres_yawrotate",v_thres_yawrotate_, 1.7);
  nh_private_.param<bool>("auto_arm",auto_arm_, true);
  nh_private_.param<bool>("en_preset_mode",en_preset_mode_, false);
  nh_private_.param<bool>("en_yaw_rotate",en_yaw_rotate_, false);
  nh_private_.param<int>("uav_number",uav_number_, 1);
  nh_private_.param<double>("T_flytocar",T_flytocar_, 5);
  nh_private_.param<double>("T_track",T_track_, 2);


  cout << "loop_sec: " << loop_sec_ << endl;
  cout << "Mission_sec: " << Mission_sec_ << endl;
  cout << "auto_arm: " << auto_arm_ << endl;
  cout << "lost_sec_thre: " << lost_sec_thre_ << endl;
  cout << "en_preset_mode: " << en_preset_mode_ << endl;
  cout << "v_thres_yawrotate: " << v_thres_yawrotate_ << endl;
  cout << "arrive_alt: " << arrive_alt_ << endl;
  cout << "track_alt: " << track_alt_ << endl;
  cout << "hover_yaw_rad: " << hover_yaw_ << endl;
  cout << "vxy_max: " << vxy_max_ << endl;
  cout << "acc_max: " << acc_max_ << endl;
  cout << "Kp: " << Kp_ << endl;
  cout << "Kd: " << Kd_ << endl;
  cout << "Ki: " << Ki_ << endl;
  cout << "car_initLat_deg: " << car_initLat_ << endl;
  cout << "car_initLon_deg: " << car_initLon_ << endl;
  cout << "en_yaw_rotate: " << en_yaw_rotate_ << endl;
  // cout << "罗炯在此: "<< endl;

  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &uavCtrl::mavpose_cb, this, ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &uavCtrl::mavtwist_cb, this, ros::TransportHints().tcpNoDelay());
  px4stateSub_ = nh_.subscribe("mavros/state", 1, &uavCtrl::px4state_cb, this, ros::TransportHints().tcpNoDelay());
  //gimbalSub_ = nh_.subscribe("/gimbal/gimbal_state", 1, &uavCtrl::gimbal_cb, this, ros::TransportHints().tcpNoDelay());
  jc_cmdSub_ = nh_.subscribe("/jc_cmd", 1, &uavCtrl::jc_cmd_cb, this, ros::TransportHints().tcpNoDelay());//设置代码
  target_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  shape_yaw_pub_ = nh_.advertise<std_msgs::Float32>("/yaw_east_to_head_deg", 10);
  vision_pub_ = nh_.advertise<std_msgs::Int8>("flight_stage", 10);
  mavLaLu_init_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("mavLaLu_init_", 10);
  search_setpoint_pub_ = nh_.advertise<nav_msgs::Odometry>("search_setpoint", 10);
  if (uav_number_==4)
  {
    uav5_mavLaLu_init_Sub_ = nh_.subscribe("/uav5/mavLaLu_init_",10,&uavCtrl::uav5_mavLaLu_init_cb, this, ros::TransportHints().tcpNoDelay());
    carpos_from_uav5_Sub_ = nh_.subscribe("/uav5/search_setpoint",10,&uavCtrl::carpos_from_uav5_cb,this,ros::TransportHints().tcpNoDelay());
  }
  if (uav_number_==5)
  {
    uav4_mavLaLu_init_Sub_ = nh_.subscribe("/uav4/mavLaLu_init_",10,&uavCtrl::uav4_mavLaLu_init_cb, this, ros::TransportHints().tcpNoDelay());
    carpos_from_uav4_Sub_ = nh_.subscribe("/uav4/search_setpoint",10,&uavCtrl::carpos_from_uav4_cb,this,ros::TransportHints().tcpNoDelay());
  }
  //en_track_pub_ = nh_.advertise<std_msgs::String>("gimbal/gimbal_cmd", 10);
	globalposSub_ = nh_.subscribe("mavros/global_position/global",10,&uavCtrl::globalpos_cb, this, ros::TransportHints().tcpNoDelay());
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  setMode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  uav_gazebo_Sub_ = nh_.subscribe("uavpos_gazebo",10,&uavCtrl::uavpos_gazebo_cb,this,ros::TransportHints().tcpNoDelay());
  // target_Sub_ = nh_.subscribe("carpos_gazebo",10,&uavCtrl::carpos_gazebo_cb,this,ros::TransportHints().tcpNoDelay());
  vision_target_Sub_ = nh_.subscribe("vision/target_position",10,&uavCtrl::carpos_vision_cb,this,ros::TransportHints().tcpNoDelay());
  car_number_Sub_ = nh_.subscribe("/car_number",10,&uavCtrl::car_number_cb,this,ros::TransportHints().tcpNoDelay());
  //precise_landing_client_ = nh_.serviceClient<std_srvs::SetBool>("precise_landing");
  publoop_timer_ = nh_.createTimer(ros::Duration(0.05), &uavCtrl::publoop_cb, this);  // Define timer for pub loop rate  
  cmdloop_timer_ = nh_.createTimer(ros::Duration(loop_sec_), &uavCtrl::cmdloop_cb, this);  // Define timer for constant loop rate  

  takeoff_triggered_ = false;
  offboard_triggered_ = false;
  return_triggered_ = false;
  find_flag_45_ = false;

  pidx.error_last = 0;
  pidx.integral = 0;
  pidy.error_last = 0;
  pidy.integral = 0;

  error_pE_ = 0.1;
  error_pN_ = 0.1;
  VxyPz_sp_ << 0.0,0.0,track_alt_;
  VxyPz_sp_tmp_ << 0.0,0.0,track_alt_;

  yaw_sp_ = hover_yaw_;

  gim_yaw_ = 0.0;
  gim_pitch_ = 0.0;
  gim_servo_state_ = "init";
  gim_track_state_ = "init";

  flag = 0;
  lost_num_ = 0;

  controller_state = PREPARE;
  last_state = PREPARE;
  command_ = BLANK;
  target_err_pE_ = 0;
  target_err_pN_ = 0;
  heading_ = 0;
  v_heading_ = 0;

  car_initposx_ = 0; car_initposy_ = 0;
  car_posx1_=0, car_posy1_=0, car_posx2_=0, car_posy2_=0, car_posx3_=0, car_posy3_=0;
  car_posx4_=0, car_posy4_=0, car_posx5_=0, car_posy5_=0, car_posx6_=0, car_posy6_=0;
  car_posx7_=0, car_posy7_=0;

  mavLat_ = 0;
  mavLon_ = 0;
  mavLat_init_ = 0;
  mavLon_init_ = 0;
  mavPE_init_ = 0;
  mavPN_init_ = 0; 

  time_init_ = 0;
  time_now_ = 0;

  car_number_=3;
  search_result_self_=0;
  search_result_another_=0;
}

void uavCtrl::cmdloop_cb(const ros::TimerEvent &event)
{
  switch (controller_state)
  {
  case PREPARE: //waiting and initilizing
	//TODO: decide whether is right lat and lon
    mavLat_init_ = mavLat_;
    mavLon_init_ = mavLon_;
    mavPE_init_ = mavPos_(0);
    mavPN_init_ = mavPos_(1); 
    if(command_ != BLANK)
    {
      cout << "warning: the jc2fk.txt content is not wait!" << endl;
    }
    //TODO:how to avoid that the initial data in txt is takeoff
    // cout << command_  << endl;
    // cout << "car_pos[:" <<car_initposx_<<","<<car_initposy_<<"]"<< endl;
    // cout<< "uav_pos:["<< mavPos_(0)<<","<<mavPos_(1)<<"]"<<endl;
    if (command_ == LAUNCH)
    {
      time_init_ = ros::Time::now().toSec();
      controller_state = TAKEOFF;
      cout << "TAKEOFF" << endl;
      //controller_state = TRACK; //just for test,straight to track mode
      //cout << "TRACK" << endl; //just for test,straight to track mode
      break;
    }
    //compute car_initposx_, car_initposy_ in local ENU
    if ((car_initLat_ == 0) || (car_initLon_ == 0))
    {
      cout << "warning: car_initLat_ or car_initLon_ is 0!" << endl;
      break;
    }
    else
    {
      compute_preset_pos(); //gps to local
      // car_uav_init_position();
    }
    last_state = PREPARE;
    // cout << "PREPARE" << endl;
    break;
  
  case TAKEOFF:
    // if (command_ == BLANK)
    // {
    //   controller_state = PREPARE;
    //   cout << "PREPARE" << endl;
    //   break;
    // }
    pub_mavLaLu_init();
    if ((car_initposx_ == 0) || (car_initposy_ == 0))
    {
	      cout << "warning: no car init pose!" << endl;
        break;
    }
    if(!px4_state_.armed && auto_arm_)  //仿真中可以自动解锁
    {
      arm_cmd_.request.value = true; //spx
      arming_client_.call(arm_cmd_); //spx
      ros::Duration(0.5).sleep(); //spx
      cout << "has armed" <<endl;
    }
    //under 2m, continuing to trigger takeoff
    if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) < 2.0))
    {
      mode_cmd_.request.custom_mode = "AUTO.TAKEOFF";
      setMode_client_.call(mode_cmd_);
    }
    if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) > 2.0))
    {
      controller_state = FLYTOCAR;
      time_flytocar_start_ = ros::Time::now().toSec();
      cout << "FLYTOCAR" << endl;
      break;
    }  
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = TAKEOFF;
    cout << "TAKEOFF" << endl;
    break;

  case FLYTOCAR:
    t_flytocar_=ros::Time::now().toSec()-time_flytocar_start_;
    cout << "t_flytocar " << t_flytocar_ << endl;
    double hover_x,hover_y;
    if (t_flytocar_ >= T_flytocar_)
    {
      t_flytocar_ = T_flytocar_;
    }
    if (uav_number_ == 1)
    {
      hover_x = car_initposx_*t_flytocar_/T_flytocar_;
      hover_y = (car_initposy_+hover_center_deviation_)*t_flytocar_/T_flytocar_;
      hover_x_ = car_initposx_;
      hover_y_ = car_initposy_+hover_center_deviation_;
      PxyPz_sp << hover_x, hover_y, arrive_alt_;
    }
    else if (uav_number_ == 2)
    {
      hover_x = (car_initposx_+sqrt(3)/3*hover_center_deviation_)*t_flytocar_/T_flytocar_;
      hover_y = (car_initposy_-0.5*hover_center_deviation_)*t_flytocar_/T_flytocar_;
      hover_x_ = car_initposx_+sqrt(3)/3*hover_center_deviation_;
      hover_y_ = car_initposy_-0.5*hover_center_deviation_;
      PxyPz_sp <<hover_x, hover_y, arrive_alt_;
    }
    else if (uav_number_ == 3)
    {
      hover_x = (car_initposx_-sqrt(3)/3*hover_center_deviation_)*t_flytocar_/T_flytocar_;
      hover_y = (car_initposy_-0.5*hover_center_deviation_)*t_flytocar_/T_flytocar_;
      hover_x_ = car_initposx_-sqrt(3)/3*hover_center_deviation_;
      hover_y_ = car_initposy_-0.5*hover_center_deviation_;
      PxyPz_sp <<hover_x, hover_y, arrive_alt_;
    }
    else if (uav_number_ == 4)
    {
      hover_x = (car_initposx_+1)*t_flytocar_/T_flytocar_;
      hover_y = (car_initposy_+5)*t_flytocar_/T_flytocar_;
      hover_x_ = car_initposx_+1;
      hover_y_ = car_initposy_+5;
      PxyPz_sp <<hover_x, hover_y, arrive_alt_;
    }
    else if (uav_number_ == 5)
    {
      hover_x = (car_initposx_-1)*t_flytocar_/T_flytocar_;
      hover_y = (car_initposy_-5)*t_flytocar_/T_flytocar_;
      hover_x_ = car_initposx_-1;
      hover_y_ = car_initposy_-5;
      PxyPz_sp <<hover_x, hover_y, arrive_alt_;
    }

    
    pubPxyPzCmd(PxyPz_sp);
    if((px4_state_.mode != "OFFBOARD")&&(offboard_triggered_ == false)) //can switch to manual mode
    {
      mode_cmd_.request.custom_mode = "OFFBOARD";
			if(setMode_client_.call(mode_cmd_))
      {
        offboard_triggered_ = true;
        cout << "offboard mode triggered" << endl;
      }      
    }
    // cout << "uav_pos[:" <<mavPos_(0)<<","<<mavPos_(1)<<"]"<< endl;
    // cout << "car_pos[:" <<car_initposx_<<","<<car_initposy_<<"]"<< endl;
    if( (fabs(mavPos_(0)-hover_x_) < 0.2) && (fabs(mavPos_(1)-hover_y_) < 0.2) )
    {
      controller_state = HOVER_ON_CAR;
      cout << "HOVER_ON_CAR" << endl;
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = FLYTOCAR;
    //cout << "FLYTOCAR" << endl;
    break;

  case HOVER_ON_CAR:
  {
    std_msgs::Int8 flag_vision;
    flag_vision.data = 1;
    vision_pub_.publish(flag_vision);    //向感知传递flag表示可以开始识别图像
    if (uav_number_ == 1||uav_number_ == 2||uav_number_ == 3)
    {
      PxyPz_sp << hover_x_, hover_y_, track_alt_;
      double x_uav2car = car_initposx_-mavPos_(0);
      double y_uav2car = car_initposy_-mavPos_(1);
      hover_yaw_=atan2(y_uav2car,x_uav2car);
      // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
      pubPxyPzCmd(PxyPz_sp);
      if((fabs(mavPos_(2)-track_alt_) < 0.5))//&&(fabs(heading_-hover_yaw_)<0.09))
      {
        controller_state = TRACK;
        cout << "TRACK" << endl;
        break;
      }
      if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
      {
        controller_state = FINISH;
        cout << "RETURN, time is up" << endl;
        break;      
      }
      last_state = HOVER_ON_CAR;
      //cout << "HOVER_ON_CAR" << endl;
      break;
    }
    //for uav4,5
    //向感知传递flag表示可以开始识别图像
    if (uav_number_== 4)
    {
      if (find_flag_45_ == false)
      {
        compute_search_result();
        //pub自己决定去的车的位置和搜索结果
        cout << "进入 4机" << endl;
        pub_search_setpoint();
      }
      if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
      {
        controller_state = FINISH;
        cout << "RETURN, time is up" << endl;
        break;      
      }
      if (search_result_self_==0 && search_result_another_==0)
      {
        // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
        pubPxyPzCmd(PxyPz_sp);
        last_state = HOVER_ON_CAR;
        break;
      }
      cout << "search result:" << search_result_self_<<","<<search_result_another_<<endl;
      if ((search_result_self_==1 && search_result_another_==0)||(search_result_self_==1 && search_result_another_==1))
      {
        find_flag_45_=true;
        PxyPz_sp << car_pose_(0)+track_center_deviation_,car_pose_(1) , track_alt_;
        // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
        pubPxyPzCmd(PxyPz_sp);
      }
      if (search_result_self_==0 && search_result_another_==1)
      {
        find_flag_45_=true;
        PxyPz_sp << car_pose_from_another_(0)+track_center_deviation_,car_pose_from_another_(1) , track_alt_;
        // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
        pubPxyPzCmd(PxyPz_sp);
      }
      if((fabs(mavPos_(0)-PxyPz_sp(0)) < 0.2) && (fabs(mavPos_(1)-PxyPz_sp(1)) < 0.2)&&(fabs(mavPos_(2)-PxyPz_sp(2)) < 0.2) )
      {
        controller_state = TRACK;
        cout << "TRACK" << endl;
        break;
      }
      last_state = HOVER_ON_CAR;
      //cout << "HOVER_ON_CAR" << endl;
      break;
    }
    if (uav_number_== 5)
    {
      if (find_flag_45_ == false)
      {
        compute_search_result();
        //pub自己看到车的位置和搜索结果
        cout << "进入 5机" << endl;
        pub_search_setpoint();
      }
      if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
      {
        controller_state = FINISH;
        cout << "RETURN, time is up" << endl;
        break;      
      }
      if (search_result_self_==0 && search_result_another_==0)
      {
        // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
        pubPxyPzCmd(PxyPz_sp);
        last_state = HOVER_ON_CAR;
        break;
      }
      if ((search_result_self_==1 && search_result_another_==0))
      {
        find_flag_45_=true;
        PxyPz_sp << car_pose_(0)-track_center_deviation_,car_pose_(1) , track_alt_;
        // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
        pubPxyPzCmd(PxyPz_sp);

      }
      if ((search_result_self_==0 && search_result_another_==1)||(search_result_self_==1 && search_result_another_==1))
      {
        find_flag_45_=true;
        PxyPz_sp << car_pose_from_another_(0)-track_center_deviation_,car_pose_from_another_(1) , track_alt_;
        // pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
        pubPxyPzCmd(PxyPz_sp);

      }
      cout << "search result:" << search_result_self_<<","<<search_result_another_<<endl;
      if((fabs(mavPos_(0)-PxyPz_sp(0)) < 0.2) && (fabs(mavPos_(1)-PxyPz_sp(1)) < 0.2)&&(fabs(mavPos_(2)-PxyPz_sp(2)) < 0.2) )
      {
        cout << "5机完成搜索进入到TRACK" << endl;
        cout << "search result:" << search_result_self_<<","<<search_result_another_<<endl;
        controller_state = TRACK;
        cout << "TRACK" << endl;
        break;
      }
      last_state = HOVER_ON_CAR;
      //cout << "HOVER_ON_CAR" << endl;
      break;
    }
}
  case TRACK:
    cout<<"TRACK"<<uav_number_<<endl;
    yaw_sp_=hover_yaw_;
    if(px4_state_.mode != "OFFBOARD")
    {
      pidx.integral = 0.0;
      pidy.integral = 0.0;
      VxyPz_sp_ << 0.0,0.0,track_alt_;
      cout<<"WAITING OFFBOARD!"<<endl;
    }
    else //offboard
    {
      if(flag == 0)
      {
        if((lost_num_ < lost_sec_thre_/loop_sec_))
        {
          lost_num_ = lost_num_ +1;
          cout << "LOST! seconds: "<< lost_num_*loop_sec_ << endl;
        }
        else
        {
          lost_num_ = 0;
          //TODO: fly to a preset position, another mode
          if(en_preset_mode_)
          {
            controller_state = PRESET;
            cout << "FLY TO PRESET" << endl;
          }
        }
      }
      else
      {
        lost_num_ = 0;
        // computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
        // cout << "error_pE: "<<target_err_pE_<<", error_pN: "<<target_err_pN_<< endl;
        if(en_yaw_rotate_)
        {
          if(sqrt(mavVel_(0)*mavVel_(0)+mavVel_(1)*mavVel_(1)) > v_thres_yawrotate_)
          {
            // yaw_sp_ = v_heading_ + PI/2;
            yaw_sp_=hover_yaw_;
          //TODO: cancle yaw_sp change, pub v_heading_ to /yaw_east_to_head_deg
          }          
        }
        // if (last_car_number_!=car_number_)
        // {
        //   time_track_start_ = ros::Time::now().toSec();
        // }
        // t_track_= ros::Time::now().toSec()-time_track_start_;
        // if (t_track_>=T_track_)
        // {
        //   t_track_=T_track_;
        // }
        // switch (uav_number_)
        // {
        if (uav_number_==1)
        {
          if (car_number_ == 0)
          {
            //撤开
            double des_pose_PE = car_pose_(0);
            double des_pose_PN = car_pose_(1)+away_center_deviation_;
            // double des_pose_PE = car_pose_(0)+0*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)+1*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
          else
          {
            //持续跟车
            double des_pose_PE = car_pose_(0);
            double des_pose_PN = car_pose_(1)+track_center_deviation_;
            // double des_pose_PE = car_pose_(0)+0*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)+1*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            cout<< "des_pose_1:["<< des_pose_PE<<","<<des_pose_PN<<"]"<<endl;
            cout<< "uav_pos_ENU_1:["<< mavPos_(0)<<","<<mavPos_(1)<<"]"<<endl;
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
        }
        if (uav_number_==2)
        {
          if (car_number_ == 0 || car_number_ == 1)
          {
            //撤开
            double des_pose_PE = car_pose_(0)+sqrt(3)/3*away_center_deviation_;
            double des_pose_PN = car_pose_(1)-0.5*away_center_deviation_;
            // double des_pose_PE = car_pose_(0)+sqrt(3)/3*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)-0.5*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
          else 
          {
            double des_pose_PE = car_pose_(0)+sqrt(3)/3*track_center_deviation_;
            double des_pose_PN = car_pose_(1)-0.5*track_center_deviation_;
            // double des_pose_PE = car_pose_(0)+sqrt(3)/3*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)-0.5*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            cout<< "des_pose_2:["<< des_pose_PE<<","<<des_pose_PN<<"]"<<endl;
            cout<< "uav_pos_ENU_2:["<< mavPos_(0)<<","<<mavPos_(1)<<"]"<<endl;
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
        }
        if (uav_number_==3)
        {
          if (car_number_ == 3)
          {
            double des_pose_PE = car_pose_(0)-sqrt(3)/3*track_center_deviation_;
            double des_pose_PN = car_pose_(1)-0.5*track_center_deviation_;
            // double des_pose_PE = car_pose_(0)-sqrt(3)/3*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)-0.5*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            cout<< "des_pose_3:["<< des_pose_PE<<","<<des_pose_PN<<"]"<<endl;
            cout<< "uav_pos_ENU_3:["<< mavPos_(0)<<","<<mavPos_(1)<<"]"<<endl;
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
          else
          {
            double des_pose_PE = car_pose_(0)-sqrt(3)/3*away_center_deviation_;
            double des_pose_PN = car_pose_(1)-0.5*away_center_deviation_;
            // double des_pose_PE = car_pose_(0)-sqrt(3)/3*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)-0.5*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
        }
        
        //4,5机跟踪
        if (uav_number_ == 4)
        {
          if (car_number_ == 0)
          {
            //撤开
            double des_pose_PE = car_pose_(0)+away_center_deviation_;;
            double des_pose_PN = car_pose_(1);
            // double des_pose_PE = car_pose_(0)+0*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)+1*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
          else
          {
            //持续跟车
            double des_pose_PE = car_pose_(0)+track_center_deviation_;
            double des_pose_PN = car_pose_(1);
            // double des_pose_PE = car_pose_(0)+0*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)+1*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            cout<< "des_pose_4:["<< des_pose_PE<<","<<des_pose_PN<<"]"<<endl;
            cout<< "uav_pos_ENU_4:["<< mavPos_(0)<<","<<mavPos_(1)<<"]"<<endl;
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
        }

        if (uav_number_==5)
        {
          if (car_number_ == 0 || car_number_ == 1)
          {
            //撤开
            double des_pose_PE = car_pose_(0)-away_center_deviation_;
            double des_pose_PN = car_pose_(1);
            // double des_pose_PE = car_pose_(0)+sqrt(3)/3*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)-0.5*(track_center_deviation_+(away_center_deviation_-track_center_deviation_)*t_track_/T_track_);
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
          else 
          {
            double des_pose_PE = car_pose_(0)-track_center_deviation_;
            double des_pose_PN = car_pose_(1);
            // double des_pose_PE = car_pose_(0)+sqrt(3)/3*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            // double des_pose_PN = car_pose_(1)-0.5*(away_center_deviation_+(track_center_deviation_-away_center_deviation_)*t_track_/T_track_);
            cout<< "des_pose_5:["<< des_pose_PE<<","<<des_pose_PN<<"]"<<endl;
            cout<< "uav_pos_ENU_5:["<< mavPos_(0)<<","<<mavPos_(1)<<"]"<<endl;
            target_err_pE_ = des_pose_PE-mavPos_(0);
            target_err_pN_ = des_pose_PN-mavPos_(1);
            computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
          }
        }
        last_car_number_=car_number_;
        // }
      }
      double x_uav2car = car_pose_(0)-mavPos_(0);
      double y_uav2car = car_pose_(1)-mavPos_(1);
      yaw_sp_=atan2(y_uav2car,x_uav2car);
    }
    AccLimit(VxyPz_sp_);
    // pubVxyPzYawCmd(VxyPz_sp_, yaw_sp_);
    pubVxyPzCmd(VxyPz_sp_);

    if (command_ == RETURN)
    {
      controller_state = FINISH;
      cout << "RETURN, receive return command" << endl;
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = TRACK;
    //cout << "TRACK" << endl;
    break;

  case PRESET:
    pidx.integral = 0.0;
    pidy.integral = 0.0;
    VxyPz_sp_ << 0.0,0.0,track_alt_;
    if (flag == 1)
    {
      controller_state = TRACK;
      cout << "TRACK" << endl;
      break;
    }
    if (command_ == RETURN)
    {
      controller_state = FINISH;
      cout << "RETURN, receive return command" << endl;
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    if ((ros::Time::now().toSec()-time_init_) < (180 + 24))
    {
      PxyPz_sp << car_posx1_, car_posy1_, track_alt_;
      yaw_sp_ = -163*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 24)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 49)) )
    {
      PxyPz_sp << car_posx2_, car_posy2_, track_alt_;
      yaw_sp_ = -76*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 49)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 61)) )
    {
      PxyPz_sp << car_posx3_, car_posy3_, track_alt_;
      yaw_sp_ = 0*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 61)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 84)) )
    {
      PxyPz_sp << car_posx4_, car_posy4_, track_alt_;
      yaw_sp_ = -87*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 84)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 102)) )
    {
      PxyPz_sp << car_posx5_, car_posy5_, track_alt_;
      yaw_sp_ = -12*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 102)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 125)) )
    {
      PxyPz_sp << car_posx6_, car_posy6_, track_alt_;
      yaw_sp_ = 20*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 +125)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 146)) )
    {
      PxyPz_sp << car_posx7_, car_posy7_, track_alt_;
      yaw_sp_ = 143*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 146)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 178)) )
    {
      PxyPz_sp << car_initposx_, car_initposy_, track_alt_;
    }

    // pubPxyzYawCmd(PxyPz_sp,yaw_sp_);
    pubPxyPzCmd(PxyPz_sp);
    last_state = PRESET;
    break;
  
  case FINISH:
    if((px4_state_.mode != "AUTO.RTL")&&(return_triggered_ == false))
    {
      mode_cmd_.request.custom_mode = "AUTO.RTL";
			if(setMode_client_.call(mode_cmd_))
      {
        return_triggered_ = true;
        cout << "return mode triggered" << endl;
      }      
    }
    last_state = FINISH;
    cout << "RETURN, receive return command" << endl;
    break;

  default:
    cout << "uav controller: error controller state!" << endl;
    break;
  }
}

void uavCtrl::publoop_cb(const ros::TimerEvent &event)
{
  std_msgs::Float32 msg;
  msg.data = heading_*180/PI - 90;
  shape_yaw_pub_.publish(msg);
}

void uavCtrl::computeVelCmd(Eigen::Vector3d &vxypz_sp, const double &error_px, const double &error_py)
{
  pidx.error = error_px;
  pidx.derivative = error_px - pidx.error_last;
  pidx.integral += error_px;
  pidx.integral = max(-vxy_max_*0.8, min(pidx.integral, vxy_max_*0.8));//saturation resist
  pidx.error_last = error_px;
  vxypz_sp(0) = Kp_*pidx.error + Kd_*pidx.derivative + Ki_*pidx.integral;
  vxypz_sp(0) = max(-vxy_max_, min(vxy_max_, vxypz_sp(0)));
  //cout << "vx_cmd: " << vxypz_sp(0) << endl;

  pidy.error = error_py;
  pidy.derivative = error_py - pidy.error_last;
  pidy.integral += error_py;
  pidy.integral = max(-vxy_max_*0.8, min(pidy.integral, vxy_max_*0.8));//saturation resist
  pidy.error_last = error_py;
  vxypz_sp(1) = Kp_*pidy.error + Kd_*pidy.derivative + Ki_*pidy.integral;
  vxypz_sp(1) = max(-vxy_max_, min(vxy_max_, vxypz_sp(1)));
  //cout << "vy_cmd: " << vxypz_sp(1) << endl;

  vxypz_sp(2) = track_alt_;
}

void uavCtrl::computeError(const float &yaw, const float &pitch,
              const Eigen::Vector4d &uavatt, const Eigen::Vector3d &uavpos)
{
  Eigen::Vector3d campose_FLU, campose_ENU;
  campose_FLU << -cos(yaw)*sin(pitch), -sin(yaw), -cos(yaw)*cos(pitch); //in FLU body frame
  //yaw y, pitch z. out z, in y. y back, z left
  float qw = uavatt(0);
  float qx = uavatt(1);
  float qy = uavatt(2);
  float qz = uavatt(3);
  float height = uavpos(2);
  Eigen::Quaterniond q(qw,qx,qy,qz);
  Eigen::Matrix3d Rib = q.toRotationMatrix();
  campose_ENU = Rib*campose_FLU; //in ENU frame
  error_pE_ = height/fabs(campose_ENU(2))*campose_ENU(0);
  error_pN_ = height/fabs(campose_ENU(2))*campose_ENU(1);
}

void uavCtrl::pubPxyPzCmd(const Eigen::Vector3d &cmd_p)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b110111111000; // pub pz+py+px. ignore yaw and yaw rate
  msg.position.x = cmd_p(0); //pub px
  msg.position.y = cmd_p(1); //pub py
  msg.position.z = cmd_p(2); // pub local z altitude setpoint
  target_pose_pub_.publish(msg);
}

void uavCtrl::pubPxyzYawCmd(const Eigen::Vector3d &cmd_p, const double &yaw_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b100111111000; // pub pz+py+px. ignore yaw and yaw rate
  msg.position.x = cmd_p(0); //pub px
  msg.position.y = cmd_p(1); //pub py
  msg.position.z = cmd_p(2); // pub local z altitude setpoint
  msg.yaw = yaw_sp;
  target_pose_pub_.publish(msg);
}

void uavCtrl::pubVxyPzYawCmd(const Eigen::Vector3d &cmd_sp, const double &yaw_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b100111000011; // pub vx+vy+vz+pz+yaw, vz is feedforward vel.
  msg.velocity.x = cmd_sp(0); //pub vx
  msg.velocity.y = cmd_sp(1); //pub vy
  msg.velocity.z = 0; //pub vz
  msg.position.z = cmd_sp(2); // pub local z altitude setpoint
  msg.yaw = yaw_sp;
  target_pose_pub_.publish(msg);
}

void uavCtrl::pubVxyPzCmd(const Eigen::Vector3d &cmd_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b110111000011; // pub vx+vy+vz+pz, vz is feedforward vel. Ignore yaw and yaw rate
  msg.velocity.x = cmd_sp(0); //pub vx
  msg.velocity.y = cmd_sp(1); //pub vy
  msg.velocity.z = 0; //pub vz
  msg.position.z = cmd_sp(2); // pub local z altitude setpoint
  target_pose_pub_.publish(msg);
}

void uavCtrl::AccLimit(Eigen::Vector3d &cmd_sp)
{
  float dvx = cmd_sp(0) - mavVel_(0);
  float dvy = cmd_sp(1) - mavVel_(1);
//TODO: not minus mavVel_, but cmd_sp_last!
  float dv = pow((dvx*dvx+dvy*dvy),0.5) ;
  if(dv/loop_sec_>acc_max_)
  {
    cmd_sp(0) = mavVel_(0) + acc_max_*loop_sec_/dv*dvx;
    cmd_sp(1) = mavVel_(1) + acc_max_*loop_sec_/dv*dvy;
  }
}

void uavCtrl::pub_body_VxyPzCmd(const Eigen::Vector3d &cmd_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 8; //pub in body local frame;
  msg.type_mask = 0b110111000011; // pub vx+vy+vz+pz, vz is feedforward vel. Ignore yaw and yaw rate
  msg.velocity.x = cmd_sp(0); //pub vx
  msg.velocity.y = cmd_sp(1); //pub vy
  msg.velocity.z = 0; //pub vz
  msg.position.z = cmd_sp(2); // pub local z altitude setpoint
  target_pose_pub_.publish(msg);
}

void uavCtrl::mavpose_cb(const geometry_msgs::PoseStamped &msg)
{
  mavPos_(0) = msg.pose.position.x;
  mavPos_(1) = msg.pose.position.y;
  mavPos_(2) = msg.pose.position.z;
  mavAtt_(0) = msg.pose.orientation.x;
  mavAtt_(1) = msg.pose.orientation.y;
  mavAtt_(2) = msg.pose.orientation.z;
  mavAtt_(3) = msg.pose.orientation.w;//顺序
  Eigen::Quaterniond quaternion(mavAtt_);
  Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
  if (abs(eulerAngle(1)) > 3) //pitch should be within (-pi/2,pi/2)
    heading_ = eulerAngle(0) - M_PI; //rad
  else
    heading_ = eulerAngle(0); //rad
    //cout<<heading_<<endl;
}

void uavCtrl::mavtwist_cb(const geometry_msgs::TwistStamped &msg)
{
  mavVel_(0) = msg.twist.linear.x;
  mavVel_(1) = msg.twist.linear.y;
  mavVel_(2) = msg.twist.linear.z;
  if (sqrt(mavVel_(0)*mavVel_(0)+mavVel_(1)*mavVel_(1)) > 0.02)
  {
    v_heading_ = atan2(mavVel_(1),mavVel_(0)); //rad
  }
}

void uavCtrl::globalpos_cb(const sensor_msgs::NavSatFix &msg)
{
  mavLat_ = msg.latitude;
  mavLon_ = msg.longitude;
}

/*
void uavCtrl::leaderpose_cb(const mavros_msgs::PositionTarget &msg)
{
  leaderPos_(0) = msg.position.x - init_x_; //in gazebo, the UAV origin is at the takeoff home postision
  leaderPos_(1) = msg.position.y - init_y_;
  leaderPos_(2) = msg.position.z;
  leaderVel_(0) = msg.velocity.x;
  leaderVel_(1) = msg.velocity.y;
  leaderVel_(2) = msg.velocity.z;
  leaderAcc_(0) = msg.acceleration_or_force.x;
  leaderAcc_(1) = msg.acceleration_or_force.y;
  leaderAcc_(2) = msg.acceleration_or_force.z;
}
*/

void uavCtrl::jc_cmd_cb(const std_msgs::Int32 &msg)
{
  int jc_cmd;
  jc_cmd = msg.data;
	// cout << "uav controller receive command: " << jc_cmd << endl;
  if(jc_cmd == 0) {command_ = BLANK;}
  else if(jc_cmd == 1) {command_ = LAUNCH;}
  else if(jc_cmd == 2) {command_ = FLYUP;}
  else if(jc_cmd == 3) {command_ = TO_ONE;}
  else if(jc_cmd == 4) {command_ = TO_TWO;}
  else if(jc_cmd == 5) {command_ = TO_THREE;}
  else if(jc_cmd == 6) {command_ = RETURN;}
  else {command_ = BLANK; cout << "uav controller: unknown jc_cmd" << endl;}
  cout << "command: " << command_ << endl;
}

// void uavCtrl::targetCallback(const geometry_msgs::Quaternion&msg)
// {
// 	//cout<<msg<<endl;
//   float err_f;
//   float err_l;
//   err_f = tan(msg.x/57.3) * mavPos_(2);//msg.x is f angle
// 	err_l = tan(msg.y/57.3) * mavPos_(2);//y
//   target_err_pE_ = err_f *cos(heading_)-err_l*sin(heading_);//east error
//   target_err_pN_ = err_f*sin(heading_)+err_l*cos(heading_);
// 	flag = msg.w;
// }

void uavCtrl::uavpos_gazebo_cb(const nav_msgs::Odometry &msg)
{
	//cout<<msg<<endl;
  uav_pose_gazebo(0)=msg.pose.pose.position.x;
  uav_pose_gazebo(1)=msg.pose.pose.position.y;
  // cout<< "uav_pos_gazebo:["<< uav_pose_gazebo(0)<<","<<uav_pose_gazebo(1)<<"]"<<endl;
}

void uavCtrl::carpos_gazebo_cb(const nav_msgs::Odometry &msg)
{
	//cout<<msg<<endl;
  car_pose_gazebo(0)=msg.pose.pose.position.x;
  car_pose_gazebo(1)=msg.pose.pose.position.y;
  car_pose_(0)=car_pose_gazebo(0)-uav_pose_gazebo(0)+mavPos_(0);
  car_pose_(1)=car_pose_gazebo(1)-uav_pose_gazebo(1)+mavPos_(1);
	flag = 0;
  // cout<< "car_pos_ENU:["<< car_pose_(0)<<","<<car_pose_(1)<<"]"<<endl;
}

void uavCtrl::carpos_vision_cb(const geometry_msgs::Pose &msg)
{
	cout<<msg<<endl;
  car_pose_(0)=msg.poses.orientation.x;
  car_pose_(1)=msg.poses.orientation.y;
  int number;//数字
  int length;//长度
  number = msg.poses.orientation.w;
	flag = (int)msg.poses.position.x;
  length =  msg.poses.position.y;
  // cout<< "car_pos_ENU:["<< car_pose_(0)<<","<<car_pose_(1)<<"]"<<endl;
}

void uavCtrl::carpos_from_uav4_cb(const nav_msgs::Odometry &msg)
{
	//cout<<msg<<endl;
  car_pose_from_another_(0)=msg.pose.pose.position.x;
  car_pose_from_another_(1)=msg.pose.pose.position.y;
  search_result_another_ = msg.pose.pose.position.z;
  search_result_another_=int(search_result_another_);
  cout<< "mavlalu_from_other:["<< mavLat_init_another_<<","<<mavLon_init_another_<<"]"<<endl;
  float dx=0, dy=0;
  gps_to_local(dx, dy,
                mavLat_init_another_, mavLon_init_another_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);
  cout<< "dxdy:["<< dx<<","<<dy<<"]"<<endl;
  car_pose_from_another_(0)= car_pose_from_another_(0)+dx;
  car_pose_from_another_(1)= car_pose_from_another_(1)+dy;
  // cout<< "car_pos_ENU:["<< car_pose_(0)<<","<<car_pose_(1)<<"]"<<endl;
}

void uavCtrl::carpos_from_uav5_cb(const nav_msgs::Odometry &msg)
{
	//cout<<msg<<endl;
  car_pose_from_another_(0)=msg.pose.pose.position.x;
  car_pose_from_another_(1)=msg.pose.pose.position.y;
  search_result_another_ = msg.pose.pose.position.z;
  search_result_another_=int(search_result_another_);
  float dx=0, dy=0;
  gps_to_local(dx, dy,
                mavLat_init_another_, mavLon_init_another_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);
  car_pose_from_another_(0)= car_pose_from_another_(0)+dx;
  car_pose_from_another_(1)= car_pose_from_another_(1)+dy;
  // cout<< "car_pos_ENU:["<< car_pose_(0)<<","<<car_pose_(1)<<"]"<<endl;
}

void uavCtrl::car_number_cb(const std_msgs::Int32 &msg)
{
	car_number_= msg.data;
  cout<< "car_number:["<< car_number_<<endl;
}

void uavCtrl::px4state_cb(const mavros_msgs::State &msg)
{
	px4_state_ = msg;
}
void uavCtrl::car_uav_init_position(const nav_msgs::Odometry &msg)
{
  car_initposx_ = msg.pose.pose.position.x;
  car_initposy_ = msg.pose.pose.position.y;
}
void uavCtrl::gps_to_local(float &local_E, float &local_N,
       const float &target_lat, const float &target_lon,
                                const float &ref_E, const float &ref_N,
                                  const float &ref_lat, const float &ref_lon)
{
  std::array<double, 2> delta_EN;
  delta_EN = wgs84::toCartesian({ref_lat, ref_lon}, {target_lat, target_lon});
  local_E = ref_E + delta_EN[0];
  local_N = ref_N + delta_EN[1];
}

void uavCtrl::compute_preset_pos()
{
  gps_to_local(car_initposx_, car_initposy_,
                car_initLat_, car_initLon_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);
  
  gps_to_local(car_posx1_, car_posy1_,
                car_Lat1_, car_Lon1_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);  
  gps_to_local(car_posx2_, car_posy2_,
                car_Lat2_, car_Lon2_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);         \
  gps_to_local(car_posx3_, car_posy3_,
                car_Lat3_, car_Lon3_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);  
  gps_to_local(car_posx4_, car_posy4_,
                car_Lat4_, car_Lon4_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);      
  gps_to_local(car_posx5_, car_posy5_,
                car_Lat5_, car_Lon5_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);  
  gps_to_local(car_posx6_, car_posy6_,
                car_Lat6_, car_Lon6_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);
  gps_to_local(car_posx7_, car_posy7_,
                car_Lat7_, car_Lon7_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);                                      
}

void uavCtrl::compute_search_result()
{
  if (sqrt(pow(car_pose_(0)-mavPos_(0),2)+pow(car_pose_(1)-mavPos_(1),2))<5)
  {
    search_result_self_=1;
  }
  else
  {
    search_result_self_=0;
  }
  
}

void uavCtrl::pub_mavLaLu_init()
{
  std_msgs::Float32MultiArray msg;
  msg.data.push_back(mavLat_init_);
  msg.data.push_back(mavLon_init_);
  mavLaLu_init_pub_.publish(msg);
}

void uavCtrl::pub_search_setpoint()
{
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = car_pose_(0);
  msg.pose.pose.position.y = car_pose_(1);
  msg.pose.pose.position.z = search_result_self_;
  search_setpoint_pub_.publish(msg);
}

void uavCtrl:: uav4_mavLaLu_init_cb(const std_msgs::Float32MultiArray &msg)
{
  mavLat_init_another_=msg.data.at(0);
  mavLon_init_another_=msg.data.at(1);
}

void uavCtrl:: uav5_mavLaLu_init_cb(const std_msgs::Float32MultiArray &msg)
{
  mavLat_init_another_=msg.data.at(0);
  mavLon_init_another_=msg.data.at(1);
}
