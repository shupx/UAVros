   	volans项目的px4控制头文件
  	核心是offboard_control.h头文件，在任务节点中引用该头文件（复制到节点的头文件夹里去），就可以使用OffboardControl系列函数给pixhawk发送指令。
	如OffboardControl::send_body_velyz_setpoint函数可以在机体坐标系下发送yz速度期望值以及期望偏航角速度至飞控
	当然，一般任务节点的头文件中给OffboardControl换个名字，如加个下划线
	OffboardControl OffboardControl_
	这样任务节点中发送速度指令就用
	OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_)

	另外px4_control_cfg.h是方便位置式PID的一些struct，在跟踪、降落等需要PID的地方可以调用一下，例子参看landing_control_p200.cpp和langing_control_p200.h
	但是位置式PID不好呀！尤其积分项，容易积累过大


	相比于Prometheus_control,这种px4控制方式要求任务节点引用这里给出的offboard_control.h头文件，并且在节点程序里用OffboardControl_.send_body_velxyz_setpoint等发送速度、位置指令。缺点很明显：
	1、python任务节点无法引用这个头文件
	2、任务节点没有将速度、位置指令发布到某个话题，不便于任务节点独立开发
	因此，还需要写个类似于Prometheus_control中px4_pos_controller.cpp或者px4_sender.cpp的程序，最好还有px4_pos_estimator.cpp将读取的飞控状态发布，省去任务节点还要直接从mavros读状态