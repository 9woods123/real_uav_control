#include <ros/ros.h>
#include <mavros2px4ctrl/mavros2px4ctrl.h>

using namespace std;

mavros2px4::mavros2px4(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh), nh_private_(nh_private), rate_(20.0) {

    setParameters();


    setpoint_raw_local_pub = nh_.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);
    takeoff_flag_pub = nh_.advertise<std_msgs::Bool>(uav_name + "/takeoff", 10);

    attitude_cmd_pub = nh_.advertise<mavros_msgs::AttitudeTarget>(uav_name +"/mavros/setpoint_raw/attitude",1);



    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");



    fcu_state_sub = nh_.subscribe(uav_name + "/mavros/state",
                                  1, &mavros2px4::fcu_state_callback, this);

    fcu_position_sub = nh_.subscribe(uav_name + "/mavros/local_position/pose",
                                     1, &mavros2px4::fcu_position_callback, this);

    fcu_velocity_sub = nh_.subscribe(uav_name + "/mavros/local_position/velocity_local", 1,
                                     &mavros2px4::fcu_velocity_callback, this,
                                     ros::TransportHints().tcpNoDelay());

    set_mode_sub = nh_.subscribe(uav_name + "/set_mode",
                                 1, &mavros2px4::set_mode_callback, this);
    setpoint_raw_local_sub = nh_.subscribe(uav_name + "/position_with_yaw",
                                           1, &mavros2px4::PostionTarget_Callback, this);


    //  main loop
    cmdloop_timer=nh_.createTimer(rate_.expectedCycleTime(),&mavros2px4::mainloop,this);


}


void mavros2px4::setParameters() {

    nh_private_.param<string>("uav_name", uav_name, "/UAV_0");
    nh_private_.param<float>("Takeoff_heiget", Takeoff_heiget, 0.4);


    nh_private_.param<float>("Kp_x", Kp_x_, 5);
    nh_private_.param<float>("Kp_y", Kp_y_, 5);
    nh_private_.param<float>("Kp_z", Kp_z_, 10);
    nh_private_.param<float>("Kv_x", Kv_x_, 5);
    nh_private_.param<float>("Kv_y", Kv_y_, 5);
    nh_private_.param<float>("Kv_z", Kv_z_, 10);


    nh_private_.param<float>("norm_thrust_const", norm_thrust_const_, 0.03);



}
float mavros2px4::getyawfrompose(geometry_msgs::PoseStamped p) {
    float x = p.pose.orientation.x;
    float y = p.pose.orientation.y;
    float z = p.pose.orientation.z;
    float w = p.pose.orientation.w;

    float siny_cosp = +2 * (w * z + x * y);
    float cosy_cosp = +1 - 2 * (z * z + y * y);
    return atan2(siny_cosp, cosy_cosp);
}

void mavros2px4::pub_attitude_cmd(const mav_control::control_cmd &cmd) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd.body_rate_cmd(0);
    msg.body_rate.y = cmd.body_rate_cmd(1);
    msg.body_rate.z = cmd.body_rate_cmd(2);
    msg.type_mask = 128;  // Ignore orientation messages
    msg.orientation.w = cmd.attitude(0);
    msg.orientation.x = cmd.attitude(1);
    msg.orientation.y = cmd.attitude(2);
    msg.orientation.z = cmd.attitude(3);
    msg.thrust = cmd.body_rate_cmd(3);
    //std::cout<<"msg==="<<msg<<std::endl;
    attitude_cmd_pub.publish(msg);
}

void mavros2px4::set_target_from_fcu() {

    target.position<<fcu_position.pose.position.x,
                     fcu_position.pose.position.y,
                     fcu_position.pose.position.z;
    target.yaw= getyawfrompose(fcu_position);

}
void mavros2px4::fcu_state_callback(const mavros_msgs::State::ConstPtr &msg) {
//获取无人机状态
    current_state = *msg;
}

void mavros2px4::fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
//获取无人机的位置（来自飞控）
    fcu_position = *msg;
}

void mavros2px4::fcu_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    fcu_velocity = *msg;
}

void mavros2px4::set_mode_callback(const std_msgs::Int32ConstPtr msg) {
//获取发布的模式指令（来自set_mode.cpp）
    mode = *msg;
}

void mavros2px4::PostionTarget_Callback(const mavros_msgs::PositionTarget msg) {
    //std::cout<<"Target_pos_yaw===="<<Target_pos_yaw<<std::endl;
    Target_pos_yaw = msg;
}

void mavros2px4::send_posvel2px4(const mavros_msgs::PositionTarget &target) {
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask =
            // mavros_msgs::PositionTarget::IGNORE_PX |
            // mavros_msgs::PositionTarget::IGNORE_PY |
            // mavros_msgs::PositionTarget::IGNORE_PZ |
            // mavros_msgs::PositionTarget::IGNORE_VX |
            // mavros_msgs::PositionTarget::IGNORE_VY |
            // mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            // mavros_msgs::PositionTarget::IGNORE_YAW |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    pos_setpoint.coordinate_frame = 1;  //1:ENU,

    // pos_setpoint.type_mask = 0b111000000010;  // 111 000 000 010  xyz + yaw
    pos_setpoint.position = target.position;
    pos_setpoint.velocity = target.velocity;
    pos_setpoint.yaw = target.yaw;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void mavros2px4::send_pos_setpoint(const Eigen::Vector3d target_pos, float yaw_sp) {
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    // pos_setpoint.type_mask = 0b111000000010;  // 111 000 000 010  xyz + yaw
    pos_setpoint.coordinate_frame = 1;  //1:ENU,

    pos_setpoint.position.x = target_pos(0);
    pos_setpoint.position.y = target_pos(1);
    pos_setpoint.position.z = target_pos(2);

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}


void mavros2px4::mainloop(const ros::TimerEvent &event) {

        /* code for loop body */
        switch (mode.data) {

            //切换成offboard模式，且不起飞。
            case ARM_AND_OFFBOARD:

                set_target_from_fcu();

                if (current_state.mode != "OFFBOARD") {

                    //  local_pos_pub.publish(target_position);
                    set_mode.request.custom_mode = "OFFBOARD";
                    set_mode_client.call(set_mode);
                    if (set_mode_client.call(set_mode) &&
                        set_mode.response.mode_sent) {
                        ROS_INFO_STREAM("\033[32m Offboard enabled \033[0m");
                    }
                }

                if (!current_state.armed) {         //try to arm  if  uav is not armed
                    arm_cmd.request.value = true;
                    arming_client.call(arm_cmd);
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                        ROS_INFO_STREAM("\033[32m Vehicle armed \033[0m");
                    }
                }

                mode_last.data = ARM_AND_OFFBOARD;
                break;

                //切换成offboard模式后，原地起飞。
            case TAKEOFF:
                //设置起飞点fcu_position

                if (mode_last.data != TAKEOFF ) {
                    set_target_from_fcu();
                    target.position.z()+=Takeoff_heiget;
                    mode_last.data = TAKEOFF;
                    ROS_INFO("Start Takeoff");
                    break;
                }
                else{
                    if (Takeoff_heiget - 0.1 < fcu_position.pose.position.z && fcu_position.pose.position.z
                                                                               < Takeoff_heiget + 0.1)   // for any x,  1<x<5 == true
                    {
                        ROS_INFO_STREAM("\033[32m Takeoff Successfully \033[0m");
                        mode.data = HOVER;                     //  turn to hover
                        takeoff_flag_value.data = 1;
                        takeoff_flag_pub.publish(takeoff_flag_value);  // publish if uav has taken off
                    }
                    mode_last.data = TAKEOFF;
                    break;
                }

            case POSTION_CONTROL:
                // fly to the target point
                target.position<<Target_pos_yaw.position.x,Target_pos_yaw.position.y,Target_pos_yaw.position.z;
                target.yaw=Target_pos_yaw.yaw;

                mode_last.data = POSTION_CONTROL;
                break;

            case LAND:
                //降落
                if (mode_last.data != LAND) {   // get current pose from fcu.
                    set_target_from_fcu();
                    mode_last.data = LAND;
                    ROS_INFO_STREAM("\033[32m Try to Land \033[0m");
                    break;
                }

                if (fcu_position.pose.position.z > Disarm_height)
                {
                    target.position.z()-= 0.1;
                    mode_last.data = LAND;
                    break;

                }
                else{
                    set_mode.request.custom_mode = "MANUAL";
                    set_mode_client.call(set_mode);
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                }

                if (!current_state.armed) {
                    ROS_INFO_STREAM("\033[31m vehicle disarmed \033[0m");

                }

                mode_last.data = LAND;
                break;

            case POS_VEL_CONTROL:
                //  fly with  pos and velocity
                send_posvel2px4(Target_pos_yaw);
                mode_last.data = POS_VEL_CONTROL;
                break;


            case HOVER:

                if (mode_last.data != HOVER) {
                    set_target_from_fcu();
                    ROS_INFO_STREAM("\033[32m Hover \033[0m");
                }
                mode_last.data = HOVER;
                break;


            case ATTITUDE_CONTROL:

                if (mode_last.data != ATTITUDE_CONTROL) {
                    ROS_INFO_STREAM("\033[32m Turn to ATTITUDE_CONTROL \033[0m");
                }

                target.position<<Target_pos_yaw.position.x,Target_pos_yaw.position.y,Target_pos_yaw.position.z;
                target.velocity<<Target_pos_yaw.velocity.x,Target_pos_yaw.velocity.y,Target_pos_yaw.velocity.z;
                target.velocity<<Target_pos_yaw.acceleration_or_force.x,Target_pos_yaw.acceleration_or_force.y,
                                 Target_pos_yaw.acceleration_or_force.z;
                target.yaw=Target_pos_yaw.yaw;

                mode_last.data = ATTITUDE_CONTROL;
                break;
        }

        controlLer.calculate_control(fcu_position,fcu_velocity,target,control_cmd);

        if(mode_last.data == ATTITUDE_CONTROL)             //  pub control msg to px4
        {
            pub_attitude_cmd(control_cmd);
        }
        else{
            send_pos_setpoint(target.position,target.yaw);
        }

        ros::spinOnce();

}



