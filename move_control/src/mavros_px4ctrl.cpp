#include <ros/ros.h>
#include <move_control/mavros_px4ctrl.h>

using namespace std;

mavros2px4::mavros2px4(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
:nh_(nh),nh_private_(nh_private),rate_(20.0)

{

    setParameters();

    fcu_state_sub = nh_.subscribe(uav_name+"/mavros/state",
                                  1000, &mavros2px4::fcu_state_callback,this);
    fcu_position_sub = nh_.subscribe(uav_name+"/mavros/local_position/pose",
                                     1000, &mavros2px4::fcu_position_callback,this);
    set_mode_sub = nh_.subscribe(uav_name+"/set_mode",
                                 10, &mavros2px4::set_mode_callback,this);
    setpoint_raw_local_sub =  nh_.subscribe(uav_name+"/position_with_yaw",
                                            10, &mavros2px4::PostionTarget_Callback,this);


    setpoint_raw_local_pub = nh_.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);
    takeoff_flag_pub = nh_.advertise<std_msgs::Bool>(uav_name + "/takeoff", 10);

    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(uav_name+"/mavros/set_mode");
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>(uav_name+"/mavros/cmd/arming");


    main_loop();

}

void mavros2px4::setParameters() {

    nh_.param<string>("uav_name",uav_name,"/UAV_0");
    nh_.param<float>("Takeoff_heiget",Takeoff_heiget, 0.4);
}

float mavros2px4::getyawfrompose(geometry_msgs::PoseStamped p) {
    float x= p.pose.orientation.x;
    float y= p.pose.orientation.y;
    float z= p.pose.orientation.z;
    float w= p.pose.orientation.w;

    float siny_cosp=+2*(w*z+x*y);
    float cosy_cosp=+1 -2*(z*z+y*y);
    return atan2(siny_cosp,cosy_cosp);
}

void mavros2px4::fcu_state_callback(const mavros_msgs::State::ConstPtr &msg) {
//获取无人机状态
current_state=*msg;
}

void mavros2px4::fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
//获取无人机的位置（来自飞控）
    fcu_position= *msg;
}

void mavros2px4::set_mode_callback(const std_msgs::Int32ConstPtr msg) {
//获取发布的模式指令（来自set_mode.cpp）
    mode=*msg;
}

void mavros2px4::PostionTarget_Callback(const mavros_msgs::PositionTarget msg) {
    Target_pos_yaw=msg;
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
    pos_setpoint.position= target.position;
    pos_setpoint.velocity= target.velocity;
    pos_setpoint.yaw = target.yaw;

    setpoint_raw_local_pub.publish(pos_setpoint);}

void mavros2px4::send_pos_setpoint(const geometry_msgs::PoseStamped pos_sp, float yaw_sp) {
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    // pos_setpoint.type_mask = 0b111000000010;  // 111 000 000 010  xyz + yaw
    pos_setpoint.coordinate_frame = 1;  //1:ENU,

    pos_setpoint.position.x = pos_sp.pose.position.x;
    pos_setpoint.position.y = pos_sp.pose.position.y;
    pos_setpoint.position.z = pos_sp.pose.position.z;

    pos_setpoint.yaw = yaw_sp;
    setpoint_raw_local_pub.publish(pos_setpoint);
}

/*void mavros2px4::send_target2px4(const std::pair <geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped> target_p_v) {


}*/

void mavros2px4::main_loop() {
    while (ros::ok())
    {
        /* code for loop body */
        switch (mode.data)
        {

            //切换成offboard模式，且不起飞。
            case DISARM_AND_OFFBOARD:
                target_position.pose= fcu_position.pose;
                for(int i = 50; ros::ok() && i > 0; --i){
                    send_pos_setpoint(target_position,getyawfrompose(target_position));
                    ros::spinOnce();
                    rate_.sleep(); // 保证更新频率为20Hz
                }

                if( current_state.mode != "OFFBOARD")
                {

                    //  local_pos_pub.publish(target_position);
                    set_mode.request.custom_mode = "OFFBOARD";
                    set_mode_client.call(set_mode);
                    if(set_mode_client.call(set_mode) &&
                       set_mode.response.mode_sent)
                    {
                        ROS_INFO("Offboard enabled");
                    }
                }
                send_pos_setpoint(target_position,getyawfrompose(target_position));

                if(!current_state.armed)
                {
                    arm_cmd.request.value = true;
                    arming_client.call(arm_cmd);
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                }
                send_pos_setpoint(target_position,getyawfrompose(target_position));
                mode_last.data=DISARM_AND_OFFBOARD;
                break;

                //切换成offboard模式后，原地起飞。
            case TAKEOFF:
                //设置起飞点fcu_position

                if (mode_last.data!=TAKEOFF)
                {
                    target_position.pose.position.x = fcu_position.pose.position.x;
                    target_position.pose.position.y = fcu_position.pose.position.y;
                    target_position.pose.position.z = fcu_position.pose.position.z+Takeoff_heiget;
                    target_position.pose.orientation.x= fcu_position.pose.orientation.x;
                    target_position.pose.orientation.y= fcu_position.pose.orientation.y;
                    target_position.pose.orientation.z= fcu_position.pose.orientation.z;
                    target_position.pose.orientation.w= fcu_position.pose.orientation.w;
                    send_pos_setpoint(target_position,getyawfrompose(target_position));
                }

                send_pos_setpoint(target_position,getyawfrompose(target_position));

                if(Takeoff_heiget-0.1 <fcu_position.pose.position.z
                && fcu_position.pose.position.z< Takeoff_heiget+0.1)   // for any x,  1<x<5 == true
                {
                    ROS_INFO("Takeoff Successfully");
                    mode.data=HOVER;                     //  turn to hover
                    takeoff_flag_value.data=1;
                    takeoff_flag_pub.publish(takeoff_flag_value);  // publish if uav has taken off
                }

                mode_last.data=TAKEOFF;
                break;

            case POSTION_CONTROL:
                // fly to the target point
                setpoint_raw_local_pub.publish(Target_pos_yaw);
                mode_last.data=POSTION_CONTROL;
                break;

            case LAND:
                //降落
                if (mode_last.data!=LAND)
                {   // get current pose from fcu.
                    target_position.pose.position.x=fcu_position.pose.position.x;
                    target_position.pose.position.y=fcu_position.pose.position.y;
                    target_position.pose.position.z=fcu_position.pose.position.z;
                    target_position.pose.orientation=fcu_position.pose.orientation;
                    ROS_INFO("Land");
                }

                if (fcu_position.pose.position.z>Disarm_height)
                {
                    target_position.pose.position.z=fcu_position.pose.position.z-0.1;
                    send_pos_setpoint(target_position,getyawfrompose(target_position));
                }else
                {
                    set_mode.request.custom_mode="MANUAL";
                    set_mode_client.call(set_mode);
                    arm_cmd.request.value=false;
                    arming_client.call(arm_cmd);

                }
                if (!current_state.armed)
                {
                    ROS_INFO("vehicle armed");
                }

                mode_last.data=LAND;

                break;

            case POS_VEL_CONTROL:
                //  fly with  pos and velocity
                send_posvel2px4(Target_pos_yaw);
                mode_last.data=POS_VEL_CONTROL;
                break;


            case HOVER:

                if(mode_last.data!=HOVER)
                {
                    target_position=fcu_position;
                    ROS_INFO("Hover");
                }
                send_pos_setpoint(target_position,getyawfrompose(target_position));
                mode_last.data=HOVER;
                break;

        }




        ros::spinOnce();
        rate_.sleep();
    }

}



