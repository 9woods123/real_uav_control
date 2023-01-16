

#ifndef MAVROS2PX4
#define MAVROS2PX4

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include <nav_msgs/Odometry.h>
#include <chrono>

#include "se3control//se3control.h"

using namespace std;

#define NO_ACTION 0
#define ARM_AND_OFFBOARD 1
#define TAKEOFF 2
#define POSTION_CONTROL 3
#define LAND 4
#define POS_VEL_CONTROL 5
#define HOVER 6
#define ATTITUDE_CONTROL 7



class mavros2px4{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    string uav_name;

    mav_control::motion_target target;    //  be published in mainloop to realize motion control.


    mavros_msgs::PositionTarget Target_pos_yaw;
    nav_msgs::Odometry target_pos_vel;
    geometry_msgs::PoseStamped target_position;
    geometry_msgs::TwistStamped target_velocity;
    std::pair <geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> target_p_v;
    geometry_msgs::PoseStamped fcu_position; // position from px4
    geometry_msgs::TwistStamped fcu_velocity; // velocity from px4
    geometry_msgs::PoseStamped pose;
    mavros_msgs::State current_state;
    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandBool arm_cmd;
    std_msgs::Int32 mode_last;
    std_msgs::Int32 mode;
    std_msgs::Bool takeoff_flag_value;

    ros::Subscriber fcu_state_sub;
    ros::Subscriber fcu_position_sub;
    ros::Subscriber fcu_velocity_sub;

    ros::Subscriber set_mode_sub;
    ros::Subscriber setpoint_raw_local_sub;
    ros::Subscriber target_posvel_sub; // subscribe the  target position and velocity


    ros::Timer cmdloop_timer;
    ros::Publisher setpoint_raw_local_pub;
    ros::Publisher takeoff_flag_pub;
    ros::Publisher  attitude_cmd_pub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;
    ros::Rate rate_;

    float Disarm_height = 0.3;
    float Takeoff_heiget = 0.7;
    float Kp_x_,Kp_y_,Kp_z_,Kv_x_,Kv_y_,Kv_z_;
    float norm_thrust_const_;
    float cmd_delay_bear_time=0.5;   // if there is no cmd received in 0.5s, turn to HOVER mode.

    ros::Time cmd_received_time;
    mav_control::se3control    controlLer;
    mav_control::control_cmd   control_cmd;


public:

    mavros2px4(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void setParameters();
    void set_target_from_fcu();
    void pub_attitude_cmd( const mav_control::control_cmd &cmd);
    float getyawfrompose(geometry_msgs::PoseStamped p);
    void  fcu_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void  fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void  fcu_velocity_callback(const geometry_msgs::TwistStamped ::ConstPtr &msg);
    void  set_mode_callback(const std_msgs::Int32ConstPtr msg);
    void  PostionTarget_Callback(const mavros_msgs::PositionTarget msg);
    //void Target_PosVel_Callback(const nav_msgs::Odometry msg);
    void  send_pos_setpoint(const geometry_msgs::PoseStamped pos_sp, float yaw_sp);
    void  send_pos_setpoint(const Eigen::Vector3d target_pos, float yaw_sp);
    void  send_posvel2px4(const mavros_msgs::PositionTarget& target);
    void  mainloop(const ros::TimerEvent& event);



};

#endif //MAVROS2PX4

