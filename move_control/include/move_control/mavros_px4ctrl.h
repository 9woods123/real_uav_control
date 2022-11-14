

#ifndef MAVROS2PX4
#define MAVROS2PX4

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include <nav_msgs/Odometry.h>
#include <chrono>

using namespace std;

#define DISARM_AND_OFFBOARD 1
#define TAKEOFF 2
#define POSTION_CONTROL 3
#define LAND 4
#define POS_VEL_CONTROL 5
#define HOVER 6
#define HOVER 6


class mavros2px4{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    string uav_name;

    mavros_msgs::PositionTarget Target_pos_yaw;
    nav_msgs::Odometry target_pos_vel;
    geometry_msgs::PoseStamped target_position;
    geometry_msgs::TwistStamped target_velocity;
    std::pair <geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> target_p_v;
    geometry_msgs::PoseStamped fcu_position;
    geometry_msgs::PoseStamped pose;
    mavros_msgs::State current_state;
    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandBool arm_cmd;
    std_msgs::Int32 mode_last;
    std_msgs::Int32 mode;
    std_msgs::Bool takeoff_flag_value;

    ros::Subscriber fcu_state_sub;
    ros::Subscriber fcu_position_sub;
    ros::Subscriber set_mode_sub;
    ros::Subscriber setpoint_raw_local_sub;
    ros::Subscriber target_posvel_sub; // subscribe the  target position and velocity

    ros::Publisher setpoint_raw_local_pub;
    ros::Publisher takeoff_flag_pub;

    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;
    ros::Rate rate_;

    float Disarm_height = 0.18;
    float Takeoff_heiget = 0.7;

public:

    mavros2px4(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void setParameters();

    float getyawfrompose(geometry_msgs::PoseStamped p);
    void  fcu_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void  fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void  set_mode_callback(const std_msgs::Int32ConstPtr msg);
    void  PostionTarget_Callback(const mavros_msgs::PositionTarget msg);
    //void Target_PosVel_Callback(const nav_msgs::Odometry msg);
    void  send_pos_setpoint(const geometry_msgs::PoseStamped pos_sp, float yaw_sp);
    void  send_posvel2px4(const mavros_msgs::PositionTarget& target);


    void  main_loop();


};

#endif //MAVROS2PX4

