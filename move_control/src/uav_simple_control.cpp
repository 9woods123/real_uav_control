/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

#define DISARM_AND_OFFBOARD 1
#define TAKEOFF 2
#define POSTION_CONTROL 3
#define LAND 4
#define POS_VEL_CONTROL 5

// #define XYZ_POS 1
// #define BODY_FRAME 1
// #define ENU_FRAME 1

using namespace std;

// ros::Publisher set_position_pub;
ros::Publisher set_mode_pub;
ros::Publisher set_position_yaw_pub;
ros::Publisher  target_posvel_pub;
mavros_msgs::State current_state; //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

geometry_msgs::PoseStamped target_position;
geometry_msgs::PoseStamped fcu_position;
geometry_msgs::PoseStamped macop_position;
double turn_yaw;
double getready_movement;

int BODY = 8;
int ENU = 1;
// float Disaerm_height=0.25;
float Takeoff_heiget;
std::string uav_name = "";
std_msgs::Int32 Num_StateMachine;




void fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    fcu_position = *msg;
}

float getyawfrompose(geometry_msgs::PoseStamped position)
{
    float x = position.pose.orientation.x;
    float y = position.pose.orientation.y;
    float z = position.pose.orientation.z;
    float w = position.pose.orientation.w;

    float siny_cosp = +2 * (w * z + x * y);
    float cosy_cosp = +1 - 2 * (z * z + y * y);
    return atan2(siny_cosp, cosy_cosp);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg) //从飞控回调无人机状态
{
    current_state = *msg;
}

void send_pos_setpoint(const geometry_msgs::PoseStamped pos_sp, float yaw_sp)
{
    Num_StateMachine.data = POSTION_CONTROL;
    set_mode_pub.publish(Num_StateMachine);

    mavros_msgs::PositionTarget pos_setpoint;
    // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    // Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000; // 100 111 111 000  xyz + yaw
    // pos_setpoint.type_mask = 0b111000000010;  // 111 000 000 010  xyz + yaw
    pos_setpoint.coordinate_frame = 1; // 1:ENU,

    pos_setpoint.position.x = pos_sp.pose.position.x;
    pos_setpoint.position.y = pos_sp.pose.position.y;
    pos_setpoint.position.z = pos_sp.pose.position.z;
    pos_setpoint.yaw = yaw_sp;

    set_position_yaw_pub.publish(pos_setpoint);
}


void send_posvel(const geometry_msgs::PoseStamped current_p,float vx,float vy,float vz,float yaw_sp,float dt)
{

    Num_StateMachine.data = POS_VEL_CONTROL;
    set_mode_pub.publish(Num_StateMachine);

    mavros_msgs::PositionTarget target_pos_vel;
    target_pos_vel.position.x = current_p.pose.position.x + vx*dt;
    target_pos_vel.position.y = current_p.pose.position.y + vy*dt;
    target_pos_vel.position.z = current_p.pose.position.z + vz*dt;
    target_pos_vel.velocity.x = vx;
    target_pos_vel.velocity.y = vy;
    target_pos_vel.velocity.z = vz;
    target_pos_vel.yaw = yaw_sp;
    set_position_yaw_pub.publish(target_pos_vel);
}

void send_pos_vel(const geometry_msgs::PoseStamped current_p, float vx,float vy,float vz,float dt)
{
    //  we do not use this function
    Num_StateMachine.data = POS_VEL_CONTROL;
    set_mode_pub.publish(Num_StateMachine);
    nav_msgs::Odometry target_p_v;
    target_p_v.header.frame_id="world";
    target_p_v.pose.pose.position.x=current_p.pose.position.x+dt*vx;
    target_p_v.pose.pose.position.y=current_p.pose.position.y+dt*vy;
    target_p_v.pose.pose.position.z=current_p.pose.position.z+dt*vz;
    target_p_v.twist.twist.linear.x=vx;
    target_p_v.twist.twist.linear.y=vy;
    target_p_v.twist.twist.linear.z=vz;
    target_posvel_pub.publish(target_p_v);
}

void takeoff()
{
    Num_StateMachine.data = TAKEOFF;
    set_mode_pub.publish(Num_StateMachine);
}

void land()
{
    Num_StateMachine.data = LAND;
    set_mode_pub.publish(Num_StateMachine);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_simple_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    nh_.param<std::string>("uav_name", uav_name, "/UAV_6"); //传参：无人机id
    nh.param<double>("turn_yaw", turn_yaw, 0.0);
    nh.param<double>("getready_movement", getready_movement, 0.3);

    ros::Subscriber fcu_position_sub = nh.subscribe(uav_name + "/mavros/local_position/pose", 100, fcu_position_callback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, state_cb);

    set_mode_pub = nh.advertise<std_msgs::Int32>(uav_name + "/set_mode", 10);
    target_posvel_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/target_pose_vel", 10);
    set_position_yaw_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/position_with_yaw", 10);
    // control the uav at  position and yaw.

    char start_flag[100];
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>woods Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
    ros::Rate rate(20.0);
    ros::Time::init();



    std::cout << "Please enter 1 to disarm and offborad the UAV." << std::endl;

    while (ros::ok())
    {
        std::cin.getline(start_flag, 100);
        if (start_flag[0] == '1' && strlen(start_flag) == 1)
        {
            if (!current_state.armed) //如果未解锁
            {
                Num_StateMachine.data = DISARM_AND_OFFBOARD;
                set_mode_pub.publish(Num_StateMachine);
                std::cout << "uav Arming... and offboard..." << std::endl;
                break;
            }
        }
        else
            std::cout << "Please enter 1 to disarm and offborad the UAV." << std::endl;
    }
    ros::Duration(3).sleep();



    takeoff();

    std::cout << "Please enter 1 to move  the UAV." << std::endl;
    char land_flag[100];

    while (ros::ok())
    {
        std::cin.getline(land_flag, 100);
        if (land_flag[0] == '1' && strlen(land_flag) == 1)
        {
            break;
        }
        std::cout << "Please enter 1 to move  the UAV." << std::endl;
    }

    ros::spinOnce();

    geometry_msgs::PoseStamped move_to_position;
    move_to_position = fcu_position;
    float dt=0.5;
    float yaw=0;


    for (int i = 0; i <= 11; i++)
    {
        std::cout << "========turn=======" << std::endl;
        // move_to_position.pose.position.x+=0.5;
        // std::cout<<"move_to_position=="<<move_to_position.pose<<std::endl;
        yaw+=3.14/12;
        send_posvel(fcu_position,0,0,0,yaw,dt);
        ros::Duration(dt).sleep();
        ros::spinOnce();
    }



    for (int i = 0; i <= 5; i++)
    {
        std::cout << "========move=======" << std::endl;
        // move_to_position.pose.position.x+=0.5;
        // std::cout<<"move_to_position=="<<move_to_position.pose<<std::endl;
        send_posvel(fcu_position,-1,0,0,getyawfrompose(fcu_position),dt);
        ros::Duration(dt).sleep();
        ros::spinOnce();
    }

    // send_pos_vel(fcu_position,0,0,0,dt);
    ros::Duration(2).sleep();

    land();

}
