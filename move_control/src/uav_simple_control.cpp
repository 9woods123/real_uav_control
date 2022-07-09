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
#include <std_msgs/Int32.h>

#define DISARM_AND_OFFBOARD 1
#define TAKEOFF 2
#define POSTION_CONTROL 3


// #define XYZ_POS 1
// #define BODY_FRAME 1
// #define ENU_FRAME 1

using namespace std;

// ros::Publisher set_position_pub;
ros::Publisher set_mode_pub;
ros::Publisher set_position_yaw_pub;

mavros_msgs::State current_state;         //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

geometry_msgs::PoseStamped target_position;
geometry_msgs::PoseStamped fcu_position;
geometry_msgs::PoseStamped macop_position;
double turn_yaw;
double getready_movement;

int BODY=8;
int ENU=1;
// float Disaerm_height=0.25;
float Takeoff_heiget;
std::string uav_name="/uav0";
double dt = 0.2;
void fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   fcu_position= *msg;
}
float getyawfrompose(geometry_msgs::PoseStamped)
{
    float x= target_position.pose.orientation.x;
    float y= target_position.pose.orientation.y;
    float z= target_position.pose.orientation.z;
    float w= target_position.pose.orientation.w;

    float siny_cosp=+2*(w*z+x*y);
    float cosy_cosp=+1 -2*(z*z+y*y);
    return atan2(siny_cosp,cosy_cosp);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)   //从飞控回调无人机状态
{
    current_state = *msg;
}

void send_pos_setpoint(const geometry_msgs::PoseStamped pos_sp, float yaw_sp)
{
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

    set_position_yaw_pub.publish(pos_setpoint);
}

int main(int argc, char** argv)
{
  ros:: init(argc, argv, "uav_simple_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  
  nh_.param<std::string>("uav_name",uav_name,"/uav0"); //传参：无人机id
  nh.param<double>("turn_yaw",turn_yaw, 0.0);
  nh.param<double>("getready_movement",getready_movement,0.3);


  // ros::Subscriber macop_position_sub=nh.subscribe(uav_name+"/mavros/vision_pose/pose",10,macop_position_callback);
  // ros::Subscriber fcu_state_sub = nh.subscribe(uav_name+"/mavros/state", 10, fcu_state_callback);

  ros::Subscriber fcu_position_sub = nh.subscribe(uav_name+"/mavros/local_position/pose", 100, fcu_position_callback);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uav_name+"/mavros/state",10, state_cb);

  set_mode_pub = nh.advertise<std_msgs::Int32>(uav_name+"/set_mode", 10);
  set_position_yaw_pub=nh.advertise<mavros_msgs::PositionTarget>(uav_name+"/position_with_yaw",10);
 // control the uav at  position and yaw.

  char start_flag[100];
  std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< std::endl;
  ros::Rate rate(20.0);
  ros::Time::init();

  std_msgs::Int32 Num_StateMachine;
  Num_StateMachine.data=0;

  std::cout << "Please enter 1 to disarm and offborad the UAV."<<std::endl;

  while(ros::ok())
  {   
      std::cin.getline(start_flag,100);
      if(start_flag[0] == '1' && strlen(start_flag) == 1)
          {
              if(!current_state.armed) //如果未解锁
              {
                Num_StateMachine.data=DISARM_AND_OFFBOARD;
                set_mode_pub.publish(Num_StateMachine);
                std::cout << "uav Arming... and offboard..." <<std::endl;
                break;
              }  
          }else std::cout << "Please enter 1 to disarm and offborad the UAV."<<std::endl;
  }
  ros::Duration(3).sleep();
  
  while (fcu_position.pose.position.z <= 0.5)
  {
  Num_StateMachine.data = TAKEOFF;
  set_mode_pub.publish(Num_StateMachine);
  std::cout << "========take off========"<<std::endl;
  ros::spinOnce();
  }

  // target_position=fcu_position;
  // double dyaw=turn_yaw/4;
  // double ty=0;

  geometry_msgs::PoseStamped move_to_position;
  move_to_position = fcu_position;
  double step_size = 0.02;

// move  10*0.02 m along the x axis.

  for (int i =0; i <=10; i++)
  {     
        Num_StateMachine.data=POSTION_CONTROL;
        set_mode_pub.publish(Num_StateMachine);
        // ty+=dyaw;
        move_to_position.pose.position.x += step_size;
        std::cout <<"========move========"<<std::endl;
        send_pos_setpoint(move_to_position, 0.0);
        ros::spinOnce();
    }



  }

