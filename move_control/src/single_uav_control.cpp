#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

using namespace std;

std_msgs::Int32 mode_last;
std_msgs::Int32 quantity;
std_msgs::Float64 circle_radius;
string uav_name;
int uav_id;
std_msgs::Int32 mode;
float kaishi;
mavros_msgs::PositionTarget Target_pos_yaw;
geometry_msgs::PoseStamped target_position1;
geometry_msgs::PoseStamped target_position;
geometry_msgs::PoseStamped fcu_position;
geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;
float Disaerm_height=0.2;
float Takeoff_heiget=1.3;

ros::Publisher local_pos_pub;
ros::Publisher local_attitude_pub;
ros::Publisher setpoint_raw_local_pub;


float getyawfrompose(geometry_msgs::PoseStamped target_position )
{
    float x= target_position.pose.orientation.x;
    float y= target_position.pose.orientation.y;
    float z= target_position.pose.orientation.z;
    float w= target_position.pose.orientation.w;

    float siny_cosp=+2*(w*z+x*y);
    float cosy_cosp=+1 -2*(z*z+y*y);
    return atan2(siny_cosp,cosy_cosp);
}


//获取无人机状态
void fcu_state_callback(const mavros_msgs::State::ConstPtr& msg)
{
     current_state=*msg;
}

//获取无人机的位置（来自飞控）
void fcu_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   fcu_position= *msg;
}

//获取发布的模式指令（来自set_mode.cpp）
void set_mode_callback(const std_msgs::Int32ConstPtr msg)
{
    mode=*msg;
}

void position_Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    pose.pose.position.x=msg->pose.position.x;
    pose.pose.position.y=msg->pose.position.y;
    pose.pose.position.z=msg->pose.position.z;
}

void quantity_Callback(const std_msgs::Int32ConstPtr msg)
{
     quantity=*msg;
}

void circle_radius_Callback(const std_msgs::Float64ConstPtr msg)
{
     circle_radius=*msg;
}

void PostionTarget_Callback(const mavros_msgs::PositionTarget msg)
{
     Target_pos_yaw=msg;
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

    setpoint_raw_local_pub.publish(pos_setpoint);
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "control");
  ros::NodeHandle nh("~");
  
  nh.param<string>("uav_name",uav_name,"/uav0");
  nh.param<int>("uav_id",uav_id,0);
  nh.param<float>("Takeoff_heiget",Takeoff_heiget, 0.6);

  ros::Subscriber fcu_state_sub = nh.subscribe(uav_name+"/mavros/state", 1000, fcu_state_callback);
  ros::Subscriber fcu_position_sub = nh.subscribe(uav_name+"/mavros/local_position/pose", 1000, fcu_position_callback);
  ros::Subscriber set_mode_sub = nh.subscribe(uav_name+"/set_mode", 10, set_mode_callback);
  ros::Subscriber position_sub = nh.subscribe(uav_name+"/position", 10, position_Callback);
  // ros::Subscriber quantity_sub = nh.subscribe("/quantity", 10, quantity_Callback);
  ros::Subscriber circle_radius_sub = nh.subscribe(uav_name+"/circle_radius", 10, circle_radius_Callback);
  ros::Subscriber setpoint_raw_local_sub =  nh.subscribe(uav_name+"/position_with_yaw", 10, PostionTarget_Callback);


  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name+"/mavros/setpoint_position/local", 10);
  setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name+"/mavros/set_mode");
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name+"/mavros/cmd/arming");

  mavros_msgs::SetMode set_mode;
  mavros_msgs::CommandBool arm_cmd;

  float time = 0.0;

  ros::Rate rate(20.0);
  ros::Time::init();

    while (ros::ok())  
    {
        /* code for loop body */
        switch (mode.data)
        { 

            //切换成offboard模式，且不起飞。
            case 1:
                        target_position.pose.position.z= fcu_position.pose.position.z;
                        target_position.pose.position.y= fcu_position.pose.position.y;
                        target_position.pose.position.x= fcu_position.pose.position.x;
                        target_position.pose.orientation.x= fcu_position.pose.orientation.x;
                        target_position.pose.orientation.y= fcu_position.pose.orientation.y;
                        target_position.pose.orientation.z= fcu_position.pose.orientation.z;
                        target_position.pose.orientation.w= fcu_position.pose.orientation.w; 

                        for(int i = 50; ros::ok() && i > 0; --i){
                            send_pos_setpoint(target_position,getyawfrompose(target_position));
                            ros::spinOnce();
                            rate.sleep(); // 保证更新频率为20Hz，自动调整睡眠时间circle_radius
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

                        mode_last.data=1;
                        break;
        
            //切换成offboard模式后，原地起飞。
            case 2:
                //设置起飞点fcu_position

                        if (mode_last.data!=2)
                        {
                                target_position.pose.position.x = fcu_position.pose.position.x;
                                target_position.pose.position.y = fcu_position.pose.position.y;
                                target_position.pose.position.z = fcu_position.pose.position.z+Takeoff_heiget;  
                                target_position.pose.orientation.x= fcu_position.pose.orientation.x;
                                target_position.pose.orientation.y= fcu_position.pose.orientation.y;
                                target_position.pose.orientation.z= fcu_position.pose.orientation.z;
                                target_position.pose.orientation.w= fcu_position.pose.orientation.w;
                                send_pos_setpoint(target_position,getyawfrompose(target_position));
                                // local_pos_pub.publish(target_position);
                        }
                            // setpoint_raw_local_pub.publish(Target_pos_yaw);
                            send_pos_setpoint(target_position,getyawfrompose(target_position));

                            if(Takeoff_heiget-0.1<fcu_position.pose.position.z<Takeoff_heiget+0.1)
                            {
                            ROS_INFO("Vehicle Takeoff");
                            }

                            mode_last.data=2;
                            break; 

            case 3:
                // fly to the target point 
                        setpoint_raw_local_pub.publish(Target_pos_yaw);
                        break;
            
            case 4:
                //降落
                
                        if (mode_last.data!=4)
                        {
                            /* code for True */
                            target_position.pose.position.x=fcu_position.pose.position.x;
                            target_position.pose.position.y=fcu_position.pose.position.y;
                            target_position.pose.position.z=fcu_position.pose.position.z;
                        }

                        if (fcu_position.pose.position.z>Disaerm_height)
                        {
                            /* code for True */ 
                            target_position.pose.position.z=fcu_position.pose.position.z-0.1;
                            target_position.pose.position.y=fcu_position.pose.position.y;
                            target_position.pose.position.x=fcu_position.pose.position.x;
                            
                            send_pos_setpoint(target_position,getyawfrompose(target_position));
                            ROS_INFO("Landing");
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
                        
                        mode_last.data=4;

                        break;
                  
        }
        ros::spinOnce();
        rate.sleep();
    }
        return 0;
}
