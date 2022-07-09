#include <ros/ros.h>  
#include <sensor_msgs/image_encodings.h> 
#include "vrpn_client_ros/vrpn_client_ros.h"
#include <geometry_msgs/PoseStamped.h>

ros::Subscriber sub;
ros::Publisher pub;
geometry_msgs::PoseStamped uav1_psoe;

std::string uav_name;

using namespace std;

void messageCallback(const geometry_msgs::PoseStamped &msg)
{
  
    uav1_psoe.header.stamp = ros::Time::now();
    uav1_psoe.pose.position.x=msg.pose.position.x;
    uav1_psoe.pose.position.y=msg.pose.position.y;
    uav1_psoe.pose.position.z=msg.pose.position.z;
    uav1_psoe.pose.orientation.w=msg.pose.orientation.w;
    uav1_psoe.pose.orientation.x=msg.pose.orientation.x;
    uav1_psoe.pose.orientation.y=msg.pose.orientation.y;
    uav1_psoe.pose.orientation.z=msg.pose.orientation.z;
    cout<<uav1_psoe.pose.position.x<<endl;
    pub.publish(uav1_psoe);
}

int main(int argc, char **argv)
{   
ros::init(argc, argv, "vrpn_to_mavros");
ros::NodeHandle nh("~");

nh.param<string>("uav_name",uav_name,"/uav0");

pub=nh.advertise<geometry_msgs::PoseStamped>(uav_name+"/mavros/vision_pose/pose",100);
ros::Subscriber sub = nh.subscribe("/vrpn_client_node"+uav_name+"/pose", 1, messageCallback);


ros::spin();
}
