//
// Created by woods on 22-11-10.
//

#include <move_control/mavros_px4ctrl.h>
#include <ros/ros.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "mavros2px4_node");
    ros::NodeHandle nh("~");  //  using private node here for convenient parameters setting
    ros::NodeHandle nh_private("~");
    mavros2px4 mavros2px4_node(nh,nh_private);
    ros::spin();
    return 0;
}
