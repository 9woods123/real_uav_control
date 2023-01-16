//
// Created by woods on 22-11-10.
//

#include <mavros2px4ctrl/mavros2px4ctrl.h>
#include <ros/ros.h>


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "mavros2px4_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~"); //  using private node to update parameters.
    mavros2px4 mavros2px4_node(nh, nh_private);
    ros::spin();
    return 0;
}
