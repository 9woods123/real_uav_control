
#ifndef SE3CONTROL
#define SE3CONTROL

#include <ros/ros.h>
#include <Eigen/Dense>
#include <se3control/common.h>
#include <se3control/control.h>
#include <se3control/nonlinear_attitude_control.h>
#include <se3control/nonlinear_geometric_control.h>
#include <se3control/jerk_tracking_control.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

namespace mav_control{





struct motion_target{
    Eigen::Vector3d position,velocity,acceleration;
    float yaw;
};

struct control_cmd{
    Eigen::Vector4d body_rate_cmd, attitude;
};



class se3control
{
private:
    Eigen::Vector3d target_pos_,target_vel_,target_acc_;
    Eigen::Vector3d current_position_, current_velocity_;

    Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.8)};
    Eigen::Vector4d current_attitude_, q_des;
    Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}

    Eigen::Vector3d Kpos_, Kvel_, D_;

    std::shared_ptr<Control> controller_;

    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;

    bool velocity_yaw_=false;
    double max_fb_acc_=10;
    float  norm_thrust_const_=0.05;
    float  norm_thrust_offset_=0.1;



public:

    se3control();
    void setParameters();
    void setProportionParams(float Kp_x,float Kp_y,float Kp_z,float Kv_x,float Kv_y,float Kv_z);
    void setThrustConst(float norm_thrust_const);// TODO =======WOODS======

    void calculate_control(geometry_msgs::PoseStamped current_pose,
                           geometry_msgs::TwistStamped current_vel,
                           motion_target target, control_cmd &output);

    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des,const motion_target &target);

    Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos,
                                    const Eigen::Vector3d &target_vel,
                                    const Eigen::Vector3d &target_acc);

    Eigen::Vector3d controlPosition(const motion_target &target);

    void UAVstatesCallback(Eigen::Vector3d mavPos, Eigen::Vector3d mavVel, Eigen::Vector3d mavRate);

    Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error,const Eigen::Vector3d &vel_error);
    static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

    static double getVelocityYaw(const Eigen::Vector3d velocity) {
        return atan2(velocity(1), velocity(0)); };
};

}
#endif //SE3CONTROL
