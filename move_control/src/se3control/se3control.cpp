#include <ros/ros.h>
#include <se3control/se3control.h>

namespace  mav_control{

se3control::se3control() {

    double attctrl_tau=0.3;

    controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
    setParameters();

}


void se3control::setProportionParams(float Kp_x, float Kp_y, float Kp_z,
                                     float Kv_x, float Kv_y, float Kv_z) {


    Kpos_x_=Kp_x;
    Kpos_y_=Kp_y;
    Kpos_z_=Kp_z;

    Kvel_x_=Kv_x;
    Kvel_y_=Kv_y;
    Kvel_z_=Kv_z;

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

}

void se3control::setParameters() {


    // default parameters.
    Kpos_x_=8;
    Kpos_y_=8;
    Kpos_z_=15;

    Kvel_x_=1.5;
    Kvel_y_=1.5;
    Kvel_z_=4;

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

    D_<<0,0,0;
    max_fb_acc_=20;

    norm_thrust_const_=0.05;
    norm_thrust_offset_=0.05;

}

void se3control::calculate_control(geometry_msgs::PoseStamped current_pose,geometry_msgs::TwistStamped current_vel,
                                       mav_control::motion_target target,mav_control::control_cmd &output) {

    current_position_<<current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z;
    current_velocity_<<current_vel.twist.linear.x,current_vel.twist.linear.y,current_vel.twist.linear.z;
    current_attitude_(0) = current_pose.pose.orientation.w;
    current_attitude_(1) = current_pose.pose.orientation.x;
    current_attitude_(2) = current_pose.pose.orientation.y;
    current_attitude_(3) = current_pose.pose.orientation.z;

    Eigen::Vector3d desired_acc;
    desired_acc= controlPosition(target);
    //desired_acc=a_des = a_fb + a_ref - a_rd - gravity_;
    //std::cout<<"desired_acc==="<<desired_acc<<std::endl;

    computeBodyRateCmd(cmdBodyRate_, desired_acc, target);

    //std::cout<<"cmdBodyRate_==="<<cmdBodyRate_<<std::endl;

    // calculate the body rate and the thrust;
    // update the output(control msg which will be sent to px4 by mavros topic)
    output.body_rate_cmd=cmdBodyRate_;
    output.attitude=q_des;

}


void se3control::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des,
                                        const mav_control::motion_target &target) {

    q_des = acc2quaternion(a_des, target.yaw);
    // =======woodsTODO====== if we need control roll pitch yaw , maybe we should change the acc2quaternion().

    controller_->Update(current_attitude_, q_des, a_des,
                        Eigen::Vector3d(0,0,0));  // Calculate BodyRate

    bodyrate_cmd.head(3) = controller_->getDesiredRate();
    double thrust_command = controller_->getDesiredThrust().z();

    bodyrate_cmd(3) =std::max(0.0,
                              std::min(1.0,
                                       norm_thrust_const_ * thrust_command +norm_thrust_offset_));

    // thrust_command==a_des * z_B
    // bodyrate_cmd= norm_thrust_const_* a_des * z_B +norm_thrust_offset_
    // why is it not C_cmd= a_des * z_B - k_h* (vx_B + vy_B)^2 ?


}



Eigen::Vector3d se3control::poscontroller(const Eigen::Vector3d &pos_error,
                                          const Eigen::Vector3d &vel_error) {

    Eigen::Vector3d a_fb =
            Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

    if (a_fb.norm() > max_fb_acc_)
        a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

    return a_fb;
}

Eigen::Vector3d se3control::controlPosition(const mav_control::motion_target &target) {

    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target.acceleration;


    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, target.yaw);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    const Eigen::Vector3d pos_error = current_position_ - target.position;
    const Eigen::Vector3d vel_error = current_velocity_ - target.velocity;



    // Position Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target.velocity;  // Rotor drag

    // Reference acceleration
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

    //std::cout<<"a_des==="<<a_des<<std::endl;

    return a_des;

}

Eigen::Vector4d se3control::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {

    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0),
    xb_des(1), yb_des(1), zb_des(1),
    xb_des(2), yb_des(2), zb_des(2);

    quat = rot2Quaternion(rotmat);
    return quat;

}

}

