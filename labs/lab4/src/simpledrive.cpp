#include "simpledrive.h"

simpledrive::simpledrive() : Node("simpledrive") {
    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&simpledrive::odoCallback,this,std::placeholders::_1));

    pub_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&simpledrive::timerCallback, this));
}

void simpledrive::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Odo updated");
    odo_ = *msg;
}

nav_msgs::msg::Odometry simpledrive::getOdo(void) {
    std::unique_lock<std::mutex> lck(mtx_);
    return odo_;
}

void simpledrive::timerCallback(void) {
    /*
    if (not enough ono data) {
        return;
    }
    */


}

void simpledrive::diveCMD(const double x_lin, const double y_lin, const double z_lin, const double roll, const double pitch, const double yaw) {
    geometry_msgs::msg::Twist cmd;

    // linear
    cmd.linear.x = x_lin;
    cmd.linear.y = y_lin;
    cmd.linear.z = z_lin;

    //angular
    cmd.angular.x = roll;
    cmd.angular.y = pitch;
    cmd.angular.z = yaw;
    
    pub_cmd_vel_->publish(cmd);
}

void simpledrive::diveCMD(const double x_lin, const double yaw) {
    geometry_msgs::msg::Twist cmd;

    // linear
    cmd.linear.x = x_lin;
    cmd.linear.y = 0;
    cmd.linear.z = 0;

    //angular
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = yaw;

    pub_cmd_vel_->publish(cmd);
}
