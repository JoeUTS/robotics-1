#include "simpledrive.h"

simpledrive::simpledrive() : Node("simpledrive") {
    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&simpledrive::odoCallback,this,std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&simpledrive::imuCallback,this,std::placeholders::_1));

    pub_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    driveInLine(1,1);
}

void simpledrive::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Odo updated");
    odo_ = *msg;
}

void simpledrive::imuCallback(const std::shared_ptr<sensor_msgs::msg::Imu> msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Imu updated");
    imu_ = *msg;
}

nav_msgs::msg::Odometry simpledrive::getOdo(void) {
    std::unique_lock<std::mutex> lck(mtx_);
    return odo_;
}

sensor_msgs::msg::Imu simpledrive::getImu(void) {
    std::unique_lock<std::mutex> lck(mtx_);
    return imu_;
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
    
    pub_vel_->publish(cmd);
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

    pub_vel_->publish(cmd);
}

int simpledrive::calculateDuration(std::chrono::time_point<std::chrono::system_clock> startTime) {
    std::chrono::time_point<std::chrono::system_clock> endTime = std::chrono::system_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    return duration;
}

void simpledrive::driveInLine(const double goalDistance, const double goalVelocity) {
    double drivenDistance = 0;
    double velocity = 0;    // we probs need this as a member variable

    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();

    while (drivenDistance < goalDistance) {
        //RCLCPP_INFO_STREAM(this->get_logger(), "distance [" << drivenDistance << " / " << goalDistance);
        //RCLCPP_INFO_STREAM(this->get_logger(), "drivenDistance: " << drivenDistance);

        // calculate driven distance
        // using IMU cause /odom not updating
        double acceleration = getImu().linear_acceleration.x;
        int duration = calculateDuration(startTime);
        velocity += acceleration * duration;
        drivenDistance += duration * velocity;

        // send command
        diveCMD(goalVelocity, 0);

        startTime = std::chrono::system_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    diveCMD(0, 0);
}