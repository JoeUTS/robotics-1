#include "simpledrive.h"

simpledrive::simpledrive() : Node("simpledrive") {
    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel/odometry", 10, std::bind(&simpledrive::odoCallback,this,std::placeholders::_1));

    pub_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    driveInLine(1,1);
}

void simpledrive::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    odo_ = *msg;
}

void simpledrive::setVelocity(const double xVel, const double yaw) {
    std::unique_lock<std::mutex> lck(mtx_);
    // linear
    vel_.linear.x = xVel;
    vel_.linear.y = 0;
    vel_.linear.z = 0;

    //angular
    vel_.angular.x = 0;
    vel_.angular.y = 0;
    vel_.angular.z = yaw;

    pub_vel_->publish(vel_);
}

int simpledrive::calculateDuration(std::chrono::time_point<std::chrono::system_clock> startTime) {
    auto endTime = std::chrono::system_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    return duration;
}

void simpledrive::driveInLine(const double distance, const double velocity) {
    double drivenDistance = 0;

    setVelocity(velocity, 0);
    auto startTime = std::chrono::system_clock::now();

    while (drivenDistance < distance) {
        int duration = calculateDuration(startTime);
        drivenDistance += duration * odo_.twist.twist.linear.x;
        startTime = std::chrono::system_clock::now();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    setVelocity(0, 0);
}