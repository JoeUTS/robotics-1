#ifndef SIMPLEDRIVE_H
#define SIMPLEDRIVE_H

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>


class simpledrive: public rclcpp::Node {

public:
    simpledrive();

    /**
     * @brief odometry callback
     * 
     * @param msg nav_msgs::msg::Odometry pointer
     */
    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /**
     * @brief sets velocitys
     * 
     * @param xVel  Linear velocity [m/s]
     * @param yaw   Angular velocity [rad/s]
     */
    void setVelocity(const double xVel, const double yaw);

    /**
     * @brief Sets the starting timer
     * 
     */
    void timerStart(void);

    /**
     * @brief returns time passed in ms
     * 
     * @param startTime 
     * @return double duration [ms]
     */
    int calculateDuration(std::chrono::time_point<std::chrono::system_clock> startTime);

    /**
     * @brief drive in a square based on dead reckoning
     * 
     * @param edgeLength [m]
     */
    void driveInLine(const double distance, const double velocity);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;  //!< odometry subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;     //!< velocity publisher

    std::mutex mtx_;    //!< Mutex to allow for future multithreadding. Not Currently required.

    nav_msgs::msg::Odometry odo_;   //!< odometry value
    geometry_msgs::msg::Twist vel_; //!< velocity values
};

#endif // SIMPLEDRIVE_H