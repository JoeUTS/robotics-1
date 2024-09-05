#ifndef SIMPLEDRIVE_H
#define SIMPLEDRIVE_H

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>


class simpledrive: public rclcpp::Node {

public:
    simpledrive();

    /**
     * @brief odometry callback
     * 
     * @param msg nav_msgs::msg::Odometry
     */
    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /**
     * @brief Get last odometry message
     * 
     * @return nav_msgs::msg::Odometry
     */
    nav_msgs::msg::Odometry getOdo(void);

    /**
     * @brief timer function
     * 
     */
    void timerCallback(void);

    /**
     * @brief 6DOF publisher
     * 
     * @param x_lin [m/s]
     * @param y_lin [m/s]
     * @param z_lin [m/s]
     * @param roll  [rad/s]
     * @param pitch [rad/s]
     * @param yaw   [rad/s]
     */
    void diveCMD(const double x_lin, const double y_lin, const double z_lin, const double roll, const double pitch, const double yaw);

    /**
     * @brief 2DOF velocity publisher
     * 
     * @param x_lin [m/s]
     * @param yaw   [rad/s]
     */
    void diveCMD(const double x_lin, const double yaw);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;  //!< odometry subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;     //!< velocity publisher
    rclcpp::TimerBase::SharedPtr timer_; //!< timer

    std::mutex mtx_;    //!< Mutex to allow for future multithreadding. Not Currently required.

    nav_msgs::msg::Odometry odo_;   //!< odometry value
    sensor_msgs::msg::Imu imu_;   //!< odometry value
    geometry_msgs::msg::Twist vel_; //!< velocity command values
};

#endif // SIMPLEDRIVE_H