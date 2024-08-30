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
     * @brief imu callback
     * 
     * @param msg sensor_msgs::msg::Imu
     */
    void imuCallback(const std::shared_ptr<sensor_msgs::msg::Imu> msg);

    /**
     * @brief Get last odometry message
     * 
     * @return nav_msgs::msg::Odometry
     */
    nav_msgs::msg::Odometry getOdo(void);

    /**
     * @brief Get last imu message
     * 
     * @return nsensor_msgs::msg::Imu 
     */
    sensor_msgs::msg::Imu getImu(void);

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
     * @brief drive for a distance in a straight line
     * 
     * @param goalDistance [m]
     * @param velocity [m/s]
     */
    void driveInLine(const double goalDistance, const double goalVelocity);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;  //!< odometry subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;  //!< odometry subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;     //!< velocity publisher

    std::mutex mtx_;    //!< Mutex to allow for future multithreadding. Not Currently required.

    nav_msgs::msg::Odometry odo_;   //!< odometry value
    sensor_msgs::msg::Imu imu_;   //!< odometry value
    geometry_msgs::msg::Twist vel_; //!< velocity command values
};

#endif // SIMPLEDRIVE_H