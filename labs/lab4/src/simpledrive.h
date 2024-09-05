#ifndef SIMPLEDRIVE_H
#define SIMPLEDRIVE_H

#include <chrono>
#include <thread>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
     * @brief timer function
     * 
     */
    void timerCallback(void);

    
    double getNoise(const double distLower, const double distUpper);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;      //!< odometry subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;   //!< velocity publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_noisey_odo_;  //!< noisey odometry publisher
    rclcpp::TimerBase::SharedPtr timer_;                                    //!< timer

    std::mutex mtx_;    //!< Mutex to allow for future multithreadding. Not Currently required.
    std::uniform_real_distribution<double> noiseDist_;  //!< noise

    const double GOAL_DISTANCE; //!< distance to drive
    const double NOISE_MAX;     //!< max noise value

    nav_msgs::msg::Odometry odo_;       //!< odometry value
    nav_msgs::msg::Odometry prevOdo_;   //!< previous odometry value
    geometry_msgs::msg::Twist vel_;     //!< velocity values
    rclcpp::Time lastTime_; //!< previous timestamp for velocity calulation
    double totalDistance_;  //!< total distance travelled
    
};

#endif // SIMPLEDRIVE_H