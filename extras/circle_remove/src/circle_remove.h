#ifndef CIRCLEREMOVE_H
#define CIRCLEREMOVE_H

#include "circle_remove.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


class circle_remove : public rclcpp::Node {
public:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;    //!< Laser scan sub

    sensor_msgs::msg::LaserScan laserScan_;  //!< Laser scan data

    circle_remove(void);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
};

#endif // CIRCLEREMOVE_H