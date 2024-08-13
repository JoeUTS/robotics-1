#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class perception : public rclcpp::Node {

public:
    perception();

    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    void laserProcessing();

    sensor_msgs::msg::LaserScan laserScan_; // laser scan data

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;    // laser scan sub
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_newlaser_;        // laser scan pub
};

#endif // PERCEPTION_H