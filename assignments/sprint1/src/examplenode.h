#ifndef EXAMPLENODE_H
#define EXAMPLENODE_H

#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "simplelaser.h"


class ExampleNode : public rclcpp::Node {

public:
    ExampleNode();

    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    sensor_msgs::msg::LaserScan laserScan_; // laser scan data
    std::unique_ptr<SimpleLaser> laserPointer_;    //!< Pointer to the laser processing object

    unsigned int nCounter_;  // counter to track message number
    const unsigned int N_VAL;   // n value to return scan msg

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;    // laser scan sub
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_ranges_;          // laser scan pub
};

#endif // EXAMPLENODE_H