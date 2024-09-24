#ifndef EXAMPLENODE_H
#define EXAMPLENODE_H

/**
 * @file examplenode.h
 * @author Joseph Tarbath (joseph.a.tarbath@student.uts.edu.au)
 * @brief Generic node template
 * @version 0.1
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "simplelaser.h"


class ExampleNode : public rclcpp::Node {

public:
    /**
     * @brief Construct a new example Node object.
     * 
     */
    ExampleNode(void);

private:
    /**
     * @brief handles ROS laser callback to update laser scan object.
     * 
     * @param msg laser scan message.
     */
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    std::unique_ptr<SimpleLaser> laserPointer_;    //!< Laser scan processing object.

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;    // laser scan subscriber
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_ranges_;          // laser scan publisher
};

#endif // EXAMPLENODE_H