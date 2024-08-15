#ifndef SIMPLELASER_H
#define SIMPLELASER_H

#include <functional>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class SimpleLaser {

public:
    SimpleLaser(sensor_msgs::msg::LaserScan laserScan);

    void updateScan(sensor_msgs::msg::LaserScan laserScan);

    sensor_msgs::msg::LaserScan getLastMsg(void);

    sensor_msgs::msg::LaserScan laserScan_;      //!< Laser scan data.
    std::mutex mtx_;
};

#endif // SIMPLELASER_H