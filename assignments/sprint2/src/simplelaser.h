#ifndef SIMPLELASER_H
#define SIMPLELASER_H

/**
 * @file simplelaser.h
 * @author Joseph Tarbath (joseph.a.tarbath@student.uts.edu.au)
 * @brief Generic 2D LiDAR template
 * @version 0.1
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <functional>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief Generic 2D LiDAR template
 * 
 */
class SimpleLaser {

public:
    /**
     * @brief Construct a new Simple Laser object with laserScan_ initialised.
     * 
     * @param laserScan laserScan_ initilaise message.
     */
    SimpleLaser(sensor_msgs::msg::LaserScan laserScan);

    /**
     * @brief update laserScan_ with new scan data.
     * 
     * @param laserScan new scan data.
     */
    void updateScan(sensor_msgs::msg::LaserScan laserScan);

    /**
     * @brief Get laserScan_;
     * 
     * @return sensor_msgs::msg::LaserScan laserScan_;
     */
    sensor_msgs::msg::LaserScan getLastMsg(void);

    /**
     * @brief returns last scan message with scan data of every Nth point.
     * 
     * @param N
     */
    sensor_msgs::msg::LaserScan getNthPoint(const int N);

    sensor_msgs::msg::LaserScan laserScan_;      //!< Laser scan data.

    std::mutex mtx_;    //!< Mutex to allow for future multithreadding. Not Currently required.
};

#endif // SIMPLELASER_H