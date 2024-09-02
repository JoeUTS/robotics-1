/**
 * @file simplelaser.cpp
 * @author Joseph Tarbath (joseph.a.tarbath@hstudent.uts.edu.au)
 * @brief Generic 2D LiDAR template
 * @version 0.1
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "simplelaser.h"

SimpleLaser::SimpleLaser(sensor_msgs::msg::LaserScan laserScan) {
    laserScan_ = laserScan;
}

void SimpleLaser::updateScan(sensor_msgs::msg::LaserScan laserScan) {
    std::unique_lock<std::mutex> lck(mtx_);
    laserScan_ = laserScan;
}

sensor_msgs::msg::LaserScan SimpleLaser::getLastMsg(void) {
    std::unique_lock<std::mutex> lck(mtx_);
    return laserScan_;
}

sensor_msgs::msg::LaserScan SimpleLaser::getNthPoint(const int N) {
    // make local copy of laserScan_
    std::unique_lock<std::mutex> lck(mtx_);
    sensor_msgs::msg::LaserScan newScan = laserScan_;
    lck.unlock();

    // find new set of ranages
    std::vector<float> newRanges;

    for (int i = 0; i < newScan.ranges.size(); i += N) {
        newRanges.push_back(newScan.ranges.at(i));
    }

    // update increment angle
    newScan.angle_increment *= N;

    // update laser message with new ranges
    newScan.ranges.clear();
    newScan.ranges = newRanges;

    return newScan;
}