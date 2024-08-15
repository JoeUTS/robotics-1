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