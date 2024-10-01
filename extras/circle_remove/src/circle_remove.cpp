#include "circle_remove.h"

circle_remove::circle_remove() : Node("circle_remove") {
    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&circle_remove::scanCallback, this, std::placeholders::_1));
}

void circle_remove::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laserScan_ = *msg;
}

void circle_remove::showImages(void) {
    cv::Mat img = laserScanToMat(laserScan_);

    // image 1
    cv::Mat firstImage = img.clone();
    cv::imshow("Image1", firstImage);
}

