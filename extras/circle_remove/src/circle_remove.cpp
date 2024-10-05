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
    cv::Mat image1 = img.clone();
    cv::imshow("Image1", image1);
    cv::waitKey(1);

    // image 2
    cv::Mat image2 = img.clone();
    // find circle centers
    // draw green circle at circle center
    cv::imshow("Image2", image2);
    cv::waitKey(1);

    // image 3
    cv::Mat image3 = img.clone();
    // find circle centers
    // draw black circle at circle center to "hide"
    cv::imshow("Image3", image3);
    cv::waitKey(1);
}

