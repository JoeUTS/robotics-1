#include "perception.h"

perception::perception() : Node("perception") {
    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&perception::laserCallback,this,std::placeholders::_1));
    
    pub_newlaser_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("/newlaserscan", 10);
}

void perception::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    laserScan_ = *msg;
    laserProcessing();
}

void perception::laserProcessing() {
    // convert vector index to angle
    // display range of scan
    for (int i = 0; i < laserScan_.ranges.size(); i++) {
        double angle = ((i * laserScan_.angle_increment) - laserScan_.angle_min) * 180 / M_PI;
        RCLCPP_INFO_STREAM(this->get_logger(), "range[" << angle << "°]: " << laserScan_.ranges.at(i));
    }

    // republish data from -pi/4 to pi/4
    std::shared_ptr<sensor_msgs::msg::LaserScan> repub; // finish this by setting it on call
}