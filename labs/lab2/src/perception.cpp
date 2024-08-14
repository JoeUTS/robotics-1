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
    double minAngle = -M_PI_4;
    double maxAngle = M_PI_4;
    std::vector<double> newRanges;

    // display range of scan
    for (int i = 0; i < laserScan_.ranges.size(); i++) {
        // set invalid ranges to -1 cause I can
        // NaN and inf
        if (std::isnan(laserScan_.ranges.at(i)) || std::isinf(laserScan_.ranges.at(i))) {
            laserScan_.ranges.at(i) = -1;
        }

        // out of range
        if (laserScan_.ranges.at(i) < laserScan_.range_min || laserScan_.ranges.at(i) > laserScan_.range_max) {
            laserScan_.ranges.at(i) = -1;
        }

        double angle = ((i * laserScan_.angle_increment) - laserScan_.angle_min);

        RCLCPP_INFO_STREAM(this->get_logger(), "range[" << angle << " rad]: " << laserScan_.ranges.at(i));

        if (angle >= minAngle && angle <= maxAngle) {
            newRanges.push_back(laserScan_.ranges.at(i));
        }
    }

    // republish data from -pi/4 to pi/4
    std::shared_ptr<sensor_msgs::msg::LaserScan> repub;
}