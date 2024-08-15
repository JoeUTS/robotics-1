#include "examplenode.h"

ExampleNode::ExampleNode() : Node("example_node"), N_VAL(5) {
    nCounter_ = 0;

    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ExampleNode::laserCallback,this,std::placeholders::_1));
    
    pub_ranges_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("/scanout", 10);
}

void ExampleNode::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    if (laserPointer_ == nullptr) {
        laserPointer_ = std::make_unique<SimpleLaser>(*msg);
    } else {
        laserPointer_->updateScan(*msg);
    }

    nCounter_++;

    if (nCounter_ == N_VAL) {
        pub_ranges_->publish(laserPointer_->getLastMsg());
        nCounter_ = 0;
    }
}
