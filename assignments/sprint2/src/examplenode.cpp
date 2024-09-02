#include "examplenode.h"

ExampleNode::ExampleNode(void) : Node("example_node") {
    // subsribers
    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ExampleNode::laserCallback,this,std::placeholders::_1));
    
    // publishers
    pub_ranges_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("/scanout", 10);
}

void ExampleNode::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    // update laserscan object && make new if doesnt exist.
    if (laserPointer_ == nullptr) {
        laserPointer_ = std::make_unique<SimpleLaser>(*msg);
    } else {
        laserPointer_->updateScan(*msg);
    }

    // republish every 3rd point
    pub_ranges_->publish(laserPointer_->getNthPoint(3));
}
