#include "placeholder.h"

placeholder::placeholder() : Node("pathfinder_placeholder") {

    // subscribe to map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&placeholder::mapCallback, this, std::placeholders::_1));
}

void placeholder::mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg) {
    if (pathfinderPointer_ = nullptr) {
        pathfinderPointer_ = std::make_unique<pathfinder>(*msg);
    } else {
        pathfinderPointer_->updateMap(*msg);
    }
}