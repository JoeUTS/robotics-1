#include "placeholder.h"

placeholder::placeholder() : Node("pathfinder_placeholder") {
    // subscribe to map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&placeholder::mapCallback, this, std::placeholders::_1));

    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&placeholder::odoCallback,this,std::placeholders::_1));

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&placeholder::timerCallback, this));
}

void placeholder::mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg) {
    if (pathfinderPointer_ == nullptr) {
        pathfinderPointer_ = std::make_unique<pathfinder>(*msg);
    } else {
        pathfinderPointer_->updateMap(*msg);
    }
}

void placeholder::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    odo_ = *msg;
}

void placeholder::timerCallback(void) {
    geometry_msgs::msg::Pose goalPose;

    goalPose.position.x = odo_.pose.pose.position.x;
    goalPose.position.y = odo_.pose.pose.position.y - 1;


    // dont run if pathfinder pointer is null
    if (pathfinderPointer_ != nullptr) {
        pathfinderPointer_->AStar(odo_.pose.pose, goalPose);
    }
}