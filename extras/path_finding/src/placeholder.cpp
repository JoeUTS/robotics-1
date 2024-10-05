#include "placeholder.h"

placeholder::placeholder() : Node("pathfinder_placeholder") {
    startSet_ = false;
    goalSet_ = false;
    // subscribe to map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&placeholder::mapCallback, this, std::placeholders::_1));

    // subscribe to odometry
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&placeholder::odoCallback,this,std::placeholders::_1));

    // subscribe to start and end pose topics
    start_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/start_pose", 10, std::bind(&placeholder::startCallback,this,std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/goal_pose", 10, std::bind(&placeholder::goalCallback,this,std::placeholders::_1));

    // timer
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

void placeholder::startCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg) {
    startPose_ = *msg;

    if (!startSet_) {
        startSet_ = true;
    }
}

void placeholder::goalCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg) {
    goalPose_ = *msg;

    if (!goalSet_) {
        goalSet_ = true;
    }
}

void placeholder::timerCallback(void) {
    // check if map and points are set
    if (pathfinderPointer_ != nullptr && startSet_ && goalSet_) {
        // get start and goal pose
        geometry_msgs::msg::Pose startPose = startPose_;
        geometry_msgs::msg::Pose goalPose = goalPose_;
        
        // run
        RCLCPP_INFO_STREAM(this->get_logger(), "finding path from (" << startPose.position.x << ", " << startPose.position.y << ") to (" << goalPose.position.x << ", " << goalPose.position.y << ")");
        geometry_msgs::msg::PoseArray path = pathfinderPointer_->AStar(startPose, goalPose);
        pathfinderPointer_->publishMarkers(path);
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Initilizing pathfinder. Map[" << (pathfinderPointer_ != nullptr) << "] Start[" << startSet_ << "] Goal[" << goalSet_ << "]" );
    }
}