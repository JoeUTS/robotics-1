#ifndef PLACEHOLDER_H
#define PLACEHOLDER_H

#include "pathfinder.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class placeholder : public rclcpp::Node {
public:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    placeholder();

    /**
     * @brief ROS2 Callback for map data. Passes to pathfinder
     * 
     * @param msg 
     */
    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg);

    std::unique_ptr<pathfinder> pathfinderPointer_;
};

#endif // PATHFINDER_H