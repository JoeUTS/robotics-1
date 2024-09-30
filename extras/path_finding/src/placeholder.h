#ifndef PLACEHOLDER_H
#define PLACEHOLDER_H

#include "pathfinder.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

class placeholder : public rclcpp::Node {
public:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
    rclcpp::TimerBase::SharedPtr timer_;    //!< Timer

    std::unique_ptr<pathfinder> pathfinderPointer_;

    nav_msgs::msg::Odometry odo_;   //!< Odometry message

    placeholder();

    /**
     * @brief ROS2 Callback for map data. Passes to pathfinder
     * 
     * @param msg 
     */
    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg);

    /**
     * @brief 
     * 
     * @param msg 
     */
    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /**
     * @brief ROS2 Callback for timer
     * 
     */
    void timerCallback(void);
};

#endif // PATHFINDER_H