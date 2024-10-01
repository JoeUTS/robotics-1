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
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;

    rclcpp::TimerBase::SharedPtr timer_;    //!< Timer

    std::unique_ptr<pathfinder> pathfinderPointer_;

    nav_msgs::msg::Odometry odo_;           //!< Odometry
    geometry_msgs::msg::Pose startPose_;    //!< Start Pose
    geometry_msgs::msg::Pose goalPose_;     //!< Goal Pose

    bool startSet_; //!< starting pose flag
    bool goalSet_;  //!< goal pose flag

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
     * @brief ROS2 Callback to set starting pose
     * 
     * @param msg 
     */
    void startCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);

    /**
     * @brief ROS2 Callback to set goal pose
     * 
     * @param msg 
     */
    void goalCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);

    /**
     * @brief ROS2 Callback for timer
     * 
     */
    void timerCallback(void);
};

#endif // PATHFINDER_H