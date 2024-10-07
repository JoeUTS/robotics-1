#ifndef CIRCLEREMOVE_H
#define CIRCLEREMOVE_H

#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class circle_remove : public rclcpp::Node {
public:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;    //!< Laser scan sub

    rclcpp::TimerBase::SharedPtr timer_;     //!< Timer

    nav_msgs::msg::OccupancyGrid map_;    //!< Map data
    nav_msgs::msg::Odometry odo_;           //!< Odometry
    sensor_msgs::msg::LaserScan laserScan_;  //!< Laser scan data

    bool mapSet;
    cv::Mat mapImage_;
    std::vector<cv::Point> objects_;

    circle_remove(void);

    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    void showImages(void);

    cv::Mat mapToMat(const nav_msgs::msg::OccupancyGrid &map);

    cv::Point findCenter(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3);

    std::vector<cv::Point> laser2Points(const sensor_msgs::msg::LaserScan &laserScan);

    std::vector<cv::Point> objectDetect(const sensor_msgs::msg::LaserScan &laserScan);

    /**
     * @brief ROS2 Callback for timer
     * 
     */
    void timerCallback(void);
};

#endif // CIRCLEREMOVE_H