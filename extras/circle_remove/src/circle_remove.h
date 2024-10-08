#ifndef CIRCLEREMOVE_H
#define CIRCLEREMOVE_H

#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class circle_remove : public rclcpp::Node {
public:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;             //!< Map sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;                  //!< Odometry sub
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;        //!< Laser scan sub
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;             //!< Map sub
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;    //!< Navigation client
   
    rclcpp::TimerBase::SharedPtr timer_;    //!< Timer

    nav_msgs::msg::OccupancyGrid map_;      //!< Map data
    nav_msgs::msg::Odometry odo_;           //!< Odometry
    sensor_msgs::msg::LaserScan laserScan_; //!< Laser scan data
    geometry_msgs::msg::Pose goalPose_;     //!< Goal pose
    geometry_msgs::msg::Pose startPose_;    //!< Start pose

    bool mapSet_;                       //!< Map set flag
    bool goalSet_;                      //!< Goal set flag
    cv::Mat mapImage_;                  //!< Map image
    

    std::vector<cv::Point> objects_;    //!< Holds object centers

    circle_remove(void);

    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg);

    void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    void goalCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);

    void sendGoal(const geometry_msgs::msg::Pose &goal);

    void showImages(void);

    cv::Mat mapToMat(const nav_msgs::msg::OccupancyGrid &map);

    cv::Point findCenter(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3);

    std::vector<cv::Point> laser2Points(const sensor_msgs::msg::LaserScan &laserScan);

    std::vector<cv::Point> objectDetect(const sensor_msgs::msg::LaserScan &laserScan);

    cv::Point worldPoseToMap(const geometry_msgs::msg::Pose &pose, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief ROS2 Callback for timer
     * 
     */
    void timerCallback(void);
};

#endif // CIRCLEREMOVE_H