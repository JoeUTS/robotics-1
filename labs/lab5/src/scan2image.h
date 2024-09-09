#ifndef SCAN2IMAGE_H
#define SCAN2IMAGE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class scan2image : public rclcpp::Node {
    
public:
    scan2image(void);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    void calculateYawChange(void);

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_;
    bool second_image_captured_;

    double angle_difference_;
    double relative_orientaion_;
};

#endif // SCAN2IMAGE_H