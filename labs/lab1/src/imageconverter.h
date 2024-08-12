#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class image_converter : public rclcpp::Node {

public:
    image_converter();

    void imageCallback(const std::shared_ptr<sensor_msgs::msg::Image> msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImage_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage_;
};

#endif // IMAGECONVERTER_H