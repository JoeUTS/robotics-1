#ifndef SLAM_H
#define SLAM_H

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class slam : public rclcpp::Node {

public:
    slam();

    rclcpp::TimerBase::SharedPtr timer_;     //!< Timer
    

};

#endif // SLAM_H