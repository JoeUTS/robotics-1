#include <rclcpp/rclcpp.hpp>

#include "slam.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slam>());
    rclcpp::shutdown();
    return 0;
}