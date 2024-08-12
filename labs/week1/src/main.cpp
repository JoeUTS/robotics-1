#include <rclcpp/rclcpp.hpp>

#include "imageconverter.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<image_converter>());
    rclcpp::shutdown();
    return 0;
}