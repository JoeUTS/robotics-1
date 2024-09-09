#include <rclcpp/rclcpp.hpp>
#include "scan2image.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scan2image>());
    rclcpp::shutdown();
    return 0;
}