#include <rclcpp/rclcpp.hpp>

#include "perception.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perception>());
    rclcpp::shutdown();
    return 0;
}