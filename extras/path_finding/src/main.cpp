#include <rclcpp/rclcpp.hpp>

#include "pathfinder.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pathfinder>());
    rclcpp::shutdown();
    return 0;
}