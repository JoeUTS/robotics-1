#include <rclcpp/rclcpp.hpp>

#include "circle_remove.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<circle_remove>());
    rclcpp::shutdown();
    return 0;
}