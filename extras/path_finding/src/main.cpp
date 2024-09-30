#include <rclcpp/rclcpp.hpp>

#include "placeholder.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<placeholder>());
    rclcpp::shutdown();
    return 0;
}