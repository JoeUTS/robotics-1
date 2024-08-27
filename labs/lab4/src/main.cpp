#include <rclcpp/rclcpp.hpp>

#include "simpledrive.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simpledrive>());
    rclcpp::shutdown();
    return 0;
}