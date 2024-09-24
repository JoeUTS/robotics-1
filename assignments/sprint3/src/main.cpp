/**
 * @file main.cpp
 * @author Joseph Tarbath (joseph.a.tarbath@student.uts.edu.au)
 * @brief Main file for example node template
 * @version 0.1
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <rclcpp/rclcpp.hpp>

#include "examplenode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>());
    rclcpp::shutdown();
    return 0;
}