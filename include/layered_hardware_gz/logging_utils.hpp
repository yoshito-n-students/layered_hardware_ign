#ifndef LAYERED_HARDWARE_GZ_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_GZ_LOGGING_UTILS_HPP

#include <rclcpp/logging.hpp>

#define LHG_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("layered_hardware_gz"), __VA_ARGS__)
#define LHG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("layered_hardware_gz"), __VA_ARGS__)
#define LHG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("layered_hardware_gz"), __VA_ARGS__)
#define LHG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("layered_hardware_gz"), __VA_ARGS__)
#define LHG_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("layered_hardware_gz"), __VA_ARGS__)

#endif