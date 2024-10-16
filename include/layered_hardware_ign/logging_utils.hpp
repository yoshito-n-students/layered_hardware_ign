#ifndef LAYERED_HARDWARE_IGN_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_IGN_LOGGING_UTILS_HPP

#include <rclcpp/logging.hpp>

#define LHI_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("layered_hardware_ign"), __VA_ARGS__)
#define LHI_INFO(...) RCLCPP_INFO(rclcpp::get_logger("layered_hardware_ign"), __VA_ARGS__)
#define LHI_WARN(...) RCLCPP_WARN(rclcpp::get_logger("layered_hardware_ign"), __VA_ARGS__)
#define LHI_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("layered_hardware_ign"), __VA_ARGS__)
#define LHI_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("layered_hardware_ign"), __VA_ARGS__)

#endif