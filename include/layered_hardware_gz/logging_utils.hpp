#ifndef LAYERED_HARDWARE_GZ_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_GZ_LOGGING_UTILS_HPP

#include <layered_hardware/logging_utils.hpp>
#include <layered_hardware_gz/common_namespaces.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace layered_hardware_gz {

// returns reference to the common logger without construction overhead
static inline rclcpp::Logger &get_lhg_logger() {
  static rclcpp::Logger logger = rclcpp::get_logger("layered_hardware_gz");
  return logger;
}

// logging functions which supports cpp-string arguments

template <typename... Args> static inline void lhg_debug(const char *format, Args &&...args) {
  RCLCPP_DEBUG(get_lhg_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhg_info(const char *format, Args &&...args) {
  RCLCPP_INFO(get_lhg_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhg_warn(const char *format, Args &&...args) {
  RCLCPP_WARN(get_lhg_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhg_error(const char *format, Args &&...args) {
  RCLCPP_ERROR(get_lhg_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhg_fatal(const char *format, Args &&...args) {
  RCLCPP_FATAL(get_lhg_logger(), format, lh::to_format_arg(args)...);
}

} // namespace layered_hardware_gz

#endif