#ifndef SD_CONTROL__SD_TELEOP_JOY_HPP_
#define SD_CONTROL__SD_TELEOP_JOY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sd_msgs/msg/sd_control.hpp> 

namespace sd_control
{

  class SdTeleopJoy : public rclcpp::Node
  {
  public:
    SdTeleopJoy(const std::string & name, const std::string & namespace_ = "");

  private:
    int enable_button_;
    int throttle_axis_;
    int steer_axis_;

    std::map<std::string, double> scale_throttle_map_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sd_msgs::msg::SDControl>::SharedPtr control_pub_;
    
    void joyCallback(sensor_msgs::msg::Joy::SharedPtr joy);
  };
}

#endif  // SD_CONTROL__SD_TELEOP_JOY_HPP_
