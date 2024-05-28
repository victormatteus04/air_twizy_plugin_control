#include "sd_control/sd_teleop_joy.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto joy_teleop = std::make_shared<sd_control::SdTeleopJoy>("sd_teleop_joy_node");

  rclcpp::spin(joy_teleop);

  rclcpp::shutdown();

  return 0;
}