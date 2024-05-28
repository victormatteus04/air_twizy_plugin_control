#ifndef SD_CONTROL__SD_CONTROL_PLUGIN_HPP
#define SD_CONTROL__SD_CONTROL_PLUGIN_HPP

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <sd_msgs/msg/sd_control.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace sd_control
{

  class SdControlPlugin : public gazebo::ModelPlugin
  {
  public:
    SdControlPlugin();

  protected:
      void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
      void Reset();

  private:
    std::string robot_namespace_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::transport::NodePtr gznode_;

    gazebo::physics::LinkPtr chassis_link_;

    gazebo::physics::JointPtr front_left_wheel_joint_;
    gazebo::physics::JointPtr front_right_wheel_joint_;
    gazebo::physics::JointPtr rear_left_wheel_joint_;
    gazebo::physics::JointPtr rear_right_wheel_joint_;
    gazebo::physics::JointPtr front_left_wheel_steering_joint_;
    gazebo::physics::JointPtr front_right_wheel_steering_joint_;

    gazebo::common::PID front_left_wheel_steering_pid_;
    gazebo::common::PID front_right_wheel_steering_pid_;

    double front_left_wheel_radius_;
    double front_right_wheel_radius_;
    double rear_left_wheel_radius_;
    double rear_right_wheel_radius_;

    double front_track_width_;
    double back_track_width_;
    double wheel_base_length_;

    double max_steer_;
    double max_speed_;
    double max_torque_;
    double front_brake_torque_;
    double back_brake_torque_;
    double chassis_aero_force_gain_;

    rclcpp::Subscription<sd_msgs::msg::SDControl>::SharedPtr control_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    gazebo::common::Time last_sim_time_;

    sd_msgs::msg::SDControl control_cmd_;

    std::mutex mutex_;

    gazebo::event::ConnectionPtr update_connection_;
    void Update();

    void publishOdometry();

    void controlCallback(const sd_msgs::msg::SDControl & msg);

    double collisionRadius(gazebo::physics::CollisionPtr coll) const;
  };

}

#endif
