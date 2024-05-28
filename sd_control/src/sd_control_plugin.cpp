#include "sd_control/sd_control_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <sd_msgs/msg/sd_control.hpp>
// #include "sd_control_plugin.hpp"

namespace sd_control
{

  SdControlPlugin::SdControlPlugin()
    : robot_namespace_{""},
      model_{nullptr},
      world_{nullptr},
      gznode_{nullptr},
      chassis_link_{nullptr},
      front_left_wheel_joint_{nullptr},
      front_right_wheel_joint_{nullptr},
      rear_left_wheel_joint_{nullptr},
      rear_right_wheel_joint_{nullptr},
      front_left_wheel_steering_joint_{nullptr},
      front_right_wheel_steering_joint_{nullptr},
      front_left_wheel_steering_pid_{},
      front_right_wheel_steering_pid_{},
      front_left_wheel_radius_{0},
      front_right_wheel_radius_{0},
      rear_left_wheel_radius_{0},
      rear_right_wheel_radius_{0},
      front_track_width_{0},
      back_track_width_{0},
      wheel_base_length_{0},
      max_steer_{0},
      max_speed_{0},
      max_torque_{0},
      front_brake_torque_{0},
      back_brake_torque_{0},
      chassis_aero_force_gain_{0},
      control_sub_{nullptr},
      odometry_pub_{nullptr},
      last_sim_time_{0},
      control_cmd_{}
  {
  }

  void SdControlPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
      try {
          if (!model || !sdf)
          {
              gzerr << "Model or SDF element is null. Plugin will not run." << std::endl;
              return;
          }

          model_ = model;
          world_ = model_->GetWorld();
          gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
          gznode_->Init();

          if (sdf->HasElement("robotNamespace"))
          {
              robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
          }

        // Ensure robot_namespace_ is not empty
          if (robot_namespace_.empty()) {
              robot_namespace_ = "sd_twizy";
          }
          
          rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(robot_namespace_);
          RCLCPP_INFO(node->get_logger(), "Loading plugin!");

          control_sub_ = node->create_subscription<sd_msgs::msg::SDControl>(
              "sd_control",
              10,
              std::bind(&SdControlPlugin::controlCallback, this, std::placeholders::_1));

          odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

          // Find joints and links
          auto findLink = [&](std::string const &link_name) {
              auto full_link_name = model_->GetName() + "::" + sdf->Get<std::string>(link_name);
              auto link = model_->GetLink(full_link_name);
              if (!link)
              {
                  gzerr << "Could not find link: " << full_link_name << "\n";
              }
              return link;
          };

          auto findJoint = [&](std::string const &joint_name) {
              auto full_joint_name = model_->GetName() + "::" + sdf->Get<std::string>(joint_name);
              auto joint = model_->GetJoint(full_joint_name);
              if (!joint)
              {
                  gzerr << "Could not find joint: " << full_joint_name << "\n";
              }
              return joint;
          };

          gzdbg << "Finding chassis link..." << std::endl;
          chassis_link_ = findLink("chassis");

          gzdbg << "Finding wheel and steering joints..." << std::endl;
          front_left_wheel_joint_ = findJoint("front_left_wheel");
          front_right_wheel_joint_ = findJoint("front_right_wheel");
          rear_left_wheel_joint_ = findJoint("rear_left_wheel");
          rear_right_wheel_joint_ = findJoint("rear_right_wheel");
          front_left_wheel_steering_joint_ = findJoint("front_left_wheel_steering");
          front_right_wheel_steering_joint_ = findJoint("front_right_wheel_steering");

          if (!chassis_link_ || !front_left_wheel_joint_ || !front_right_wheel_joint_ ||
              !rear_left_wheel_joint_ || !rear_right_wheel_joint_ ||
              !front_left_wheel_steering_joint_ || !front_right_wheel_steering_joint_)
          {
              gzerr << "Failed to find one or more required joints or links. Plugin will not run." << std::endl;
              return;
          }

          // Read parameters
          auto findParameter = [&](std::string const &param_name, double default_value) {
              if (sdf->HasElement(param_name))
              {
                  return sdf->Get<double>(param_name);
              }
              else
              {
                  gzmsg << "Parameter [" << param_name << "] not found, using default value: " << default_value << std::endl;
                  return default_value;
              }
          };

          gzdbg << "Reading parameters..." << std::endl;
          max_steer_ = findParameter("max_steer", 0.785398);
          max_speed_ = findParameter("max_speed", 10.0);
          max_torque_ = findParameter("max_torque", 57.0);
          front_brake_torque_ = findParameter("front_brake_torque", 500.0);
          back_brake_torque_ = findParameter("back_brake_torque", 500.0);
          chassis_aero_force_gain_ = findParameter("chassis_aero_force_gain", 1.0);

          front_left_wheel_steering_pid_.SetPGain(findParameter("front_left_steering_p_gain", 0.0));
          front_left_wheel_steering_pid_.SetIGain(findParameter("front_left_steering_i_gain", 0.0));
          front_left_wheel_steering_pid_.SetDGain(findParameter("front_left_steering_d_gain", 0.0));

          front_right_wheel_steering_pid_.SetPGain(findParameter("front_right_steering_p_gain", 0.0));
          front_right_wheel_steering_pid_.SetIGain(findParameter("front_right_steering_i_gain", 0.0));
          front_right_wheel_steering_pid_.SetDGain(findParameter("front_right_steering_d_gain", 0.0));

          front_left_wheel_steering_pid_.SetCmdMin(-5000);
          front_left_wheel_steering_pid_.SetCmdMax(5000);
          front_right_wheel_steering_pid_.SetCmdMin(-5000);
          front_right_wheel_steering_pid_.SetCmdMax(5000);

          // Determine physical properties
          auto id = unsigned{0};
          gzdbg << "Determining physical properties..." << std::endl;
          front_left_wheel_radius_ = collisionRadius(front_left_wheel_joint_->GetChild()->GetCollision(id));
          front_right_wheel_radius_ = collisionRadius(front_right_wheel_joint_->GetChild()->GetCollision(id));
          rear_left_wheel_radius_ = collisionRadius(rear_left_wheel_joint_->GetChild()->GetCollision(id));
          rear_right_wheel_radius_ = collisionRadius(rear_right_wheel_joint_->GetChild()->GetCollision(id));

          RCLCPP_INFO(node->get_logger(), "Radii found: %f %f %f %f",
                      front_left_wheel_radius_, front_right_wheel_radius_, rear_left_wheel_radius_, rear_right_wheel_radius_);

          // Compute wheelbase, frontTrackWidth, and rearTrackWidth
          gzdbg << "Computing wheelbase and track widths..." << std::endl;
          auto front_left_center_pos = front_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
          auto front_right_pos = front_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
          auto rear_left_pos = rear_left_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
          auto rear_right_pos = rear_right_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();

          auto vec3 = front_left_center_pos - front_right_pos;
          front_track_width_ = vec3.Length();
          vec3 = rear_left_pos - rear_right_pos;
          back_track_width_ = vec3.Length();

          auto front_axle_pos = (front_left_center_pos + front_right_pos) / 2;
          auto back_axle_pos = (rear_left_pos + rear_right_pos) / 2;
          vec3 = front_axle_pos - back_axle_pos;
          wheel_base_length_ = vec3.Length();

          update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
              std::bind(&SdControlPlugin::Update, this));

          gzdbg << "SdControlPlugin loaded successfully!" << std::endl;
      } catch (const std::exception &e) {
          gzerr << "Exception while loading SdControlPlugin: " << e.what() << std::endl;
      } catch (...) {
          gzerr << "Unknown exception while loading SdControlPlugin." << std::endl;
      }
  }

  void SdControlPlugin::Reset()
  {
  }
  void SdControlPlugin::controlCallback(const sd_msgs::msg::SDControl &msg)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    control_cmd_ = msg;
    if (control_cmd_.torque > 0 && control_cmd_.torque <= 25){
      control_cmd_.torque = 0; //This captures the dead pedal band on the real vehicle. 
    }
  }

  void SdControlPlugin::Update()
  {
    std::lock_guard<std::mutex> lock{mutex_};

    auto cur_time = world_->SimTime();
    auto dt = (cur_time - last_sim_time_).Double();
    if (dt < 0) {
      return;
    }
    else if (dt == 0.0)
    {
      return;
    }

    auto fl_steering_angle = front_left_wheel_steering_joint_->Position(0);
    auto fr_steering_angle = front_right_wheel_steering_joint_->Position(0);

    // Comment out unused variables
    // auto front_left_angular_velocity = front_left_wheel_joint_->GetVelocity(0);
    // auto front_right_angular_velocity = front_right_wheel_joint_->GetVelocity(0);
    auto rear_left_angular_velocity = rear_left_wheel_joint_->GetVelocity(0);
    auto rear_right_angular_velocity = rear_right_wheel_joint_->GetVelocity(0);

    auto chassis_linear_velocity = chassis_link_->WorldCoGLinearVel();

    auto drag_force = -chassis_aero_force_gain_ * chassis_linear_velocity.Length() * chassis_linear_velocity.Normalized();
    chassis_link_->AddForce(drag_force);

    auto steer_ratio = std::max(-100.0, std::min(100.0, control_cmd_.steer)) / 100.0;
    auto steer_angle = steer_ratio * max_steer_;

    auto tan_steer = std::tan(steer_angle);
    auto front_left_steering_command = std::atan2(tan_steer, 1.0 + front_track_width_ / 2 / wheel_base_length_ * tan_steer);
    auto front_right_steering_command = std::atan2(tan_steer, 1.0 - front_track_width_ / 2 / wheel_base_length_ * tan_steer);

    auto fl_steering_error = fl_steering_angle - front_left_steering_command;
    auto fr_steering_error = fr_steering_angle - front_right_steering_command;

    auto front_left_steering_force = front_left_wheel_steering_pid_.Update(fl_steering_error, dt);
    auto front_right_steering_force = front_right_wheel_steering_pid_.Update(fr_steering_error, dt);

    front_left_wheel_steering_joint_->SetForce(0, front_left_steering_force);
    front_right_wheel_steering_joint_->SetForce(0, front_right_steering_force);

    auto throttle_ratio = 0.0;
    auto brake_ratio = 0.0;
    if (control_cmd_.torque > 0)
      throttle_ratio = std::min(100.0, control_cmd_.torque) / 100.0;
    if (control_cmd_.torque < 0)
      brake_ratio = std::min(100.0, -control_cmd_.torque) / 100.0;

    auto regen_braking_ratio = 0.025;
    brake_ratio = std::max(regen_braking_ratio - throttle_ratio, brake_ratio);
    brake_ratio = std::max(0.0, std::min(1.0, brake_ratio));

    front_left_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);
    front_right_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);
    rear_left_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);
    rear_right_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);

    auto throttle_torque = 0.0;
    if (std::abs(rear_left_angular_velocity * rear_left_wheel_radius_) < max_speed_ &&
        std::abs(rear_right_angular_velocity * rear_right_wheel_radius_) < max_speed_)
      throttle_torque = throttle_ratio * max_torque_;

    rear_left_wheel_joint_->SetForce(0, throttle_torque);
    rear_right_wheel_joint_->SetForce(0, throttle_torque);

    last_sim_time_ = cur_time;
  }

  double SdControlPlugin::collisionRadius(gazebo::physics::CollisionPtr coll) const
  {
    if (coll == nullptr || coll->GetShape() == nullptr)
      return 0;

    if (coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
    {
      auto *cyl = static_cast<gazebo::physics::CylinderShape*>(coll->GetShape().get());
      return cyl->GetRadius();
    }
    else if (coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE))
    {
      auto *sph = static_cast<gazebo::physics::SphereShape*>(coll->GetShape().get());
      return sph->GetRadius();
    }
    return 0;
  }

  GZ_REGISTER_MODEL_PLUGIN(SdControlPlugin)

} // namespace sd_control
