#ifndef GZ_VIRTUAL_BASE_VELOCITY_CONTROLLER__BASE_VELOCITY_CONTROLLER_HPP_
#define GZ_VIRTUAL_BASE_VELOCITY_CONTROLLER__BASE_VELOCITY_CONTROLLER_HPP_

#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace gz_virtual_base_velocity_controller
{

class GazeboVirtualBaseVelocityController : public controller_interface::ControllerInterface
{
public:
  GazeboVirtualBaseVelocityController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void cmdVelCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;

  std::mutex cmd_mutex_;

  double actual_vx_{0.0};
  double actual_vy_{0.0};
  double actual_wz_{0.0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_msg_;

  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};

  rclcpp::Time last_cmd_time_;

  double cmd_vel_timeout_{0.5};
};

}  // namespace gz_virtual_base_velocity_controller

#endif