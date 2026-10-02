#include "gz_virtual_base_system/gz_virtual_base_velocity_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace gz_virtual_base_velocity_controller
{

controller_interface::CallbackReturn
GazeboVirtualBaseVelocityController::on_init()
{
  try
  {
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<std::string>("cmd_vel_topic", "~/cmd_vel");
    auto_declare<std::string>("odom_topic", "~/odom");
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<std::string>("base_frame_id", "base_link");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception during init: %s",
      e.what());

    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GazeboVirtualBaseVelocityController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
      {
        "mobile_base/vx",
        "mobile_base/vy",
        "mobile_base/wz"
      }
    };
}

controller_interface::InterfaceConfiguration
GazeboVirtualBaseVelocityController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      "virtual_joint_x/position",
      "virtual_joint_y/position",
      "virtual_joint_yaw/position",
      "mobile_base/vx",
      "mobile_base/vy",
      "mobile_base/wz",
    }
  };
}

controller_interface::CallbackReturn
GazeboVirtualBaseVelocityController::on_configure(
  const rclcpp_lifecycle::State &)
{

  cmd_vel_timeout_ =
    get_node()->get_parameter("cmd_vel_timeout").as_double();

  const auto sub_topic =
    get_node()->get_parameter("cmd_vel_topic").as_string();

  const auto odom_topic = 
    get_node()->get_parameter("odom_topic").as_string();

  odom_pub_ =
    get_node()->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SystemDefaultsQoS());

  odom_msg_.header.frame_id =
    get_node()->get_parameter("odom_frame_id").as_string();
  odom_msg_.child_frame_id =
    get_node()->get_parameter("base_frame_id").as_string();

  cmd_sub_ =
    get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      sub_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &GazeboVirtualBaseVelocityController::cmdVelCallback,
        this,
        std::placeholders::_1));

  last_cmd_time_ = get_node()->now();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Subscribed to %s (timeout %.3f s)",
    sub_topic.c_str(),
    cmd_vel_timeout_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GazeboVirtualBaseVelocityController::on_activate(
  const rclcpp_lifecycle::State &)
{
  last_cmd_time_ = get_node()->now();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GazeboVirtualBaseVelocityController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() >= 3)
  {
    if (!command_interfaces_[0].set_value(0.0))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set vx command");
    }

    if (!command_interfaces_[1].set_value(0.0))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set vy command");
    }

    if (!command_interfaces_[2].set_value(0.0))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set wz command");
    }
  }
  return CallbackReturn::SUCCESS;
}

void GazeboVirtualBaseVelocityController::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::scoped_lock lock(cmd_mutex_);

  cmd_vx_ = msg->twist.linear.x;
  cmd_vy_ = msg->twist.linear.y;
  cmd_wz_ = msg->twist.angular.z;

  last_cmd_time_ = get_node()->now();
}

controller_interface::return_type
GazeboVirtualBaseVelocityController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{

  auto x_world = state_interfaces_[0].get_optional();
  auto y_world = state_interfaces_[1].get_optional();
  auto yaw_world = state_interfaces_[2].get_optional();
  auto actual_vx = state_interfaces_[3].get_optional();
  auto actual_vy = state_interfaces_[4].get_optional();
  auto actual_wz = state_interfaces_[5].get_optional();
  if (x_world && y_world && yaw_world &&
      actual_vx && actual_vy && actual_wz)
  {
    odom_msg_.pose.pose.position.x = x_world.value();
    odom_msg_.pose.pose.position.y = y_world.value();
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = sin(yaw_world.value() / 2.0);
    odom_msg_.pose.pose.orientation.w = cos(yaw_world.value() / 2.0);
    odom_msg_.twist.twist.linear.x = actual_vx.value();
    odom_msg_.twist.twist.linear.y = actual_vy.value();
    odom_msg_.twist.twist.angular.z = actual_wz.value();
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      5000,
      "Failed to read state interfaces");
  }
  odom_msg_.header.stamp = time;

  odom_pub_->publish(odom_msg_);

  double vx;
  double vy;
  double wz;

  {
    std::scoped_lock lock(cmd_mutex_);

    vx = cmd_vx_;
    vy = cmd_vy_;
    wz = cmd_wz_;
  }

  const bool timed_out =
    (time - last_cmd_time_).seconds() > cmd_vel_timeout_;

  if (timed_out)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  const bool ok =
    command_interfaces_[0].set_value(vx) &&
    command_interfaces_[1].set_value(vy) &&
    command_interfaces_[2].set_value(wz);

  if (!ok)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      5000,
      "Failed to write base velocity command");
  }

  return controller_interface::return_type::OK;
}

}  // namespace gz_virtual_base_velocity_controller

PLUGINLIB_EXPORT_CLASS(
  gz_virtual_base_velocity_controller::GazeboVirtualBaseVelocityController,
  controller_interface::ControllerInterface)