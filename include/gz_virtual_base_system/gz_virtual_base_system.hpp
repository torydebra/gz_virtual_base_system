// Custom gz_ros2_control system plugin: body-frame velocity interface for a
// holonomic base modelled as three virtual joints (x, y, yaw) in the world/odom frame.
//
// Exposes (as a <gpio> named "base" by default):
//   command: base/vx, base/vy, base/wz      (body frame, m/s and rad/s)
//   state:   base/vx, base/vy, base/wz      (measured, body frame, optional)
// and the usual position/velocity state interfaces of the three virtual joints.
//
// API mirrors the gz_ros2_control_demos custom system (newer ros2_control API with
// shared-pointer interfaces and HardwareComponentInterfaceParams).

#ifndef GZ_VIRTUAL_BASE_SYSTEM_GZ_VIRTUAL_BASE_SYSTEM_HPP_
#define GZ_VIRTUAL_BASE_SYSTEM_GZ_VIRTUAL_BASE_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/EntityComponentManager.hh>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace gz_virtual_base_system
{
namespace sim = gz::sim;

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GazeboVirtualBaseSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & enableJoints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & _ecm,
    unsigned int update_rate) override;

private:
  /// One virtual joint (x, y or yaw) and its exported joint-level state interfaces.
  struct Axis
  {
    std::string name;
    sim::Entity entity{sim::kNullEntity};
    hardware_interface::StateInterface::SharedPtr position;
    hardware_interface::StateInterface::SharedPtr velocity;
  };

  bool setupAxis(
    Axis & axis, const std::string & name,
    const hardware_interface::HardwareInfo & info,
    std::map<std::string, sim::Entity> & enable_joints);

  void readAxis(const Axis & axis, double & position, double & velocity) const;

  sim::EntityComponentManager * ecm_{nullptr};

  Axis x_, y_, yaw_;

  // Body-frame command interfaces (gpio "base")
  hardware_interface::CommandInterface::SharedPtr cmd_vx_, cmd_vy_, cmd_wz_;
  // Body-frame measured velocity (optional state interfaces on the gpio)
  hardware_interface::StateInterface::SharedPtr st_vx_, st_vy_, st_wz_;

  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  // Latest yaw measured in read(), used to rotate commands in write()
  double yaw_angle_{0.0};

  // Acceleration limiting in the body frame (0 = disabled)
  double max_linear_acc_{0.0};
  double max_angular_acc_{0.0};
  double applied_vx_{0.0}, applied_vy_{0.0}, applied_wz_{0.0};
};

}  // namespace gz_virtual_base_system

#endif  // GZ_VIRTUAL_BASE_SYSTEM_GZ_VIRTUAL_BASE_SYSTEM_HPP_
