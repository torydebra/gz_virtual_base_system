#include "gz_virtual_base_system/gz_virtual_base_system.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gz_virtual_base_system
{

namespace
{
// Read a command value; treat unreadable / NaN as zero.
double readCommand(const hardware_interface::CommandInterface::SharedPtr & handle)
{
  double v = 0.0;
  if (handle && handle->get_value(v, true) && std::isfinite(v)) {
    return v;
  }
  return 0.0;
}

// First-order slew limiter: move `current` toward `target` by at most max_acc * dt.
void slew(double target, double & current, double max_acc, double dt)
{
  if (max_acc <= 0.0 || dt <= 0.0) {
    current = target;
    return;
  }
  const double step = max_acc * dt;
  current += std::clamp(target - current, -step, step);
}
}  // namespace

bool GazeboVirtualBaseSystem::setupAxis(
  Axis & axis, const std::string & name,
  const hardware_interface::HardwareInfo & info,
  std::map<std::string, sim::Entity> & enable_joints)
{
  axis.name = name;
  auto it = enable_joints.find(name);
  if (it == enable_joints.end()) {
    RCLCPP_ERROR_STREAM(
      this->nh_->get_logger(), "Virtual joint '" << name << "' not found in the Gazebo model.");
    return false;
  }
  axis.entity = it->second;

  // Make sure Gazebo publishes position and velocity for this joint
  if (!ecm_->EntityHasComponentType(axis.entity, sim::components::JointPosition().TypeId())) {
    ecm_->CreateComponent(axis.entity, sim::components::JointPosition());
  }
  if (!ecm_->EntityHasComponentType(axis.entity, sim::components::JointVelocity().TypeId())) {
    ecm_->CreateComponent(axis.entity, sim::components::JointVelocity());
  }

  // Joint-level state interfaces declared in the URDF (position / velocity)
  for (const auto & joint_info : info.joints) {
    if (joint_info.name != name) {
      continue;
    }
    for (const auto & si : joint_info.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        axis.position = std::make_shared<hardware_interface::StateInterface>(
          name, hardware_interface::HW_IF_POSITION);
        (void)axis.position->set_value(0.0, true);
        state_interfaces_.push_back(axis.position);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        axis.velocity = std::make_shared<hardware_interface::StateInterface>(
          name, hardware_interface::HW_IF_VELOCITY);
        (void)axis.velocity->set_value(0.0, true);
        state_interfaces_.push_back(axis.velocity);
      }
    }
  }
  RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Virtual base joint: " << name);
  return true;
}

bool GazeboVirtualBaseSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int /*update_rate*/)
{
  this->nh_ = model_nh;
  ecm_ = &_ecm;

  // ---- hardware parameters from the URDF <hardware> block ----
  auto param = [&hardware_info](const std::string & key, const std::string & def) {
      auto it = hardware_info.hardware_parameters.find(key);
      return it == hardware_info.hardware_parameters.end() ? def : it->second;
    };

  const std::string x_name = param("x_joint", "virtual_joint_x");
  const std::string y_name = param("y_joint", "virtual_joint_y");
  const std::string yaw_name = param("yaw_joint", "virtual_joint_yaw");
  const std::string gpio_name = param("gpio_name", "mobile_base");

  try {
    max_linear_acc_ = std::stod(param("max_linear_acc", "0.0"));
    max_angular_acc_ = std::stod(param("max_angular_acc", "0.0"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      this->nh_->get_logger(), "Invalid max_linear_acc / max_angular_acc: " << e.what());
    return false;
  }

  // ---- the three virtual joints ----
  if (!setupAxis(x_, x_name, hardware_info, enableJoints) ||
    !setupAxis(y_, y_name, hardware_info, enableJoints) ||
    !setupAxis(yaw_, yaw_name, hardware_info, enableJoints))
  {
    return false;
  }

  // ---- body-frame gpio interfaces ----
  const hardware_interface::ComponentInfo * gpio = nullptr;
  for (const auto & g : hardware_info.gpios) {
    if (g.name == gpio_name) {
      gpio = &g;
      break;
    }
  }
  if (gpio == nullptr) {
    RCLCPP_ERROR_STREAM(
      this->nh_->get_logger(),
      "No <gpio name=\"" << gpio_name << "\"> declared in the ros2_control block.");
    return false;
  }

  for (const auto & ci : gpio->command_interfaces) {
    auto handle = std::make_shared<hardware_interface::CommandInterface>(gpio->name, ci.name);
    (void)handle->set_value(0.0, true);
    if (ci.name == "vx") {
      cmd_vx_ = handle;
    } else if (ci.name == "vy") {
      cmd_vy_ = handle;
    } else if (ci.name == "wz") {
      cmd_wz_ = handle;
    } else {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Ignoring unknown gpio command interface '" << ci.name << "'");
      continue;
    }
    command_interfaces_.push_back(handle);
  }
  if (!cmd_vx_ || !cmd_vy_ || !cmd_wz_) {
    RCLCPP_ERROR(
      this->nh_->get_logger(), "gpio '%s' must declare command interfaces vx, vy and wz.",
      gpio_name.c_str());
    return false;
  }

  for (const auto & si : gpio->state_interfaces) {
    auto handle = std::make_shared<hardware_interface::StateInterface>(gpio->name, si.name);
    (void)handle->set_value(0.0, true);
    if (si.name == "vx") {
      st_vx_ = handle;
    } else if (si.name == "vy") {
      st_vy_ = handle;
    } else if (si.name == "wz") {
      st_wz_ = handle;
    } else {
      continue;
    }
    state_interfaces_.push_back(handle);
  }

  RCLCPP_INFO(
    this->nh_->get_logger(), "Virtual base ready (max_linear_acc=%.3f, max_angular_acc=%.3f)",
    max_linear_acc_, max_angular_acc_);
  return true;
}

CallbackReturn GazeboVirtualBaseSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboVirtualBaseSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(this->nh_->get_logger(), "GazeboVirtualBaseSystem configured.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
GazeboVirtualBaseSystem::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> out;
  for (auto & si : state_interfaces_) {
    out.push_back(si);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
GazeboVirtualBaseSystem::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> out;
  for (auto & ci : command_interfaces_) {
    out.push_back(ci);
  }
  return out;
}

CallbackReturn GazeboVirtualBaseSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  applied_vx_ = applied_vy_ = applied_wz_ = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboVirtualBaseSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

void GazeboVirtualBaseSystem::readAxis(
  const Axis & axis, double & position, double & velocity) const
{
  const auto * p = ecm_->Component<sim::components::JointPosition>(axis.entity);
  const auto * v = ecm_->Component<sim::components::JointVelocity>(axis.entity);
  position = (p && !p->Data().empty()) ? p->Data()[0] : 0.0;
  velocity = (v && !v->Data().empty()) ? v->Data()[0] : 0.0;
}

hardware_interface::return_type GazeboVirtualBaseSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double px, vx, py, vy, pyaw, vyaw;
  readAxis(x_, px, vx);
  readAxis(y_, py, vy);
  readAxis(yaw_, pyaw, vyaw);

  yaw_angle_ = pyaw;

  if (x_.position) {(void)x_.position->set_value(px, true);}
  if (x_.velocity) {(void)x_.velocity->set_value(vx, true);}
  if (y_.position) {(void)y_.position->set_value(py, true);}
  if (y_.velocity) {(void)y_.velocity->set_value(vy, true);}
  if (yaw_.position) {(void)yaw_.position->set_value(pyaw, true);}
  if (yaw_.velocity) {(void)yaw_.velocity->set_value(vyaw, true);}

  // Measured velocity expressed in the body frame (world -> body rotation)
  const double c = std::cos(pyaw);
  const double s = std::sin(pyaw);
  if (st_vx_) {(void)st_vx_->set_value(c * vx + s * vy, true);}
  if (st_vy_) {(void)st_vy_->set_value(-s * vx + c * vy, true);}
  if (st_wz_) {(void)st_wz_->set_value(vyaw, true);}

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboVirtualBaseSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // Body-frame targets
  const double target_vx = readCommand(cmd_vx_);
  const double target_vy = readCommand(cmd_vy_);
  const double target_wz = readCommand(cmd_wz_);

  // Optional acceleration limiting, applied in the body frame
  slew(target_vx, applied_vx_, max_linear_acc_, dt);
  slew(target_vy, applied_vy_, max_linear_acc_, dt);
  slew(target_wz, applied_wz_, max_angular_acc_, dt);

  // Rotate body-frame velocity into the world/odom frame using the current yaw
  const double c = std::cos(yaw_angle_);
  const double s = std::sin(yaw_angle_);
  const double world_vx = applied_vx_ * c - applied_vy_ * s;
  const double world_vy = applied_vx_ * s + applied_vy_ * c;

  ecm_->SetComponentData<sim::components::JointVelocityCmd>(x_.entity, {world_vx});
  ecm_->SetComponentData<sim::components::JointVelocityCmd>(y_.entity, {world_vy});
  ecm_->SetComponentData<sim::components::JointVelocityCmd>(yaw_.entity, {applied_wz_});

  return hardware_interface::return_type::OK;
}

}  // namespace gz_virtual_base_system

PLUGINLIB_EXPORT_CLASS(
  gz_virtual_base_system::GazeboVirtualBaseSystem, gz_ros2_control::GazeboSimSystemInterface)
