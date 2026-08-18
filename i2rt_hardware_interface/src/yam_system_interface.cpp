#include "i2rt_hardware_interface/yam_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace i2rt_hardware_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
std::string param_or(const std::unordered_map<std::string, std::string> & params, const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}
}  // namespace

CallbackReturn YamSystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & info = get_hardware_info();

  const auto can_channel_it = info.hardware_parameters.find("can_channel");
  if (can_channel_it == info.hardware_parameters.end()) {
    RCLCPP_ERROR(get_logger(), "Missing required <hardware> param 'can_channel'");
    return CallbackReturn::ERROR;
  }
  can_channel_ = can_channel_it->second;

  std::vector<std::pair<int, i2rt_can_driver::MotorType>> motor_list;
  std::vector<double> offsets;
  std::vector<double> directions;

  for (const auto & joint : info.joints) {
    try {
      const int motor_id = std::stoi(joint.parameters.at("motor_id"));
      const auto motor_type = i2rt_can_driver::motor_type_from_string(joint.parameters.at("motor_type"));
      const double kp = std::stod(joint.parameters.at("kp"));
      const double kd = std::stod(joint.parameters.at("kd"));
      const double offset = std::stod(param_or(joint.parameters, "offset", "0.0"));
      const double direction = std::stod(param_or(joint.parameters, "direction", "1.0"));
      const double gravity_comp_factor = std::stod(param_or(joint.parameters, "gravity_comp_factor", "1.0"));

      joint_names_.push_back(joint.name);
      motor_types_.push_back(motor_type);
      joint_kp_.push_back(kp);
      joint_kd_.push_back(kd);
      gravity_comp_factor_.push_back(gravity_comp_factor);
      motor_list.emplace_back(motor_id, motor_type);
      offsets.push_back(offset);
      directions.push_back(direction);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' is missing/invalid required <param> (motor_id, motor_type, kp, kd): %s",
        joint.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  gravity_root_link_ = param_or(info.hardware_parameters, "gravity_root_link", "base_link");
  gravity_tip_link_ = param_or(info.hardware_parameters, "gravity_tip_link", "link_6");
  try {
    max_gravity_torque_nm_ = std::stod(param_or(info.hardware_parameters, "max_gravity_torque_nm", "25.0"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid 'max_gravity_torque_nm' hardware param: %s", e.what());
    return CallbackReturn::ERROR;
  }

  try {
    transport_ = std::make_unique<i2rt_can_driver::CanTransport>(
      can_channel_, i2rt_can_driver::ReceiveMode::P16, info.name);
    auto logger = get_logger();
    transport_->set_log_callback([logger](i2rt_can_driver::LogLevel level, const std::string & msg) {
      switch (level) {
        case i2rt_can_driver::LogLevel::Warning:
          RCLCPP_WARN(logger, "%s", msg.c_str());
          break;
        case i2rt_can_driver::LogLevel::Error:
          RCLCPP_ERROR(logger, "%s", msg.c_str());
          break;
        default:
          RCLCPP_INFO(logger, "%s", msg.c_str());
      }
    });
    dm_chain_ = std::make_unique<i2rt_can_driver::DmChain>(motor_list, offsets, directions, *transport_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN channel '%s': %s", can_channel_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  commands_.assign(joint_names_.size(), i2rt_can_driver::JointCommand{});

  // Diagnostics + gravity model: a small internal node added to
  // controller_manager's own executor, publishing i2rt_msgs/MotorStatus at
  // ~10 Hz and subscribing to /robot_description to build the KDL gravity
  // chain. See the class comment for why these live here instead of
  // dedicated broadcaster/controller plugins.
  if (auto executor = params.executor.lock()) {
    diagnostics_node_ = std::make_shared<rclcpp::Node>(info.name + "_diagnostics", info.name);
    diagnostics_pub_ = diagnostics_node_->create_publisher<i2rt_msgs::msg::MotorStatus>(
      "motor_feedback", rclcpp::QoS(10));
    robot_description_sub_ = diagnostics_node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String & msg) { on_robot_description(msg); });
    executor->add_node(diagnostics_node_);
  } else {
    RCLCPP_WARN(
      get_logger(), "No executor available at on_init; motor diagnostics and gravity comp will not be available");
  }
  const unsigned int rw_rate = info.rw_rate > 0 ? info.rw_rate : 250;
  diagnostics_publish_every_n_cycles_ = std::max(1u, rw_rate / 10u);

  gain_ramp_seconds_ = std::stod(param_or(info.hardware_parameters, "gain_ramp_seconds", "1.5"));
  if (gain_ramp_seconds_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "gain_ramp_seconds must be >= 0, got %f", gain_ramp_seconds_);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void YamSystemInterface::on_robot_description(const std_msgs::msg::String & msg)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(msg.data, tree)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse /robot_description into a KDL tree");
    return;
  }
  KDL::Chain chain;
  if (!tree.getChain(gravity_root_link_, gravity_tip_link_, chain)) {
    RCLCPP_ERROR(
      get_logger(), "KDL tree has no chain from '%s' to '%s'; gravity compensation disabled",
      gravity_root_link_.c_str(), gravity_tip_link_.c_str());
    return;
  }
  if (chain.getNrOfJoints() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_logger(),
      "KDL chain '%s'->'%s' has %u joints but this hardware component has %zu; gravity compensation disabled",
      gravity_root_link_.c_str(), gravity_tip_link_.c_str(), chain.getNrOfJoints(), joint_names_.size());
    return;
  }

  std::lock_guard<std::mutex> lock(gravity_model_mutex_);
  gravity_chain_ = std::make_unique<KDL::Chain>(chain);
  gravity_dyn_param_ = std::make_unique<KDL::ChainDynParam>(*gravity_chain_, KDL::Vector(0.0, 0.0, -9.81));
  RCLCPP_INFO(
    get_logger(), "Gravity compensation model ready ('%s' -> '%s', %u joints)", gravity_root_link_.c_str(),
    gravity_tip_link_.c_str(), chain.getNrOfJoints());
}

std::vector<double> YamSystemInterface::compute_gravity_torques()
{
  std::vector<double> torques(joint_names_.size(), 0.0);

  std::lock_guard<std::mutex> lock(gravity_model_mutex_);
  if (!gravity_dyn_param_) {
    if (!gravity_model_warned_) {
      RCLCPP_WARN(
        get_logger(),
        "Gravity compensation model not ready yet (waiting on /robot_description); "
        "commanding zero gravity feed-forward until it arrives");
      gravity_model_warned_ = true;
    }
    return torques;
  }

  KDL::JntArray q(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = dm_chain_->joint_position(i);
  }
  KDL::JntArray gravity(joint_names_.size());
  if (gravity_dyn_param_->JntToGravity(q, gravity) < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "KDL JntToGravity failed; commanding zero gravity feed-forward");
    return torques;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double t = gravity_comp_factor_[i] * gravity(i);
    if (!std::isfinite(t)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Non-finite gravity torque for joint '%s'; commanding zero",
        joint_names_[i].c_str());
      continue;
    }
    torques[i] = std::clamp(t, -max_gravity_torque_nm_, max_gravity_torque_nm_);
    if (std::abs(t) > max_gravity_torque_nm_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Gravity torque for joint '%s' clamped: %.2f -> %.2f Nm",
        joint_names_[i].c_str(), t, torques[i]);
    }
  }
  return torques;
}

CallbackReturn YamSystemInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    dm_chain_->enable();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to enable YAM motors: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    commands_[i].pos = dm_chain_->joint_position(i);
    commands_[i].vel = 0.0;
    commands_[i].torque = 0.0;
    // kp/kd are set per-cycle in write() as a ramp from activation_time_ —
    // left at zero here so the very first write() cycle starts the ramp
    // from zero, not from a full-gain value.
    commands_[i].kp = 0.0;
    commands_[i].kd = 0.0;
  }
  activation_time_ = get_clock()->now();
  cycles_since_diagnostics_ = 0;
  enabled_ = true;
  RCLCPP_INFO(
    get_logger(), "YAM motors enabled; ramping gains to target over %.2fs", gain_ramp_seconds_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn YamSystemInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  enabled_ = false;
  try {
    dm_chain_->disable();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to cleanly disable YAM motors: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

return_type YamSystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!enabled_) {
    return return_type::OK;
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, dm_chain_->joint_position(i));
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, dm_chain_->joint_velocity(i));
    set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT, dm_chain_->joint_effort(i));
  }
  publish_diagnostics_if_due();
  return return_type::OK;
}

return_type YamSystemInterface::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!enabled_) {
    return return_type::OK;
  }

  // Linear gain ramp from 0 at activation_time_ up to each joint's target
  // kp/kd (URDF default, or a controller's live kp/kd command once one is
  // claimed) over gain_ramp_seconds_, then held at target. See the class
  // comment for why this exists.
  const double elapsed_s = (time - activation_time_).seconds();
  const double ramp = gain_ramp_seconds_ > 0.0
                         ? std::clamp(elapsed_s / gain_ramp_seconds_, 0.0, 1.0)
                         : 1.0;

  const std::vector<double> gravity_torques = compute_gravity_torques();

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Command interfaces read back as NaN until some controller actually
    // claims and writes them — e.g. every cycle between hardware activation
    // and a controller successfully activating, which can be many cycles.
    // Encoding NaN through the MIT frame's uint clamp is undefined behavior;
    // on this platform it was observed to silently decode as "command
    // position = the joint's minimum limit", which combined with the gain
    // ramp caused real, dangerous motion. Never let a non-finite command
    // reach the motors — hold the last known-good value instead. The same
    // guard now applies to kp/kd: unclaimed (NaN) falls back to the
    // URDF-configured default, ramped exactly as before.
    const double pos_cmd = get_command(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
    const double vel_cmd = get_command(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    const double eff_cmd = get_command(joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT);
    const double kp_cmd = get_command(joint_names_[i] + "/kp");
    const double kd_cmd = get_command(joint_names_[i] + "/kd");
    if (std::isfinite(pos_cmd)) {
      commands_[i].pos = pos_cmd;
    }
    if (std::isfinite(vel_cmd)) {
      commands_[i].vel = vel_cmd;
    }
    const double base_torque = std::isfinite(eff_cmd) ? eff_cmd : 0.0;
    commands_[i].torque = base_torque + gravity_torques[i];
    commands_[i].kp = (std::isfinite(kp_cmd) ? kp_cmd : joint_kp_[i]) * ramp;
    commands_[i].kd = (std::isfinite(kd_cmd) ? kd_cmd : joint_kd_[i]) * ramp;
  }

  try {
    dm_chain_->set_commands(commands_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAM motor communication failure, recovery exhausted: %s", e.what());
    return return_type::ERROR;
  }
  return return_type::OK;
}

void YamSystemInterface::publish_diagnostics_if_due()
{
  if (!diagnostics_pub_) {
    return;
  }
  if (++cycles_since_diagnostics_ < diagnostics_publish_every_n_cycles_) {
    return;
  }
  cycles_since_diagnostics_ = 0;

  i2rt_msgs::msg::MotorStatus status_msg;
  status_msg.header.stamp = get_clock()->now();
  status_msg.num_motors = static_cast<uint8_t>(joint_names_.size());
  status_msg.all_motors_healthy = true;
  float max_temperature = 0.0f;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto & fb = dm_chain_->raw_feedback(i);

    i2rt_msgs::msg::MotorFeedback mf;
    mf.header = status_msg.header;
    mf.motor_id = static_cast<uint8_t>(fb.motor_id);
    mf.motor_type = i2rt_can_driver::motor_type_to_string(motor_types_[i]);
    mf.position = static_cast<float>(fb.position);
    mf.velocity = static_cast<float>(fb.velocity);
    mf.torque = static_cast<float>(fb.torque);
    mf.mos_temperature = static_cast<float>(fb.temperature_mos);
    mf.rotor_temperature = static_cast<float>(fb.temperature_rotor);
    mf.error_code = static_cast<uint8_t>(fb.error_code);
    mf.has_error = fb.error_code != i2rt_can_driver::motor_error_code::kNormal;
    mf.error_description = fb.error_message;

    if (mf.has_error) {
      status_msg.all_motors_healthy = false;
    }
    max_temperature = std::max({max_temperature, mf.mos_temperature, mf.rotor_temperature});
    status_msg.motors.push_back(mf);
  }
  status_msg.max_temperature = max_temperature;
  diagnostics_pub_->publish(status_msg);
}

}  // namespace i2rt_hardware_interface

PLUGINLIB_EXPORT_CLASS(i2rt_hardware_interface::YamSystemInterface, hardware_interface::SystemInterface)
