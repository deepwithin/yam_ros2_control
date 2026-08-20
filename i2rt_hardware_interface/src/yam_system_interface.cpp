#include "i2rt_hardware_interface/yam_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

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
      // Only meaningful while compliant_mode_ is true (see class comment);
      // default of 0.3 is a placeholder for joints that don't set it -
      // arm joints should set this explicitly to their ported grav_comp_kd
      // value (big_yam_ros2_control.xacro does).
      const double compliant_kd = std::stod(param_or(joint.parameters, "compliant_kd", "0.3"));

      joint_names_.push_back(joint.name);
      motor_types_.push_back(motor_type);
      joint_kp_.push_back(kp);
      joint_kd_.push_back(kd);
      gravity_comp_factor_.push_back(gravity_comp_factor);
      compliant_kd_.push_back(compliant_kd);
      motor_list.emplace_back(motor_id, motor_type);
      offsets.push_back(offset);
      directions.push_back(direction);

      if (param_or(joint.parameters, "requires_calibration", "false") == "true") {
        if (direction != 1.0 || offset != 0.0) {
          RCLCPP_ERROR(
            get_logger(),
            "Joint '%s' has requires_calibration=true but direction=%.1f/offset=%.3f - calibrated joints must "
            "use direction=1.0 and offset=0.0. The calibration routine applies its own raw<->URDF-units mapping "
            "on top of raw feedback; DmChain's own offset/direction transform needs to stay a no-op for this "
            "joint, or the two mappings would compound incorrectly.",
            joint.name.c_str(), direction, offset);
          return CallbackReturn::ERROR;
        }
        GripperCalibration cal;
        cal.joint_index = joint_names_.size() - 1;
        cal.polarity = std::stod(param_or(joint.parameters, "calibration_polarity", "1.0"));
        cal.lower_limit_m = std::stod(joint.parameters.at("calibration_lower_limit_m"));
        cal.upper_limit_m = std::stod(joint.parameters.at("calibration_upper_limit_m"));
        cal.test_torque_nm = std::stod(param_or(joint.parameters, "calibration_test_torque_nm", "0.5"));
        cal.max_duration_s = std::stod(param_or(joint.parameters, "calibration_max_duration_s", "2.0"));
        cal.position_threshold = std::stod(param_or(joint.parameters, "calibration_position_threshold", "0.01"));
        cal.check_interval_s = std::stod(param_or(joint.parameters, "calibration_check_interval_s", "0.05"));
        cal.stable_count_required = std::stoi(param_or(joint.parameters, "calibration_stable_count", "3"));
        cal.direction_pause_s = std::stod(param_or(joint.parameters, "calibration_direction_pause_s", "0.3"));
        gripper_calibrations_.push_back(cal);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' is missing/invalid required <param> (motor_id, motor_type, kp, kd, or - if "
        "requires_calibration=true - calibration_lower_limit_m/calibration_upper_limit_m): %s",
        joint.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  calibration_index_by_joint_.assign(joint_names_.size(), -1);
  for (size_t c = 0; c < gripper_calibrations_.size(); ++c) {
    calibration_index_by_joint_[gripper_calibrations_[c].joint_index] = static_cast<int>(c);
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

  compliant_mode_ = param_or(info.hardware_parameters, "compliant_mode", "false") == "true";
  if (compliant_mode_) {
    RCLCPP_WARN(
      get_logger(),
      "compliant_mode is true: every joint's kp fallback is exactly 0.0 (gravity-comp does the holding; see "
      "class comment) and kd fallback is that joint's own \"compliant_kd\" param, unless a controller "
      "explicitly claims the kp/kd command interfaces.");
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

  // Map each of the chain's (non-fixed) joints to this component's joint
  // index by name, rather than assuming the chain's joint count/order lines
  // up positionally with joint_names_. joint_names_ may contain joints that
  // are legitimately NOT on this kinematic path (e.g. a gripper joint
  // hanging off gravity_tip_link_ rather than continuing the chain) - those
  // just never appear here and get zero gravity feed-forward below. A chain
  // joint that this component has no <joint> entry for at all, on the other
  // hand, is a real config error.
  std::vector<size_t> chain_joint_indices;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & kdl_joint = chain.getSegment(i).getJoint();
    if (kdl_joint.getType() == KDL::Joint::Fixed) {
      continue;
    }
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), kdl_joint.getName());
    if (it == joint_names_.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "KDL chain '%s'->'%s' includes joint '%s', which this hardware component has no <joint> entry for; "
        "gravity compensation disabled",
        gravity_root_link_.c_str(), gravity_tip_link_.c_str(), kdl_joint.getName().c_str());
      return;
    }
    chain_joint_indices.push_back(static_cast<size_t>(std::distance(joint_names_.begin(), it)));
  }

  std::lock_guard<std::mutex> lock(gravity_model_mutex_);
  gravity_chain_ = std::make_unique<KDL::Chain>(chain);
  gravity_dyn_param_ = std::make_unique<KDL::ChainDynParam>(*gravity_chain_, KDL::Vector(0.0, 0.0, -9.81));
  gravity_chain_joint_indices_ = std::move(chain_joint_indices);
  RCLCPP_INFO(
    get_logger(), "Gravity compensation model ready ('%s' -> '%s', %zu joints)", gravity_root_link_.c_str(),
    gravity_tip_link_.c_str(), gravity_chain_joint_indices_.size());
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

  const size_t n = gravity_chain_joint_indices_.size();
  KDL::JntArray q(n);
  for (size_t i = 0; i < n; ++i) {
    q(i) = dm_chain_->joint_position(gravity_chain_joint_indices_[i]);
  }
  KDL::JntArray gravity(n);
  if (gravity_dyn_param_->JntToGravity(q, gravity) < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "KDL JntToGravity failed; commanding zero gravity feed-forward");
    return torques;
  }

  for (size_t i = 0; i < n; ++i) {
    const size_t joint_idx = gravity_chain_joint_indices_[i];
    const double t = gravity_comp_factor_[joint_idx] * gravity(i);
    if (!std::isfinite(t)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Non-finite gravity torque for joint '%s'; commanding zero",
        joint_names_[joint_idx].c_str());
      continue;
    }
    torques[joint_idx] = std::clamp(t, -max_gravity_torque_nm_, max_gravity_torque_nm_);
    if (std::abs(t) > max_gravity_torque_nm_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Gravity torque for joint '%s' clamped: %.2f -> %.2f Nm",
        joint_names_[joint_idx].c_str(), t, torques[joint_idx]);
    }
  }
  return torques;
}

double YamSystemInterface::raw_to_joint_position(const GripperCalibration & cal, double raw) const
{
  const double span_raw = cal.raw_open - cal.raw_closed;
  const double frac = (raw - cal.raw_closed) / span_raw;
  return cal.lower_limit_m + frac * (cal.upper_limit_m - cal.lower_limit_m);
}

double YamSystemInterface::joint_position_to_raw(const GripperCalibration & cal, double joint_position) const
{
  const double span_m = cal.upper_limit_m - cal.lower_limit_m;
  const double frac = (joint_position - cal.lower_limit_m) / span_m;
  return cal.raw_closed + frac * (cal.raw_open - cal.raw_closed);
}

double YamSystemInterface::raw_to_joint_velocity(const GripperCalibration & cal, double raw_velocity) const
{
  return raw_velocity * (cal.upper_limit_m - cal.lower_limit_m) / (cal.raw_open - cal.raw_closed);
}

double YamSystemInterface::joint_velocity_to_raw(const GripperCalibration & cal, double joint_velocity) const
{
  return joint_velocity * (cal.raw_open - cal.raw_closed) / (cal.upper_limit_m - cal.lower_limit_m);
}

bool YamSystemInterface::run_gripper_calibrations()
{
  for (auto & cal : gripper_calibrations_) {
    const std::string & name = joint_names_[cal.joint_index];
    RCLCPP_INFO(
      get_logger(),
      "Calibrating gripper joint '%s': probing both hard stops (test_torque=%.2f Nm, up to %.1fs/direction)...",
      name.c_str(), cal.test_torque_nm, cal.max_duration_s);

    // Hold every other joint at whatever torque it was reading right after
    // enable(), kp=kd=0, for the whole probe - mirrors detect_gripper_
    // limits's `init_torque` snapshot in the Python reference
    // (i2rt/robots/utils.py): a brief, un-gravity-compensated hold on the
    // rest of the arm while only the gripper motor is actively driven.
    std::vector<double> held_torque(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      held_torque[i] = dm_chain_->joint_effort(i);
    }

    // Positions are sampled from raw_feedback (pre-offset/pre-direction) so
    // the polarity-based pairing below matches the exact algorithm this
    // mirrors; see the GripperCalibration comment for why that also means
    // this joint must be configured with direction=1.0/offset=0.0.
    std::vector<double> positions;
    positions.push_back(dm_chain_->raw_feedback(cal.joint_index).position);

    for (const double sweep_direction : {1.0, -1.0}) {
      const auto sweep_start = std::chrono::steady_clock::now();
      std::optional<double> last_pos;
      int stable_count = 0;

      while (std::chrono::duration<double>(std::chrono::steady_clock::now() - sweep_start).count() <
             cal.max_duration_s) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          commands_[i].pos = dm_chain_->joint_position(i);
          commands_[i].vel = 0.0;
          commands_[i].kp = 0.0;
          commands_[i].kd = 0.0;
          commands_[i].torque = (i == cal.joint_index) ? sweep_direction * cal.test_torque_nm : held_torque[i];
        }
        try {
          dm_chain_->set_commands(commands_);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Gripper calibration aborted: motor communication failure: %s", e.what());
          return false;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(cal.check_interval_s));

        const double current_pos = dm_chain_->raw_feedback(cal.joint_index).position;
        positions.push_back(current_pos);
        if (last_pos.has_value()) {
          if (std::abs(current_pos - *last_pos) < cal.position_threshold) {
            if (++stable_count >= cal.stable_count_required) {
              break;  // Hit a hard stop.
            }
          } else {
            stable_count = 0;
          }
        }
        last_pos = current_pos;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(cal.direction_pause_s));
    }

    const double raw_min = *std::min_element(positions.begin(), positions.end());
    const double raw_max = *std::max_element(positions.begin(), positions.end());
    if (cal.polarity > 0.0) {
      cal.raw_closed = raw_max;
      cal.raw_open = raw_min;
    } else {
      cal.raw_closed = raw_min;
      cal.raw_open = raw_max;
    }

    constexpr double kMinValidRawRange = 1e-3;
    if (std::abs(cal.raw_open - cal.raw_closed) < kMinValidRawRange) {
      RCLCPP_ERROR(
        get_logger(),
        "Gripper joint '%s' calibration failed: detected raw range too small (closed=%.4f, open=%.4f) - motor "
        "may not be moving freely, or CAN feedback isn't updating",
        name.c_str(), cal.raw_closed, cal.raw_open);
      return false;
    }

    cal.calibrated = true;
    RCLCPP_INFO(
      get_logger(), "Gripper joint '%s' calibrated: closed=%.4f, open=%.4f (raw motor rad) -> URDF [%.4f, %.4f] m",
      name.c_str(), cal.raw_closed, cal.raw_open, cal.lower_limit_m, cal.upper_limit_m);
  }
  return true;
}

CallbackReturn YamSystemInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    dm_chain_->enable();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to enable YAM motors: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!gripper_calibrations_.empty() && !run_gripper_calibrations()) {
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
    const int cal_idx = calibration_index_by_joint_[i];
    if (cal_idx >= 0 && gripper_calibrations_[cal_idx].calibrated) {
      // Calibrated joint: raw motor-space feedback, rescaled into the
      // URDF's linear units via this joint's detected hard-stop range. Not
      // dm_chain_->joint_position()/velocity() - those apply DmChain's own
      // offset/direction transform, which this joint is configured to keep
      // as a no-op (see GripperCalibration's comment) precisely so raw
      // feedback here matches what run_gripper_calibrations() calibrated
      // against.
      const auto & cal = gripper_calibrations_[cal_idx];
      const auto & fb = dm_chain_->raw_feedback(i);
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, raw_to_joint_position(cal, fb.position));
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, raw_to_joint_velocity(cal, fb.velocity));
      // Effort is reported as-is: raw motor torque (Nm), not force-
      // calibrated to the gripper's linear units - no motor-torque-to-
      // gripper-force ratio was available to convert it correctly.
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT, fb.torque);
    } else {
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, dm_chain_->joint_position(i));
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, dm_chain_->joint_velocity(i));
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT, dm_chain_->joint_effort(i));
    }
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
    const int cal_idx = calibration_index_by_joint_[i];
    if (cal_idx >= 0 && gripper_calibrations_[cal_idx].calibrated) {
      // Calibrated joint: incoming position/velocity commands are in the
      // URDF's linear units, so rescale into raw motor-space before they
      // reach commands_[i].pos/.vel - see the read() branch above for why
      // this bypasses DmChain's own offset/direction transform.
      const auto & cal = gripper_calibrations_[cal_idx];
      if (std::isfinite(pos_cmd)) {
        commands_[i].pos = joint_position_to_raw(cal, pos_cmd);
      } else if (compliant_mode_) {
        // kp's own fallback is always exactly 0.0 (see class comment), so
        // this is a no-op in practice, but keeps commands_[i].pos from
        // going stale (rather than holding whatever it was at
        // on_activate()) in the one case it'd matter: a real controller
        // explicitly claiming kp with a nonzero value while still leaving
        // position unclaimed.
        commands_[i].pos = dm_chain_->raw_feedback(i).position;
      }
      if (std::isfinite(vel_cmd)) {
        commands_[i].vel = joint_velocity_to_raw(cal, vel_cmd);
      } else if (compliant_mode_) {
        commands_[i].vel = dm_chain_->raw_feedback(i).velocity;
      }
    } else {
      if (std::isfinite(pos_cmd)) {
        commands_[i].pos = pos_cmd;
      } else if (compliant_mode_) {
        commands_[i].pos = dm_chain_->joint_position(i);
      }
      if (std::isfinite(vel_cmd)) {
        commands_[i].vel = vel_cmd;
      } else if (compliant_mode_) {
        commands_[i].vel = dm_chain_->joint_velocity(i);
      }
    }
    const double base_torque = std::isfinite(eff_cmd) ? eff_cmd : 0.0;
    commands_[i].torque = base_torque + gravity_torques[i];
    // In compliant_mode, the fallback (nothing has claimed kp/kd) target is
    // exactly 0.0 for kp and this joint's own compliant_kd_ for kd - see
    // the class comment for why (matches MotorChainRobot's
    // zero_gravity_mode exactly: kp=0 always, kd=the ported grav_comp_kd
    // value). An explicit kp/kd command still always wins.
    const double default_kp = compliant_mode_ ? 0.0 : joint_kp_[i];
    const double default_kd = compliant_mode_ ? compliant_kd_[i] : joint_kd_[i];
    commands_[i].kp = (std::isfinite(kp_cmd) ? kp_cmd : default_kp) * ramp;
    commands_[i].kd = (std::isfinite(kd_cmd) ? kd_cmd : default_kd) * ramp;
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
