#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "i2rt_can_driver/can_transport.hpp"
#include "i2rt_can_driver/dm_chain.hpp"
#include "i2rt_msgs/msg/motor_status.hpp"
#include "kdl/chain.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/jntarray.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace i2rt_hardware_interface
{

// ros2_control SystemInterface for the I2RT YAM arm. Backed by i2rt_can_driver
// (a plain SocketCAN port of i2rt/motor_drivers/{can_interface.py,dm_driver.py}),
// with no internal control-loop thread of its own — controller_manager's own
// update loop drives read()/write() at whatever rate the URDF's <hardware>
// rw_rate specifies, so read()/write() just do blocking CAN I/O synchronously.
//
// Per joint, exposes standard position/velocity/effort command and state
// interfaces, plus kp/kd command interfaces (see <ros2_control> in
// yam_macro.xacro). The DM motors' MIT-mode kp/kd impedance gains default to
// each joint's "kp"/"kd" URDF parameters (ramped from 0 on activation, as
// before) but can be overridden per-cycle by any controller that claims the
// kp/kd command interfaces — e.g. a compliant/gravity-comp controller wanting
// near-zero stiffness. A controller that never touches kp/kd (like
// joint_trajectory_controller) sees no behavior change: those interfaces read
// back NaN until claimed, and the isfinite guard below falls back to the URDF
// default exactly as it always has.
//
// Gravity compensation: a per-joint feed-forward torque, computed once per
// write() cycle from a KDL chain (parsed from the /robot_description topic
// this class subscribes to internally) via KDL::ChainDynParam::JntToGravity,
// scaled by each joint's "gravity_comp_factor" URDF param and clamped to
// +/-max_gravity_torque_nm. This is unconditional — added on top of whatever
// the effort command interface requests (0 if unclaimed) — mirroring
// MotorChainRobot.update()'s `motor_torques = joint_commands.torques + g *
// gravity_comp_factor` in the Python reference, which is active in every
// control mode, not just an idle mode.
//
// Per-motor diagnostics (temperature, error code) that don't fit
// sensor_msgs/JointState are published directly on an internal rclcpp::Node
// as i2rt_msgs/MotorStatus, throttled to ~10 Hz — a pragmatic choice over a
// dedicated broadcaster-controller plugin. The same internal node is reused
// to subscribe to /robot_description for the gravity model.
//
// Safety: kp/kd are ramped linearly from 0 up to each joint's target gain
// over gain_ramp_seconds_ (default 1.5s) starting from every on_activate(),
// rather than commanding full gain instantly. This bounds how hard the arm
// can snap toward its hold command if the very first position reading after
// enable is ever wrong (bad config, transient CAN glitch, etc.) — added
// after a big_yam unit briefly moved at high speed on activation when it was
// mistakenly brought up with the standard yam's motor/gain config.
//
// A joint with requires_calibration=true (the linear_4310 gripper: it has no
// absolute encoder, so software doesn't know where its hard stops are after
// a power cycle) gets its hard-stop limits probed automatically, every
// on_activate(), before the gain ramp starts — see GripperCalibration and
// run_gripper_calibrations() below. This drives that motor into both hard
// stops with a small constant torque for up to a few seconds; keep the
// gripper clear of obstructions/fingers before activating real hardware.
class YamSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publish_diagnostics_if_due();
  void on_robot_description(const std_msgs::msg::String & msg);
  // Returns the per-joint gravity feed-forward torque (zero vector if the KDL
  // chain hasn't been built yet, e.g. /robot_description not received).
  std::vector<double> compute_gravity_torques();

  // Startup hard-stop calibration for a joint with no absolute encoder (e.g.
  // the linear_4310 gripper): probes both hard stops with a constant test
  // torque and records the raw motor position where it stops moving.
  // Mirrors i2rt's detect_gripper_limits (Python reference:
  // i2rt/robots/utils.py), run once, synchronously, from on_activate()
  // before normal ramped operation begins. See yam_system_interface.cpp's
  // run_gripper_calibrations() for the full algorithm and rationale.
  //
  // Calibrated joints must be configured with direction=1.0/offset=0.0 (see
  // on_init): DmChain's own per-motor transform must stay a no-op for them,
  // because raw_closed/raw_open below are recorded directly from raw_feedback
  // (bypassing DmChain's offset/direction), and the raw<->URDF-units mapping
  // this struct enables (see raw_to_joint_position/joint_position_to_raw) is
  // applied on top, in read()/write(), instead.
  struct GripperCalibration
  {
    size_t joint_index = 0;
    // Which raw sweep extreme is "closed" vs "open" -- same role as
    // Python's `motor_chain.motor_direction[gripper_index]`, kept separate
    // from this joint's own (fixed at 1.0) DmChain direction param.
    double polarity = 1.0;
    double lower_limit_m = 0.0;  // URDF joint position at the "closed" hard stop
    double upper_limit_m = 0.0;  // URDF joint position at the "open" hard stop
    double test_torque_nm = 0.5;
    double max_duration_s = 2.0;
    double position_threshold = 0.01;
    double check_interval_s = 0.05;
    int stable_count_required = 3;
    double direction_pause_s = 0.3;

    // Filled in by run_gripper_calibrations().
    double raw_closed = 0.0;
    double raw_open = 0.0;
    bool calibrated = false;
  };

  // Runs the hard-stop probe for every entry in gripper_calibrations_, in
  // order. Returns false (having logged why) if any probe fails outright or
  // yields a degenerate (near-zero) raw range -- on_activate() must refuse
  // to activate in that case rather than risk commanding a bogus position.
  bool run_gripper_calibrations();
  // Raw motor-space <-> URDF joint-space (linear) conversions for a
  // calibrated joint, valid only once cal.calibrated is true.
  double raw_to_joint_position(const GripperCalibration & cal, double raw) const;
  double joint_position_to_raw(const GripperCalibration & cal, double joint_position) const;
  double raw_to_joint_velocity(const GripperCalibration & cal, double raw_velocity) const;
  double joint_velocity_to_raw(const GripperCalibration & cal, double joint_velocity) const;

  std::string can_channel_;
  std::vector<std::string> joint_names_;
  std::vector<i2rt_can_driver::MotorType> motor_types_;
  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;
  std::vector<double> gravity_comp_factor_;

  std::vector<GripperCalibration> gripper_calibrations_;
  // joint index -> index into gripper_calibrations_, or -1 if that joint
  // isn't calibrated (the common case for every arm joint).
  std::vector<int> calibration_index_by_joint_;

  std::unique_ptr<i2rt_can_driver::CanTransport> transport_;
  std::unique_ptr<i2rt_can_driver::DmChain> dm_chain_;
  std::vector<i2rt_can_driver::JointCommand> commands_;
  bool enabled_ = false;

  double gain_ramp_seconds_ = 1.5;
  rclcpp::Time activation_time_;

  // Gravity model, built lazily from /robot_description once it arrives.
  // gravity_chain_joint_indices_[i] maps the gravity KDL chain's i-th
  // (non-fixed) joint to this component's joint index -- the chain only
  // needs to cover whatever's actually between gravity_root_link_ and
  // gravity_tip_link_ (the arm), so joint_names_ may legitimately contain
  // more joints than the chain does (e.g. a gripper hanging off the tip
  // link); those simply get zero gravity feed-forward.
  std::string gravity_root_link_;
  std::string gravity_tip_link_;
  double max_gravity_torque_nm_ = 25.0;
  std::mutex gravity_model_mutex_;
  std::unique_ptr<KDL::Chain> gravity_chain_;
  std::unique_ptr<KDL::ChainDynParam> gravity_dyn_param_;
  std::vector<size_t> gravity_chain_joint_indices_;
  bool gravity_model_warned_ = false;

  rclcpp::Node::SharedPtr diagnostics_node_;
  rclcpp::Publisher<i2rt_msgs::msg::MotorStatus>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  unsigned int diagnostics_publish_every_n_cycles_ = 1;
  unsigned int cycles_since_diagnostics_ = 0;
};

}  // namespace i2rt_hardware_interface
