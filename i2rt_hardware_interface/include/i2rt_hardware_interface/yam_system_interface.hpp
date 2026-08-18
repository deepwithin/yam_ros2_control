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

  std::string can_channel_;
  std::vector<std::string> joint_names_;
  std::vector<i2rt_can_driver::MotorType> motor_types_;
  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;
  std::vector<double> gravity_comp_factor_;

  std::unique_ptr<i2rt_can_driver::CanTransport> transport_;
  std::unique_ptr<i2rt_can_driver::DmChain> dm_chain_;
  std::vector<i2rt_can_driver::JointCommand> commands_;
  bool enabled_ = false;

  double gain_ramp_seconds_ = 1.5;
  rclcpp::Time activation_time_;

  // Gravity model, built lazily from /robot_description once it arrives.
  std::string gravity_root_link_;
  std::string gravity_tip_link_;
  double max_gravity_torque_nm_ = 25.0;
  std::mutex gravity_model_mutex_;
  std::unique_ptr<KDL::Chain> gravity_chain_;
  std::unique_ptr<KDL::ChainDynParam> gravity_dyn_param_;
  bool gravity_model_warned_ = false;

  rclcpp::Node::SharedPtr diagnostics_node_;
  rclcpp::Publisher<i2rt_msgs::msg::MotorStatus>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  unsigned int diagnostics_publish_every_n_cycles_ = 1;
  unsigned int cycles_since_diagnostics_ = 0;
};

}  // namespace i2rt_hardware_interface
