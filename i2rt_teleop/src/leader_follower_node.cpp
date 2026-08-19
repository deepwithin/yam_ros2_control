// Leader-follower teleoperation for a pair of I2RT YAM arms: continuously
// commands a follower arm's joint_trajectory_controller to mirror a leader
// arm's /joint_states. Loosely mirrors i2rt/examples/minimum_gello/
// minimum_gello.py's leader-mode loop (raw 1:1 joint-position mirroring, no
// per-joint remapping), but ported onto ros2_control's joint_trajectory_
// controller instead of a direct motor-chain RPC call, and with additional
// safety behavior described below that the Python reference doesn't need
// (it has a physical deadman/teaching-handle button and a 400ms motor-
// firmware command-timeout watchdog; neither exists at this ROS layer).
//
// SAFETY MODEL - read before running on real hardware:
//
// 1. Zero-alignment is NOT verified by software. Every joint's raw position
//    is a firmware-persisted absolute encoder value (see i2rt_can_driver's
//    DmChain/DmMotorInterface and i2rt/motor_config_tool/set_zero.py) that
//    is self-consistent for a GIVEN arm unit across power cycles, but
//    nothing in this repo (big_yam_ros2_control.xacro's offset=0.0 for every
//    joint, on every arm unit, is a hardcoded constant, not a per-unit
//    calibration) or in the upstream i2rt Python config guarantees that two
//    DIFFERENT arm units agree on what "joint position 0" means physically.
//    This node therefore REFUSES to command the follower at all unless the
//    `alignment_confirmed` parameter is explicitly set true - the operator
//    must first manually verify the two arms agree (e.g. command both to a
//    known pose such as all-zero or the "home" SRDF group state, with the
//    follower's own controller directly, and visually confirm they match)
//    before setting this. This mirrors the same kind of manual "Alignment
//    Confirmed" gate i2rt's own viser_control_interface.py uses before
//    enabling a real arm from a simulated pose.
//
// 2. Startup transition is a slow, explicit ramp, not a snap. On first
//    receiving both arms' current joint states, the follower is commanded
//    through a multi-waypoint trajectory from its OWN current position to
//    the leader's CURRENT position over startup_ramp_duration_s (extended
//    automatically if that duration would imply exceeding
//    max_joint_velocity for how far apart the two poses happen to be).
//    Continuous mirroring only begins once that ramp completes.
//
// 3. Every commanded step thereafter (steady-state mirroring, and recovery
//    from a stale/lost leader stream) is clamped to max_joint_velocity
//    relative to the last position actually commanded - so a leader glitch,
//    a large single jump, or reconnecting after a stale period can't snap
//    the follower; it always ramps at a bounded rate.
//
// 4. If the leader's /joint_states goes stale (no message within
//    leader_timeout_s), the follower simply stops receiving new commands -
//    joint_trajectory_controller holds the last commanded position, rather
//    than this node extrapolating or repeating a frozen target.
//
// 5. Every target is clamped to [joint_lower_limits, joint_upper_limits]
//    before being sent, independent of whatever limits the leader itself
//    is respecting.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace i2rt_teleop
{

class LeaderFollowerNode : public rclcpp::Node
{
public:
  LeaderFollowerNode() : Node("leader_follower_node")
  {
    mirrored_joints_ = declare_parameter<std::vector<std::string>>(
      "mirrored_joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    // Defaults below are big_yam's arm position limits from
    // i2rt_description/config/joint_limits_big_yam.yaml - that file's own
    // header notes they're ported from a model and not yet verified against
    // real firmware limits, so treat these defaults with the same caution;
    // override via parameters if you have better-known limits.
    joint_lower_limits_ = declare_parameter<std::vector<double>>(
      "joint_lower_limits", {-2.61799, 0.0, 0.0, -1.5708, -1.5708, -2.0944});
    joint_upper_limits_ = declare_parameter<std::vector<double>>(
      "joint_upper_limits", {3.05433, 3.14159, 3.14159, 1.5708, 1.5708, 2.0944});
    max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 1.0);
    // Per-joint override, e.g. for a joint lifting a lot of arm mass with
    // little torque headroom left over gravity-holding, where tracking at
    // the same rad/s as light joints can push it into saturation. Leave
    // empty (default) to use max_joint_velocity for every joint.
    max_joint_velocity_per_joint_ =
      declare_parameter<std::vector<double>>("max_joint_velocity_per_joint", std::vector<double>{});
    leader_joint_states_topic_ = declare_parameter<std::string>("leader_joint_states_topic", "/leader/joint_states");
    follower_joint_states_topic_ =
      declare_parameter<std::string>("follower_joint_states_topic", "/follower/joint_states");
    follower_trajectory_topic_ = declare_parameter<std::string>(
      "follower_trajectory_topic", "/follower/joint_trajectory_controller/joint_trajectory");
    startup_ramp_duration_s_ = declare_parameter<double>("startup_ramp_duration_s", 4.0);
    startup_ramp_waypoints_ = declare_parameter<int>("startup_ramp_waypoints", 40);
    mirror_rate_hz_ = declare_parameter<double>("mirror_rate_hz", 30.0);
    mirror_time_from_start_s_ = declare_parameter<double>("mirror_time_from_start_s", 0.15);
    leader_timeout_s_ = declare_parameter<double>("leader_timeout_s", 0.5);
    alignment_confirmed_ = declare_parameter<bool>("alignment_confirmed", false);

    if (joint_lower_limits_.size() != mirrored_joints_.size() ||
        joint_upper_limits_.size() != mirrored_joints_.size()) {
      throw std::runtime_error(
        "joint_lower_limits/joint_upper_limits must be the same length as mirrored_joints");
    }
    if (mirror_rate_hz_ <= 0.0 || max_joint_velocity_ <= 0.0 || leader_timeout_s_ <= 0.0 ||
        startup_ramp_duration_s_ < 0.0 || startup_ramp_waypoints_ < 1) {
      throw std::runtime_error(
        "mirror_rate_hz, max_joint_velocity, leader_timeout_s must be > 0, startup_ramp_duration_s must be >= 0, "
        "and startup_ramp_waypoints must be >= 1");
    }
    if (!max_joint_velocity_per_joint_.empty() && max_joint_velocity_per_joint_.size() != mirrored_joints_.size()) {
      throw std::runtime_error("max_joint_velocity_per_joint, if set, must be the same length as mirrored_joints");
    }
    max_step_per_joint_.resize(mirrored_joints_.size());
    for (size_t i = 0; i < mirrored_joints_.size(); ++i) {
      const double limit = max_joint_velocity_per_joint_.empty() ? max_joint_velocity_ : max_joint_velocity_per_joint_[i];
      if (limit <= 0.0) {
        throw std::runtime_error("every entry in max_joint_velocity_per_joint must be > 0");
      }
      max_step_per_joint_[i] = limit / mirror_rate_hz_;
    }

    follower_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(follower_trajectory_topic_, 10);

    if (!alignment_confirmed_) {
      RCLCPP_ERROR(
        get_logger(),
        "alignment_confirmed is false: this node will NOT command the follower. Nothing in this repo verifies "
        "that two different arm units agree on what joint position 0 means (see this file's class comment). "
        "Before setting alignment_confirmed:=true: command BOTH arms to the same known pose (e.g. all-zero, or "
        "the 'home' SRDF group state) independently and visually confirm they physically match. Only then is "
        "mirroring this node's joint values between them safe.");
      reminder_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {
        RCLCPP_ERROR(get_logger(), "Still inert: alignment_confirmed is false (see startup log for why).");
      });
      return;
    }

    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      leader_joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { leader_callback(msg); });
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      follower_joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { follower_callback(msg); });
    mirror_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / mirror_rate_hz_), [this]() { mirror_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Waiting for initial joint states from leader ('%s') and follower ('%s') before starting the startup "
      "sync ramp...",
      leader_joint_states_topic_.c_str(), follower_joint_states_topic_.c_str());
  }

private:
  enum class State { kWaitingForInitialStates, kRamping, kMirroring };

  // Looks up `names` in `values` by key; returns false (leaving `out`
  // untouched) if any name is missing, so callers can wait for full
  // coverage instead of publishing a partial/malformed trajectory.
  static bool extract_in_order(
    const std::unordered_map<std::string, double> & values, const std::vector<std::string> & names,
    std::vector<double> & out)
  {
    std::vector<double> result;
    result.reserve(names.size());
    for (const auto & name : names) {
      const auto it = values.find(name);
      if (it == values.end()) {
        return false;
      }
      result.push_back(it->second);
    }
    out = std::move(result);
    return true;
  }

  void leader_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      latest_leader_positions_[msg->name[i]] = msg->position[i];
    }
    last_leader_msg_time_ = now();

    if (state_ == State::kWaitingForInitialStates) {
      maybe_start_ramp();
    }
  }

  void follower_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (have_follower_initial_) {
      return;  // Only need the follower's pose once, to seed the startup ramp.
    }
    std::unordered_map<std::string, double> values;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      values[msg->name[i]] = msg->position[i];
    }
    std::vector<double> ordered;
    if (!extract_in_order(values, mirrored_joints_, ordered)) {
      return;  // Wait for a message that reports every mirrored joint.
    }
    follower_initial_positions_ = std::move(ordered);
    have_follower_initial_ = true;
    RCLCPP_INFO(get_logger(), "Received follower's initial joint state.");

    if (state_ == State::kWaitingForInitialStates) {
      maybe_start_ramp();
    }
  }

  void maybe_start_ramp()
  {
    if (!have_follower_initial_) {
      return;
    }
    std::vector<double> leader_now;
    if (!extract_in_order(latest_leader_positions_, mirrored_joints_, leader_now)) {
      return;  // Wait for a leader message that reports every mirrored joint.
    }
    start_ramp(*follower_initial_positions_, leader_now);
  }

  void start_ramp(const std::vector<double> & start, const std::vector<double> & end)
  {
    double max_delta = 0.0;
    double min_duration_for_velocity = 0.0;
    for (size_t i = 0; i < start.size(); ++i) {
      const double delta = std::abs(end[i] - start[i]);
      max_delta = std::max(max_delta, delta);
      // Never let a large start/end gap imply a joint velocity above that
      // joint's configured limit, even if that means running longer than
      // startup_ramp_duration_s_. max_step_per_joint_ is already per-joint
      // (see max_joint_velocity_per_joint), so a joint with a lower limit
      // (e.g. one with little torque headroom over gravity-holding) sizes
      // the ramp correctly even if other joints could go faster.
      min_duration_for_velocity = std::max(min_duration_for_velocity, delta / (max_step_per_joint_[i] * mirror_rate_hz_));
    }
    const double duration = std::max(startup_ramp_duration_s_, min_duration_for_velocity);

    RCLCPP_WARN(
      get_logger(),
      "Starting startup sync ramp: follower will move from its current pose to the leader's current pose over "
      "%.2fs (%d waypoints, max single-joint travel %.3f rad). Keep the workspace clear.",
      duration, startup_ramp_waypoints_, max_delta);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = mirrored_joints_;
    const int n = startup_ramp_waypoints_;
    for (int step = 1; step <= n; ++step) {
      const double frac = static_cast<double>(step) / static_cast<double>(n);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(start.size());
      for (size_t j = 0; j < start.size(); ++j) {
        point.positions[j] = start[j] + frac * (end[j] - start[j]);
      }
      point.time_from_start = rclcpp::Duration::from_seconds(frac * duration);
      traj.points.push_back(point);
    }
    follower_traj_pub_->publish(traj);

    last_commanded_positions_ = end;
    state_ = State::kRamping;

    ramp_complete_timer_ = create_wall_timer(std::chrono::duration<double>(duration), [this]() {
      state_ = State::kMirroring;
      ramp_complete_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Startup sync ramp complete; now continuously mirroring leader to follower.");
    });
  }

  void mirror_tick()
  {
    if (state_ != State::kMirroring) {
      return;
    }

    const double stale_s = (now() - last_leader_msg_time_).seconds();
    if (stale_s > leader_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No leader joint state for %.2fs (timeout %.2fs); holding follower.",
        stale_s, leader_timeout_s_);
      return;
    }

    std::vector<double> target;
    if (!extract_in_order(latest_leader_positions_, mirrored_joints_, target)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Leader joint state missing one or more mirrored joints; skipping.");
      return;
    }

    bool clamped_to_limits = false;
    for (size_t i = 0; i < target.size(); ++i) {
      const double bounded = std::clamp(target[i], joint_lower_limits_[i], joint_upper_limits_[i]);
      if (bounded != target[i]) {
        clamped_to_limits = true;
      }
      target[i] = bounded;
    }
    if (clamped_to_limits) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Leader position outside the follower's configured joint limits; clamped.");
    }

    // Bound every step to max_step_per_joint_ relative to what was actually
    // last commanded - this is what protects against a leader glitch, an
    // unusually large single jump, or catching up smoothly after the
    // staleness check above skipped some cycles, uniformly.
    for (size_t i = 0; i < target.size(); ++i) {
      const double delta = target[i] - last_commanded_positions_[i];
      if (std::abs(delta) > max_step_per_joint_[i]) {
        target[i] = last_commanded_positions_[i] + std::copysign(max_step_per_joint_[i], delta);
      }
    }

    // Report the velocity our own step implies, not just a bare position -
    // without this, joint_trajectory_controller has no choice but to spline
    // toward zero velocity at every single message's short time_from_start,
    // then re-accelerate from rest on the next one. That repeated brake/
    // re-accelerate pattern is invisible on a light joint but shows up as
    // exactly the "vibrates and struggles" symptom on a joint with little
    // torque headroom left over gravity-holding (e.g. one lifting a lot of
    // arm mass) — a continuous, non-zero velocity target lets the
    // controller spline smoothly instead of braking every cycle.
    std::vector<double> velocity(target.size());
    for (size_t i = 0; i < target.size(); ++i) {
      velocity[i] = (target[i] - last_commanded_positions_[i]) * mirror_rate_hz_;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = mirrored_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target;
    point.velocities = velocity;
    point.time_from_start = rclcpp::Duration::from_seconds(mirror_time_from_start_s_);
    traj.points.push_back(point);
    follower_traj_pub_->publish(traj);

    last_commanded_positions_ = target;
  }

  // Parameters.
  std::vector<std::string> mirrored_joints_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  double max_joint_velocity_ = 1.0;
  std::vector<double> max_joint_velocity_per_joint_;
  std::vector<double> max_step_per_joint_;  // precomputed: velocity limit / mirror_rate_hz_, per joint
  std::string leader_joint_states_topic_;
  std::string follower_joint_states_topic_;
  std::string follower_trajectory_topic_;
  double startup_ramp_duration_s_ = 4.0;
  int startup_ramp_waypoints_ = 40;
  double mirror_rate_hz_ = 30.0;
  double mirror_time_from_start_s_ = 0.15;
  double leader_timeout_s_ = 0.5;
  bool alignment_confirmed_ = false;

  // State.
  State state_ = State::kWaitingForInitialStates;
  std::unordered_map<std::string, double> latest_leader_positions_;
  rclcpp::Time last_leader_msg_time_;
  bool have_follower_initial_ = false;
  std::optional<std::vector<double>> follower_initial_positions_;
  std::vector<double> last_commanded_positions_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr follower_traj_pub_;
  rclcpp::TimerBase::SharedPtr mirror_timer_;
  rclcpp::TimerBase::SharedPtr ramp_complete_timer_;
  rclcpp::TimerBase::SharedPtr reminder_timer_;
};

}  // namespace i2rt_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<i2rt_teleop::LeaderFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
