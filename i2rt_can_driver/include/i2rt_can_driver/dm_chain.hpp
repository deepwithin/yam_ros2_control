#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "i2rt_can_driver/can_transport.hpp"
#include "i2rt_can_driver/dm_motor_interface.hpp"

namespace i2rt_can_driver
{

// Sim-space (joint-space) command, mirrors dm_driver.py::MotorCmd.
struct JointCommand
{
  double pos = 0.0;
  double vel = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double torque = 0.0;
};

// Multi-motor DM chain: applies per-joint offset/direction, tracks unwrapped
// absolute position across the encoder's wraparound range, and retries a
// bounded number of times to clear motor errors before giving up. Mirrors
// dm_driver.py::DMChainCanInterface, minus the internal background thread —
// in ros2_control, controller_manager's own update loop calls read()/write()
// at the configured rate, so there is no need for a second periodic thread
// here; set_commands() is meant to be called synchronously once per cycle.
class DmChain
{
public:
  DmChain(
    std::vector<std::pair<int, MotorType>> motor_list, std::vector<double> motor_offset,
    std::vector<double> motor_direction, CanTransport & transport,
    ControlMode control_mode = ControlMode::MIT);

  // Drains stale frames and enables every motor in sequence, seeding the
  // absolute-position tracker. Call once during hardware activation.
  void enable();
  // Commands every motor off. Call during hardware deactivation.
  void disable();

  size_t size() const { return motor_list_.size(); }

  // Sends per-joint commands (sim/joint space) to all motors sequentially,
  // updating cached feedback and absolute-position tracking. On a motor
  // error, attempts recovery (clean error + re-enable + reverify) up to 3
  // times before rethrowing MotorErrorDetected.
  void set_commands(const std::vector<JointCommand> & commands);

  // Sim-space accessors reflecting the most recent set_commands() call.
  double joint_position(size_t idx) const;
  double joint_velocity(size_t idx) const;
  double joint_effort(size_t idx) const;
  const MotorFeedback & raw_feedback(size_t idx) const { return last_feedback_.at(idx); }

  // Shifts this joint's software zero to its current position, matching
  // dm_driver.py::DMChainCanInterface.set_zero_position.
  void set_zero_position(size_t idx);

private:
  std::vector<MotorFeedback> send_commands_once(const std::vector<JointCommand> & commands);
  void update_absolute_positions(const std::vector<MotorFeedback> & feedback);
  bool try_recover_motors(std::optional<std::vector<MotorFeedback>> feedback, int max_retries = 3);

  std::vector<std::pair<int, MotorType>> motor_list_;
  std::vector<double> motor_offset_;
  std::vector<double> motor_direction_;
  CanTransport & transport_;
  DmMotorInterface motor_interface_;

  bool absolute_positions_initialized_ = false;
  std::vector<double> absolute_positions_;
  std::vector<MotorFeedback> last_feedback_;
  std::vector<JointCommand> last_commands_;
};

}  // namespace i2rt_can_driver
