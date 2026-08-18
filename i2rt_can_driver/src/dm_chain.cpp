#include "i2rt_can_driver/dm_chain.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <thread>

namespace i2rt_can_driver
{

namespace
{
// Python's `%` always returns a result with the sign of the divisor; C++'s
// std::fmod keeps the sign of the dividend. update_absolute_positions relies
// on Python's convention for the wrap-around math, so replicate it exactly.
double python_mod(double a, double b)
{
  double r = std::fmod(a, b);
  if (r != 0.0 && ((r < 0) != (b < 0))) {
    r += b;
  }
  return r;
}
}  // namespace

DmChain::DmChain(
  std::vector<std::pair<int, MotorType>> motor_list, std::vector<double> motor_offset,
  std::vector<double> motor_direction, CanTransport & transport, ControlMode control_mode)
: motor_list_(std::move(motor_list)),
  motor_offset_(std::move(motor_offset)),
  motor_direction_(std::move(motor_direction)),
  transport_(transport),
  motor_interface_(transport, control_mode)
{
  if (motor_list_.empty()) {
    throw std::invalid_argument("DmChain requires at least one motor");
  }
  if (motor_list_.size() != motor_offset_.size() || motor_list_.size() != motor_direction_.size()) {
    throw std::invalid_argument("motor_list, motor_offset, and motor_direction must be the same length");
  }
}

void DmChain::enable()
{
  transport_.drain_bus(0.05);
  std::vector<MotorFeedback> feedback;
  feedback.reserve(motor_list_.size());
  for (const auto & [motor_id, motor_type] : motor_list_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    feedback.push_back(motor_interface_.motor_on(motor_id, motor_type));
  }
  update_absolute_positions(feedback);
  last_feedback_ = std::move(feedback);
  last_commands_.assign(motor_list_.size(), JointCommand{});
  for (size_t idx = 0; idx < motor_list_.size(); ++idx) {
    last_commands_[idx].torque = last_feedback_[idx].torque;
  }
}

void DmChain::disable()
{
  for (const auto & [motor_id, motor_type] : motor_list_) {
    (void)motor_type;
    motor_interface_.motor_off(motor_id);
  }
}

void DmChain::update_absolute_positions(const std::vector<MotorFeedback> & feedback)
{
  if (!absolute_positions_initialized_) {
    absolute_positions_.assign(motor_list_.size(), 0.0);
    absolute_positions_initialized_ = true;
    for (size_t idx = 0; idx < motor_list_.size(); ++idx) {
      absolute_positions_[idx] = feedback[idx].position;
    }
    return;
  }

  for (size_t idx = 0; idx < motor_list_.size(); ++idx) {
    const MotorConstants c = get_motor_constants(motor_list_[idx].second);
    const double position_range = c.position_max - c.position_min;
    const double current_position = feedback[idx].position;
    const double previous_position = absolute_positions_[idx];

    double delta_position = current_position - python_mod(previous_position, position_range);
    if (delta_position > position_range / 2.0) {
      delta_position -= position_range;
    } else if (delta_position < -position_range / 2.0) {
      delta_position += position_range;
    }
    absolute_positions_[idx] += delta_position;
  }
}

std::vector<MotorFeedback> DmChain::send_commands_once(const std::vector<JointCommand> & commands)
{
  std::vector<MotorFeedback> feedback;
  feedback.reserve(motor_list_.size());
  for (size_t idx = 0; idx < motor_list_.size(); ++idx) {
    const auto & [motor_id, motor_type] = motor_list_[idx];
    const double torque = commands[idx].torque * motor_direction_[idx];
    const double pos = commands[idx].pos * motor_direction_[idx] + motor_offset_[idx];
    const double vel = commands[idx].vel * motor_direction_[idx];
    feedback.push_back(
      motor_interface_.set_control(motor_id, motor_type, pos, vel, commands[idx].kp, commands[idx].kd, torque));
  }
  return feedback;
}

void DmChain::set_commands(const std::vector<JointCommand> & commands)
{
  if (commands.size() != motor_list_.size()) {
    throw std::invalid_argument("set_commands: command count does not match motor count");
  }
  last_commands_ = commands;
  try {
    auto feedback = send_commands_once(commands);
    update_absolute_positions(feedback);
    last_feedback_ = std::move(feedback);
  } catch (const MotorErrorDetected &) {
    // Mirrors dm_driver.py's control-loop recovery: a motor error aborts the
    // cycle with no partial feedback, so recovery targets every motor.
    if (!try_recover_motors(std::nullopt)) {
      throw;
    }
  }
}

bool DmChain::try_recover_motors(std::optional<std::vector<MotorFeedback>> feedback, int max_retries)
{
  for (int attempt = 0; attempt < max_retries; ++attempt) {
    std::vector<size_t> error_indices;
    if (feedback) {
      for (size_t idx = 0; idx < feedback->size(); ++idx) {
        if ((*feedback)[idx].error_code != motor_error_code::kNormal) {
          error_indices.push_back(idx);
        }
      }
    } else {
      error_indices.resize(motor_list_.size());
      std::iota(error_indices.begin(), error_indices.end(), 0);
    }
    if (error_indices.empty()) {
      return true;
    }

    for (size_t idx : error_indices) {
      const auto & [motor_id, motor_type] = motor_list_[idx];
      motor_interface_.clean_error(motor_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
      transport_.try_receive(std::nullopt, 0.002);
      try {
        motor_interface_.motor_on(motor_id, motor_type);
      } catch (const std::exception &) {
        continue;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    try {
      auto recovered = send_commands_once(last_commands_);
      const bool all_ok = std::all_of(
        recovered.begin(), recovered.end(),
        [](const MotorFeedback & fb) { return fb.error_code == motor_error_code::kNormal; });
      if (all_ok) {
        update_absolute_positions(recovered);
        last_feedback_ = recovered;
        return true;
      }
      feedback = std::move(recovered);
    } catch (const MotorErrorDetected &) {
      continue;
    }
  }
  return false;
}

double DmChain::joint_position(size_t idx) const
{
  return (absolute_positions_.at(idx) - motor_offset_.at(idx)) * motor_direction_.at(idx);
}

double DmChain::joint_velocity(size_t idx) const { return last_feedback_.at(idx).velocity * motor_direction_.at(idx); }

double DmChain::joint_effort(size_t idx) const { return last_feedback_.at(idx).torque * motor_direction_.at(idx); }

void DmChain::set_zero_position(size_t idx) { motor_offset_.at(idx) = absolute_positions_.at(idx); }

}  // namespace i2rt_can_driver
