#pragma once

#include <string>
#include <vector>

#include "i2rt_can_driver/can_transport.hpp"
#include "i2rt_can_driver/motor_types.hpp"

namespace i2rt_can_driver
{

// Mirrors i2rt/motor_drivers/utils.py::FeedbackFrameInfo.
struct MotorFeedback
{
  int motor_id = 0;
  int error_code = motor_error_code::kDisabled;  // raw 4-bit DM error code
  std::string error_message;
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double temperature_mos = 0.0;
  double temperature_rotor = 0.0;
};

// Thrown by set_control/motor_on when a motor reports a non-normal error code
// and ignore_error is false. Mirrors the RuntimeError raised by
// DMSingleMotorCanInterface.parse_recv_message in dm_driver.py.
class MotorErrorDetected : public std::runtime_error
{
public:
  explicit MotorErrorDetected(const std::string & what) : std::runtime_error(what) {}
};

// Single-motor DM protocol driver riding on a CanTransport. Mirrors
// dm_driver.py::DMSingleMotorCanInterface, minus the CanInterface plumbing
// (that lives in CanTransport instead of being inherited).
class DmMotorInterface
{
public:
  explicit DmMotorInterface(CanTransport & transport, ControlMode control_mode = ControlMode::MIT);

  // Enables the motor, clearing and retrying on any reported error until it
  // comes up clean. Blocks until success or a CanCommunicationError is thrown
  // by the underlying transport.
  MotorFeedback motor_on(int motor_id, MotorType motor_type);
  void motor_off(int motor_id);
  void clean_error(int motor_id);

  // Sends one MIT-mode (or VEL-mode) control frame and returns the motor's
  // feedback. Throws MotorErrorDetected if the motor reports a non-normal
  // error code.
  MotorFeedback set_control(
    int motor_id, MotorType motor_type, double pos, double vel, double kp, double kd, double torque);

  MotorFeedback parse_recv_message(const CanFrame & frame, MotorType motor_type, bool ignore_error = false) const;

private:
  int frame_id(int motor_id) const { return id_offset_ + motor_id; }

  CanTransport & transport_;
  ControlMode control_mode_;
  int id_offset_;
};

}  // namespace i2rt_can_driver
