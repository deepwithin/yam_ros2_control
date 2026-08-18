#pragma once

#include <array>
#include <cstdint>

#include "i2rt_can_driver/motor_types.hpp"

namespace i2rt_can_driver
{

// Raw, receive-mode-agnostic decode of a DM feedback frame's 8 data bytes.
// Pulled out of DmMotorInterface::parse_recv_message so the bit-packing math
// is unit-testable without a live CAN socket.
struct MitFeedbackRaw
{
  int error_code = 0;
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double temperature_mos = 0.0;
  double temperature_rotor = 0.0;
};

// Packs a MIT-mode command frame. Mirrors the MIT branch of
// dm_driver.py::DMSingleMotorCanInterface.set_control exactly.
std::array<uint8_t, 8> pack_mit_frame(
  double pos, double vel, double kp, double kd, double torque, const MotorConstants & c);

// Packs a VEL-mode command frame (only the velocity command is honored).
// Mirrors the ControlMode.VEL branch of the same function.
std::array<uint8_t, 8> pack_vel_frame(double vel);

// Decodes a DM feedback frame's 8 data bytes. Mirrors
// DMSingleMotorCanInterface.parse_recv_message's bit math (error code,
// position/velocity/torque, temperatures), independent of receive-mode id
// bookkeeping.
MitFeedbackRaw unpack_mit_feedback(const std::array<uint8_t, 8> & data, const MotorConstants & c);

}  // namespace i2rt_can_driver
