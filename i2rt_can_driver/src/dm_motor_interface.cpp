#include "i2rt_can_driver/dm_motor_interface.hpp"

#include <cstring>

#include "i2rt_can_driver/mit_frame.hpp"

namespace i2rt_can_driver
{

namespace
{
// Trailing command bytes for the DM enable/disable/clear-error frames.
// Mirrors the [0xFF]*7 + [0x..] payloads in dm_driver.py.
constexpr uint8_t kMotorOnCmd = 0xFC;
constexpr uint8_t kMotorOffCmd = 0xFD;
constexpr uint8_t kClearErrorCmd = 0xFB;

std::vector<uint8_t> command_frame(uint8_t trailing_byte)
{
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, trailing_byte};
}
}  // namespace

DmMotorInterface::DmMotorInterface(CanTransport & transport, ControlMode control_mode)
: transport_(transport), control_mode_(control_mode), id_offset_(control_mode_id_offset(control_mode))
{
}

void DmMotorInterface::clean_error(int motor_id)
{
  const auto data = command_frame(kClearErrorCmd);
  for (int attempt = 0; attempt < 3; ++attempt) {
    try {
      transport_.send_raw(motor_id, data);
    } catch (const CanCommunicationError &) {
      // Fire-and-forget, matching the Python source's swallow-and-retry loop.
    }
  }
}

void DmMotorInterface::motor_off(int motor_id)
{
  transport_.send_and_receive(frame_id(motor_id), motor_id, command_frame(kMotorOffCmd));
}

MotorFeedback DmMotorInterface::motor_on(int motor_id, MotorType motor_type)
{
  const auto data = command_frame(kMotorOnCmd);
  CanFrame frame = transport_.send_and_receive(motor_id, motor_id, data);
  MotorFeedback info = parse_recv_message(frame, MotorType::DM4310, /*ignore_error=*/true);

  while (info.error_code != motor_error_code::kNormal) {
    clean_error(motor_id);
    transport_.try_receive();
    frame = transport_.send_and_receive(motor_id, motor_id, data);
    info = parse_recv_message(frame, motor_type, /*ignore_error=*/true);
  }

  return parse_recv_message(frame, motor_type, /*ignore_error=*/false);
}

MotorFeedback DmMotorInterface::set_control(
  int motor_id, MotorType motor_type, double pos, double vel, double kp, double kd, double torque)
{
  std::array<uint8_t, 8> frame_data{};

  if (control_mode_ == ControlMode::MIT) {
    frame_data = pack_mit_frame(pos, vel, kp, kd, torque, get_motor_constants(motor_type));
  } else if (control_mode_ == ControlMode::Vel) {
    frame_data = pack_vel_frame(vel);
  }
  // ControlMode::PosVel has no packing branch in the Python source either —
  // frames are sent all-zero in that mode there, so we mirror that gap as-is.

  const std::vector<uint8_t> data(frame_data.begin(), frame_data.end());
  const CanFrame response = transport_.send_and_receive(frame_id(motor_id), motor_id, data, /*max_retry=*/15);
  return parse_recv_message(response, motor_type);
}

MotorFeedback DmMotorInterface::parse_recv_message(const CanFrame & frame, MotorType motor_type, bool ignore_error) const
{
  const MitFeedbackRaw raw = unpack_mit_feedback(frame.data, get_motor_constants(motor_type));
  const std::string error_message = motor_error_code::get_error_message(raw.error_code);
  const int motor_id_of_response = receive_id_to_motor_id(transport_.receive_mode(), static_cast<int>(frame.arbitration_id));

  if (raw.error_code != motor_error_code::kNormal && !ignore_error) {
    throw MotorErrorDetected(
      "Motor error detected: motor id: " + std::to_string(motor_id_of_response) + ", error: " + error_message);
  }

  MotorFeedback info;
  info.motor_id = motor_id_of_response;
  info.error_code = raw.error_code;
  info.error_message = error_message;
  info.position = raw.position;
  info.velocity = raw.velocity;
  info.torque = raw.torque;
  info.temperature_mos = raw.temperature_mos;
  info.temperature_rotor = raw.temperature_rotor;
  return info;
}

}  // namespace i2rt_can_driver
