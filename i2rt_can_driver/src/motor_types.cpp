#include "i2rt_can_driver/motor_types.hpp"

#include <algorithm>
#include <cmath>

namespace i2rt_can_driver
{

double uint_to_float(uint32_t x_int, double x_min, double x_max, int bits)
{
  const double span = x_max - x_min;
  const uint32_t max_int = (1u << bits) - 1u;
  return (static_cast<double>(x_int) * span / static_cast<double>(max_int)) + x_min;
}

uint32_t float_to_uint(double x, double x_min, double x_max, int bits)
{
  if (!std::isfinite(x)) {
    // Casting NaN/Inf to an integer is undefined behavior — on at least one
    // platform this was observed to silently produce 0, which then decodes
    // as x_min (e.g. a joint's minimum position limit) instead of raising an
    // error. A caller feeding this a live ros2_control command interface
    // that hasn't been claimed by any controller yet (which reads as NaN)
    // must catch that itself; this is the last line of defense so it fails
    // loudly instead of commanding a silently wrong value to a motor.
    throw std::invalid_argument("float_to_uint: x is not finite (" + std::to_string(x) + ")");
  }
  const double span = x_max - x_min;
  x = std::min(x, x_max);
  x = std::max(x, x_min);
  const uint32_t max_int = (1u << bits) - 1u;
  return static_cast<uint32_t>((x - x_min) * static_cast<double>(max_int) / span);
}

MotorType motor_type_from_string(const std::string & name)
{
  static const std::map<std::string, MotorType> kNames = {
    {"DM8009", MotorType::DM8009},
    {"DM4310", MotorType::DM4310},
    {"DM4310V", MotorType::DM4310V},
    {"DM4340", MotorType::DM4340},
    {"DM6248", MotorType::DM6248},
    {"DMH6215", MotorType::DMH6215},
    {"DMH6215MIT", MotorType::DMH6215MIT},
    {"DM3507", MotorType::DM3507},
    {"DM_FLOW_WHEEL", MotorType::DM_FLOW_WHEEL},
  };
  const auto it = kNames.find(name);
  if (it == kNames.end()) {
    throw std::invalid_argument("Motor type '" + name + "' not recognized.");
  }
  return it->second;
}

std::string motor_type_to_string(MotorType type)
{
  switch (type) {
    case MotorType::DM8009: return "DM8009";
    case MotorType::DM4310: return "DM4310";
    case MotorType::DM4310V: return "DM4310V";
    case MotorType::DM4340: return "DM4340";
    case MotorType::DM6248: return "DM6248";
    case MotorType::DMH6215: return "DMH6215";
    case MotorType::DMH6215MIT: return "DMH6215MIT";
    case MotorType::DM3507: return "DM3507";
    case MotorType::DM_FLOW_WHEEL: return "DM_FLOW_WHEEL";
  }
  throw std::invalid_argument("Unhandled MotorType enum value.");
}

MotorConstants get_motor_constants(MotorType type)
{
  // Values mirror i2rt/motor_drivers/utils.py::MotorType.get_motor_constants exactly.
  switch (type) {
    case MotorType::DM8009:
      return MotorConstants{-12.5, 12.5, -45, 45, -54, 54, 0.0, 500.0, 0.0, 5.0};
    case MotorType::DM4310:
      return MotorConstants{-12.5, 12.5, -30, 30, -10, 10, 0.0, 500.0, 0.0, 5.0};
    case MotorType::DM4310V:
    case MotorType::DM_FLOW_WHEEL:
    case MotorType::DMH6215:
      return MotorConstants{-3.1415926, 3.1415926, -30, 30, -10, 10, 0.0, 500.0, 0.0, 5.0};
    case MotorType::DM4340:
      return MotorConstants{-12.5, 12.5, -10, 10, -28, 28, 0.0, 500.0, 0.0, 5.0};
    case MotorType::DM6248:
      return MotorConstants{-12.5, 12.5, -20, 20, -120, 120, 0.0, 500.0, 0.0, 5.0};
    case MotorType::DMH6215MIT:
      return MotorConstants{-12.5, 12.5, -45, 45, -10, 10, 0.0, 500.0, 0.0, 5.0};
    case MotorType::DM3507:
      return MotorConstants{-12.5, 12.5, -50, 50, -5, 5, 0.0, 500.0, 0.0, 5.0};
  }
  throw std::invalid_argument("Unhandled MotorType enum value.");
}

int get_receive_id(ReceiveMode mode, int motor_id)
{
  switch (mode) {
    case ReceiveMode::P16: return motor_id + 16;
    case ReceiveMode::Same: return motor_id;
    case ReceiveMode::Zero: return 0;
    case ReceiveMode::PlusOne: return motor_id + 1;
  }
  throw std::invalid_argument("Unhandled ReceiveMode enum value.");
}

int receive_id_to_motor_id(ReceiveMode mode, int receive_id)
{
  switch (mode) {
    case ReceiveMode::P16: return receive_id - 16;
    case ReceiveMode::Same: return receive_id;
    case ReceiveMode::Zero: return 0;
    case ReceiveMode::PlusOne:
      throw std::invalid_argument("receive_mode PlusOne has no to_motor_id mapping");
  }
  throw std::invalid_argument("Unhandled ReceiveMode enum value.");
}

namespace motor_error_code
{
std::string get_error_message(int error_code)
{
  static const std::map<int, std::string> kMessages = {
    {kNormal, "normal"},
    {kDisabled, "disabled"},
    {kOverVoltage, "over voltage"},
    {kUnderVoltage, "under voltage"},
    {kOverCurrent, "over current"},
    {kMosfetOverTemperature, "mosfet over temperature"},
    {kMotorOverTemperature, "motor over temperature"},
    {kLossCommunication, "loss communication"},
    {kOverload, "overload"},
  };
  const auto it = kMessages.find(error_code);
  if (it == kMessages.end()) {
    return "Unknown error code: " + std::to_string(error_code);
  }
  return it->second;
}
}  // namespace motor_error_code

int control_mode_id_offset(ControlMode mode)
{
  switch (mode) {
    case ControlMode::MIT: return 0x000;
    case ControlMode::PosVel: return 0x100;
    case ControlMode::Vel: return 0x200;
  }
  throw std::invalid_argument("Unhandled ControlMode enum value.");
}

}  // namespace i2rt_can_driver
