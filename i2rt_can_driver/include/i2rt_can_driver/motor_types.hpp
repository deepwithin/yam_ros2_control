#pragma once

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

namespace i2rt_can_driver
{

// Converts an unsigned int (bits wide) to a float, given a value range. Mirrors
// i2rt/motor_drivers/utils.py::uint_to_float.
double uint_to_float(uint32_t x_int, double x_min, double x_max, int bits);

// Converts a float to an unsigned int (bits wide), given a value range, clamping
// out-of-range inputs. Mirrors i2rt/motor_drivers/utils.py::float_to_uint.
uint32_t float_to_uint(double x, double x_min, double x_max, int bits);

struct MotorConstants
{
  double position_min = -12.5;
  double position_max = 12.5;
  double velocity_min = -45.0;
  double velocity_max = 45.0;
  double torque_min = -54.0;
  double torque_max = 54.0;
  double kp_min = 0.0;
  double kp_max = 500.0;
  double kd_min = 0.0;
  double kd_max = 5.0;
};

enum class MotorType
{
  DM8009,
  DM4310,
  DM4310V,
  DM4340,
  DM6248,
  DMH6215,
  DMH6215MIT,
  DM3507,
  DM_FLOW_WHEEL,
};

// Throws std::invalid_argument if the string doesn't match a known motor type,
// mirroring MotorType.get_motor_constants raising ValueError in Python.
MotorType motor_type_from_string(const std::string & name);
std::string motor_type_to_string(MotorType type);
MotorConstants get_motor_constants(MotorType type);

// Mirrors i2rt/motor_drivers/utils.py::ReceiveMode. Determines the arbitration ID
// a motor's response frame is expected on, relative to its own CAN ID.
enum class ReceiveMode
{
  P16,
  Same,
  Zero,
  PlusOne,
};

int get_receive_id(ReceiveMode mode, int motor_id);
// Only defined for P16/Same/Zero — PlusOne has no inverse in the Python source either.
int receive_id_to_motor_id(ReceiveMode mode, int receive_id);

// Mirrors i2rt/motor_drivers/utils.py::MotorErrorCode.
namespace motor_error_code
{
constexpr int kDisabled = 0x0;
constexpr int kNormal = 0x1;
constexpr int kOverVoltage = 0x8;
constexpr int kUnderVoltage = 0x9;
constexpr int kOverCurrent = 0xA;
constexpr int kMosfetOverTemperature = 0xB;
constexpr int kMotorOverTemperature = 0xC;
constexpr int kLossCommunication = 0xD;
constexpr int kOverload = 0xE;

std::string get_error_message(int error_code);
}  // namespace motor_error_code

// Mirrors dm_driver.py::ControlMode. Only MIT is fully wired up end to end today;
// POS_VEL/VEL exist for parity with the Python source's frame-ID offset table.
enum class ControlMode
{
  MIT,
  PosVel,
  Vel,
};

int control_mode_id_offset(ControlMode mode);

}  // namespace i2rt_can_driver
