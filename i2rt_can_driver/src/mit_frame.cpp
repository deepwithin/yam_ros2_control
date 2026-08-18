#include "i2rt_can_driver/mit_frame.hpp"

#include <cstring>

namespace i2rt_can_driver
{

std::array<uint8_t, 8> pack_mit_frame(double pos, double vel, double kp, double kd, double torque, const MotorConstants & c)
{
  const uint32_t pos_tmp = float_to_uint(pos, c.position_min, c.position_max, 16);
  const uint32_t vel_tmp = float_to_uint(vel, c.velocity_min, c.velocity_max, 12);
  const uint32_t kp_tmp = float_to_uint(kp, c.kp_min, c.kp_max, 12);
  const uint32_t kd_tmp = float_to_uint(kd, c.kd_min, c.kd_max, 12);
  const uint32_t tor_tmp = float_to_uint(torque, c.torque_min, c.torque_max, 12);

  std::array<uint8_t, 8> data{};
  data[0] = (pos_tmp >> 8) & 0xFF;
  data[1] = pos_tmp & 0xFF;
  data[2] = (vel_tmp >> 4) & 0xFF;
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp & 0xFF;
  data[5] = (kd_tmp >> 4) & 0xFF;
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp & 0xFF;
  return data;
}

std::array<uint8_t, 8> pack_vel_frame(double vel)
{
  std::array<uint8_t, 8> data{};
  const float vel_f = static_cast<float>(vel);
  std::memcpy(data.data(), &vel_f, sizeof(vel_f));
  return data;
}

MitFeedbackRaw unpack_mit_feedback(const std::array<uint8_t, 8> & data, const MotorConstants & c)
{
  const int error_int = (data[0] & 0xF0) >> 4;
  const uint32_t p_int = (static_cast<uint32_t>(data[1]) << 8) | data[2];
  const uint32_t v_int = (static_cast<uint32_t>(data[3]) << 4) | (data[4] >> 4);
  const uint32_t t_int = (static_cast<uint32_t>(data[4] & 0xF) << 8) | data[5];

  MitFeedbackRaw fb;
  fb.error_code = error_int;
  fb.position = uint_to_float(p_int, c.position_min, c.position_max, 16);
  fb.velocity = uint_to_float(v_int, c.velocity_min, c.velocity_max, 12);
  fb.torque = uint_to_float(t_int, c.torque_min, c.torque_max, 12);
  fb.temperature_mos = static_cast<double>(data[6]);
  fb.temperature_rotor = static_cast<double>(data[7]);
  return fb;
}

}  // namespace i2rt_can_driver
