// Cross-checks the C++ port against ground-truth values computed by actually
// running i2rt/motor_drivers/{utils.py,dm_driver.py} in Python (see the
// command used to generate these vectors in the port's commit message /
// PR description). Any divergence here means the C++ math no longer matches
// the motors' real wire protocol.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "i2rt_can_driver/mit_frame.hpp"
#include "i2rt_can_driver/motor_types.hpp"

using namespace i2rt_can_driver;

TEST(FloatUintConversion, RejectsNonFiniteInput)
{
  // Regression test: a NaN command (e.g. an unclaimed ros2_control command
  // interface) must never silently reach the uint cast — see
  // float_to_uint's comment for the incident this guards against.
  EXPECT_THROW(float_to_uint(std::numeric_limits<double>::quiet_NaN(), -12.5, 12.5, 16), std::invalid_argument);
  EXPECT_THROW(float_to_uint(std::numeric_limits<double>::infinity(), -12.5, 12.5, 16), std::invalid_argument);
}

TEST(FloatUintConversion, RoundTripMatchesPython)
{
  // float_to_uint(0.0, -12.5, 12.5, 16) == 32767
  EXPECT_EQ(float_to_uint(0.0, -12.5, 12.5, 16), 32767u);
  EXPECT_NEAR(uint_to_float(32767, -12.5, 12.5, 16), -0.0001907377737087046, 1e-12);

  EXPECT_EQ(float_to_uint(3.14159, -12.5, 12.5, 16), 41002u);
  EXPECT_NEAR(uint_to_float(41002, -12.5, 12.5, 16), 3.1412603952086666, 1e-12);
}

TEST(FloatUintConversion, ClampsOutOfRangeInputs)
{
  // float_to_uint(-30.0, -30, 30, 12) == 0 (at the low boundary)
  EXPECT_EQ(float_to_uint(-30.0, -30, 30, 12), 0u);
  EXPECT_DOUBLE_EQ(uint_to_float(0, -30, 30, 12), -30.0);

  // float_to_uint(100.0, -30, 30, 12) == 4095 (clamped to the high boundary)
  EXPECT_EQ(float_to_uint(100.0, -30, 30, 12), 4095u);
  EXPECT_DOUBLE_EQ(uint_to_float(4095, -30, 30, 12), 30.0);
}

TEST(MitFramePacking, MatchesPythonReferenceBytes)
{
  const MotorConstants c = get_motor_constants(MotorType::DM4310);
  const auto data = pack_mit_frame(/*pos=*/1.0, /*vel=*/2.0, /*kp=*/50.0, /*kd=*/1.0, /*torque=*/3.0, c);

  // Reference bytes from the Python DM4310 packing branch in dm_driver.py.
  const std::array<uint8_t, 8> expected = {138, 60, 136, 129, 153, 51, 58, 101};
  EXPECT_EQ(data, expected);
}

TEST(MitFrameUnpacking, RoundTripsThroughPackedBytesLikePython)
{
  // Feeding the packed command bytes back through the feedback-frame decode
  // isn't physically meaningful (command and feedback frames encode
  // different quantities) — it only checks that the C++ decode arithmetic
  // matches Python's bit-for-bit given the same input bytes.
  const MotorConstants c = get_motor_constants(MotorType::DM4310);
  const std::array<uint8_t, 8> data = {138, 60, 136, 129, 153, 51, 58, 101};

  const MitFeedbackRaw fb = unpack_mit_feedback(data, c);
  EXPECT_EQ(fb.error_code, 8);
  EXPECT_NEAR(fb.position, -6.588654917219806, 1e-12);
  EXPECT_NEAR(fb.velocity, 0.3736263736263723, 1e-12);
  EXPECT_NEAR(fb.torque, 1.5018315018315018, 1e-12);
  EXPECT_DOUBLE_EQ(fb.temperature_mos, 58.0);
  EXPECT_DOUBLE_EQ(fb.temperature_rotor, 101.0);
}

TEST(ReceiveMode, P16MatchesPython)
{
  EXPECT_EQ(get_receive_id(ReceiveMode::P16, 3), 19);
  EXPECT_EQ(receive_id_to_motor_id(ReceiveMode::P16, 19), 3);
}

TEST(MotorErrorCode, MessagesMatchPython)
{
  EXPECT_EQ(motor_error_code::get_error_message(0x1), "normal");
  EXPECT_EQ(motor_error_code::get_error_message(0xB), "mosfet over temperature");
}

TEST(MotorType, UnknownStringThrows)
{
  EXPECT_THROW(motor_type_from_string("NOT_A_MOTOR"), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
