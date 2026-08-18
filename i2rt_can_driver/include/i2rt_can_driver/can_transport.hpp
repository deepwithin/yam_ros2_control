#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "i2rt_can_driver/motor_types.hpp"

namespace i2rt_can_driver
{

enum class LogLevel
{
  Info,
  Warning,
  Error,
};

using LogCallback = std::function<void(LogLevel, const std::string &)>;

// Thrown when a motor fails to respond after all retries are exhausted.
// Mirrors the AssertionError raised by CanInterface._send_message_get_response
// in i2rt/motor_drivers/can_interface.py.
class CanCommunicationError : public std::runtime_error
{
public:
  explicit CanCommunicationError(const std::string & what) : std::runtime_error(what) {}
};

struct CanFrame
{
  uint32_t arbitration_id = 0;
  uint8_t len = 0;
  std::array<uint8_t, 8> data{};
};

// Thin SocketCAN transport with the same request/response + retry semantics as
// i2rt/motor_drivers/can_interface.py::CanInterface. Standard (11-bit) CAN 2.0A
// frames only, matching is_extended_id=False in the Python source.
class CanTransport
{
public:
  explicit CanTransport(
    std::string channel, ReceiveMode receive_mode = ReceiveMode::P16,
    std::string name = "default_can_interface");

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;

  ~CanTransport();

  void close();

  void set_log_callback(LogCallback callback) { log_callback_ = std::move(callback); }

  // Sends `data` to `arbitration_id` and waits for a response matching
  // `expected_id` (defaults to receive_mode.get_receive_id(motor_id)), retrying
  // up to `max_retry` times. Throws CanCommunicationError if no matching
  // response arrives. Mirrors _send_message_get_response.
  CanFrame send_and_receive(
    int arbitration_id, int motor_id, const std::vector<uint8_t> & data, int max_retry = 5,
    std::optional<int> expected_id = std::nullopt);

  // Fire-and-forget send with no response wait, matching the raw `self.bus.send`
  // calls in DMSingleMotorCanInterface.clean_error.
  void send_raw(int arbitration_id, const std::vector<uint8_t> & data);

  // Non-throwing receive; returns std::nullopt on timeout instead of raising.
  // Mirrors CanInterface.try_receive_message.
  std::optional<CanFrame> try_receive(std::optional<int> motor_id = std::nullopt, double timeout = 0.009);

  // Drains pending frames until the bus is idle. Mirrors CanInterface._drain_bus.
  int drain_bus(double timeout_s = 0.05, int idle_count = 10);

  const std::string & channel() const { return channel_; }
  const std::string & name() const { return name_; }
  ReceiveMode receive_mode() const { return receive_mode_; }

private:
  std::optional<CanFrame> receive_message(
    std::optional<int> motor_id, double timeout, bool suppress_warning = false);
  void log(LogLevel level, const std::string & message) const;

  int fd_ = -1;
  std::string channel_;
  std::string name_;
  ReceiveMode receive_mode_;
  LogCallback log_callback_;
};

}  // namespace i2rt_can_driver
