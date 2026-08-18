#include "i2rt_can_driver/can_transport.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <thread>

namespace i2rt_can_driver
{

namespace
{
CanFrame to_can_frame(const can_frame & raw)
{
  CanFrame frame;
  frame.arbitration_id = raw.can_id & CAN_SFF_MASK;
  frame.len = raw.can_dlc;
  std::copy(raw.data, raw.data + 8, frame.data.begin());
  return frame;
}
}  // namespace

CanTransport::CanTransport(std::string channel, ReceiveMode receive_mode, std::string name)
: channel_(std::move(channel)), name_(std::move(name)), receive_mode_(receive_mode)
{
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw CanCommunicationError("Failed to open SocketCAN socket: " + std::string(std::strerror(errno)));
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw CanCommunicationError("CAN interface '" + channel_ + "' not found: " + std::string(std::strerror(errno)));
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw CanCommunicationError("Failed to bind to '" + channel_ + "': " + std::string(std::strerror(errno)));
  }
}

CanTransport::~CanTransport() { close(); }

void CanTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void CanTransport::log(LogLevel level, const std::string & message) const
{
  if (log_callback_) {
    log_callback_(level, message);
  }
}

void CanTransport::send_raw(int arbitration_id, const std::vector<uint8_t> & data)
{
  can_frame raw{};
  raw.can_id = static_cast<canid_t>(arbitration_id) & CAN_SFF_MASK;
  raw.can_dlc = static_cast<uint8_t>(std::min<size_t>(data.size(), 8));
  std::copy(data.begin(), data.begin() + raw.can_dlc, raw.data);

  if (write(fd_, &raw, sizeof(raw)) != sizeof(raw)) {
    throw CanCommunicationError(
      "CAN send failed on " + name_ + " channel " + channel_ + ": " + std::string(std::strerror(errno)));
  }
}

std::optional<CanFrame> CanTransport::receive_message(
  std::optional<int> motor_id, double timeout, bool suppress_warning)
{
  pollfd pfd{fd_, POLLIN, 0};
  const int timeout_ms = static_cast<int>(timeout * 1000.0);
  const int ready = poll(&pfd, 1, timeout_ms);
  if (ready > 0 && (pfd.revents & POLLIN)) {
    can_frame raw{};
    const ssize_t n = read(fd_, &raw, sizeof(raw));
    if (n == sizeof(raw)) {
      return to_can_frame(raw);
    }
  }
  if (!suppress_warning) {
    const std::string motor_str = motor_id ? std::to_string(*motor_id) : "unknown";
    log(
      LogLevel::Warning, name_ + ": Failed to receive message, motor id " + motor_str +
        " motor timeout. Check if the motor is powered on or if the motor ID exists.");
  }
  return std::nullopt;
}

std::optional<CanFrame> CanTransport::try_receive(std::optional<int> motor_id, double timeout)
{
  return receive_message(motor_id, timeout, /*suppress_warning=*/true);
}

int CanTransport::drain_bus(double timeout_s, int idle_count)
{
  int drained = 0;
  int idle = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline && idle < idle_count) {
    if (!try_receive(std::nullopt, 0.001)) {
      ++idle;
    } else {
      idle = 0;
      ++drained;
    }
  }
  return drained;
}

CanFrame CanTransport::send_and_receive(
  int arbitration_id, int motor_id, const std::vector<uint8_t> & data, int max_retry,
  std::optional<int> expected_id)
{
  const int resolved_expected_id = expected_id.value_or(get_receive_id(receive_mode_, motor_id));

  for (int attempt = 0; attempt < max_retry; ++attempt) {
    try {
      send_raw(arbitration_id, data);
      const auto response = receive_message(motor_id, 0.01, /*suppress_warning=*/true);
      if (response && static_cast<int>(response->arbitration_id) == resolved_expected_id) {
        return *response;
      }
      // Drain one stale frame before retrying, mirroring the Python source's
      // extra try_receive_message(id) call between attempts.
      try_receive(motor_id);
    } catch (const CanCommunicationError & e) {
      log(
        LogLevel::Warning, "CAN Error " + name_ + ": Failed to communicate with motor " +
          std::to_string(arbitration_id) + " over can bus. Retrying... (" + e.what() + ")");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  throw CanCommunicationError(
    "fail to communicate with the motor " + std::to_string(arbitration_id) + " on " + name_ + " at can channel " +
    channel_);
}

}  // namespace i2rt_can_driver
