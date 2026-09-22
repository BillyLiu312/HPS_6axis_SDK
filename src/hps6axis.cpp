#include "hps6axis/hps6axis.hpp"

#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <cstdio>
#include <termios.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <utility>
#include <exception>

namespace hps6axis {
namespace {
constexpr uint8_t kHead0 = 0xF6;
constexpr uint8_t kHead1 = 0x6F;
constexpr uint8_t kTail0 = 0x6F;
constexpr uint8_t kTail1 = 0xF6;

speed_t baudConstant(uint32_t baud) {
  switch (baud) {
    case 9600: return B9600;
    case 115200: return B115200;
#ifdef B128000
    case 128000: return B128000;
#endif
#ifdef B256000
    case 256000: return B256000;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B500000
    case 500000: return B500000;
#endif
#ifdef B576000
    case 576000: return B576000;
#endif
#ifdef B600000
    case 600000: return B600000;
#endif
#ifdef B750000
    case 750000: return B750000;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
    default: throw Error("unsupported baud rate on this platform: " + std::to_string(baud));
  }
}

int64_t deadlineMs(int timeout_ms) {
  return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count()) + timeout_ms;
}

int remainingMs(int64_t deadline) {
  const auto now = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  return static_cast<int>(std::max<int64_t>(0, deadline - now));
}

uint32_t readU32(const std::vector<uint8_t>& p, size_t i) {
  return static_cast<uint32_t>(p[i]) | (static_cast<uint32_t>(p[i + 1]) << 8) |
         (static_cast<uint32_t>(p[i + 2]) << 16) | (static_cast<uint32_t>(p[i + 3]) << 24);
}

template <typename First, typename Second>
void runParallel(First first, Second second) {
  std::exception_ptr first_error;
  std::exception_ptr second_error;
  std::thread first_thread([&] {
    try { first(); } catch (...) { first_error = std::current_exception(); }
  });
  std::thread second_thread([&] {
    try { second(); } catch (...) { second_error = std::current_exception(); }
  });
  first_thread.join();
  second_thread.join();
  if (first_error) std::rethrow_exception(first_error);
  if (second_error) std::rethrow_exception(second_error);
}

}  // namespace

Sensor::Sensor(SerialConfig config) : config_(std::move(config)) {}

Sensor::~Sensor() {
  close();
}

void Sensor::open() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) return;
  fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) throw Error("failed to open " + config_.device + ": " + std::strerror(errno));
  try {
    configurePort(config_.baudrate);
    tcflush(fd_, TCIOFLUSH);
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}

void Sensor::close() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Sensor::isOpen() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

void Sensor::ensureOpen() const {
  if (fd_ < 0) throw Error("sensor serial port is not open");
}

void Sensor::configurePort(uint32_t baudrate) {
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) throw Error("tcgetattr failed: " + std::string(std::strerror(errno)));
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD | PARENB;
  tty.c_cflag &= static_cast<tcflag_t>(~PARODD);  // 8 data bits, even parity, 1 stop bit
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  const speed_t speed = baudConstant(baudrate);
  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0)
    throw Error("failed to set baud rate: " + std::string(std::strerror(errno)));
  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    throw Error("tcsetattr failed: " + std::string(std::strerror(errno)));
}

void Sensor::writeAll(const uint8_t* data, size_t size) {
  while (size) {
    const ssize_t n = ::write(fd_, data, size);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) throw Error("serial write failed: " + std::string(std::strerror(errno)));
    data += n;
    size -= static_cast<size_t>(n);
  }
  if (tcdrain(fd_) != 0) throw Error("tcdrain failed: " + std::string(std::strerror(errno)));
}

bool Sensor::readByte(uint8_t& value, int timeout_ms) {
  pollfd pfd{fd_, POLLIN, 0};
  while (true) {
    const int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc == 0) return false;
    if (rc < 0 && errno == EINTR) continue;
    if (rc < 0) throw Error("poll failed: " + std::string(std::strerror(errno)));
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) throw Error("serial port disconnected");
    const ssize_t n = ::read(fd_, &value, 1);
    if (n == 1) return true;
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) throw Error("serial read failed: " + std::string(std::strerror(errno)));
  }
}

bool Sensor::readFrame(Frame& frame, int timeout_ms) {
  ensureOpen();
  const int64_t deadline = deadlineMs(timeout_ms < 0 ? config_.timeout_ms : timeout_ms);
  uint8_t b = 0;
  while (remainingMs(deadline) > 0) {
    if (!readByte(b, remainingMs(deadline))) return false;
    if (b != kHead0) continue;
    if (!readByte(b, remainingMs(deadline))) return false;
    if (b != kHead1) continue;
    uint8_t length = 0;
    if (!readByte(length, remainingMs(deadline))) return false;
    if (length < 3) continue;
    std::vector<uint8_t> body(length + 4);  // body, CRC (2), tail (2)
    for (auto& byte : body) if (!readByte(byte, remainingMs(deadline))) return false;
    if (body[length + 2] != kTail0 || body[length + 3] != kTail1) continue;
    const uint16_t expected = static_cast<uint16_t>(body[length]) |
                              (static_cast<uint16_t>(body[length + 1]) << 8);
    if (crc16Ccitt(body.data(), length) != expected) continue;
    frame.address = body[0];
    frame.reserved = body[1];
    frame.command = body[2];
    frame.payload.assign(body.begin() + 3, body.begin() + length);
    return true;
  }
  return false;
}

std::vector<uint8_t> Sensor::encodeFrame(uint8_t address, uint8_t command,
                                         const std::vector<uint8_t>& payload) {
  const size_t length = 3 + payload.size();
  if (length > 255) throw Error("payload too large");
  std::vector<uint8_t> frame;
  frame.reserve(length + 7);
  frame.push_back(kHead0);
  frame.push_back(kHead1);
  frame.push_back(static_cast<uint8_t>(length));
  frame.push_back(address);
  frame.push_back(0);
  frame.push_back(command);
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16Ccitt(frame.data() + 3, length);
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(kTail0);
  frame.push_back(kTail1);
  return frame;
}

uint16_t Sensor::crc16Ccitt(const uint8_t* data, size_t size) {
  uint16_t crc = 0xFFFF;
  while (size--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (int i = 0; i < 8; ++i) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                                       : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

Sensor::Frame Sensor::transact(uint8_t command, const std::vector<uint8_t>& payload,
                               int timeout_ms, int expected_address) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureOpen();
  const auto bytes = encodeFrame(config_.address, command, payload);
  writeAll(bytes.data(), bytes.size());
  const int timeout = timeout_ms < 0 ? config_.timeout_ms : timeout_ms;
  const int64_t deadline = deadlineMs(timeout);
  Frame response;
  while (remainingMs(deadline) > 0) {
    if (!readFrame(response, remainingMs(deadline))) break;
    const bool address_matches = expected_address == -2 ||
                                 response.address == static_cast<uint8_t>(expected_address < 0 ? config_.address : expected_address);
    if (address_matches && response.command == command) return response;
  }
  throw Error("timeout waiting for response to command 0x" + [&] { char b[3]; std::snprintf(b, sizeof(b), "%02X", command); return std::string(b); }());
}

bool Sensor::transactAck(uint8_t command, const std::vector<uint8_t>& payload) {
  const Frame response = transact(command, payload);
  return !response.payload.empty() && response.payload[0] == 0x01;
}

Wrench Sensor::parseWrench(const Frame& frame) {
  if (frame.payload.size() < 24) throw Error("invalid measurement payload");
  Wrench w;
  w.status = frame.reserved;
  auto value = [&](size_t i) { return static_cast<int32_t>(static_cast<uint32_t>(frame.payload[i]) |
      (static_cast<uint32_t>(frame.payload[i + 1]) << 8) |
      (static_cast<uint32_t>(frame.payload[i + 2]) << 16) |
      (static_cast<uint32_t>(frame.payload[i + 3]) << 24)); };
  w.fx = value(0) / 1000.0; w.fy = value(4) / 1000.0; w.fz = value(8) / 1000.0;
  w.mx = value(12) / 1000.0; w.my = value(16) / 1000.0; w.mz = value(20) / 1000.0;
  timespec timestamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0)
    throw Error("clock_gettime failed: " + std::string(std::strerror(errno)));
  w.monotonic_ns = static_cast<uint64_t>(timestamp.tv_sec) * 1000000000ULL +
                   static_cast<uint64_t>(timestamp.tv_nsec);
  return w;
}

uint16_t Sensor::getDeviceId() {
  const Frame f = transact(0x01);
  if (f.payload.size() < 2) throw Error("invalid device ID response");
  return static_cast<uint16_t>(f.payload[0] | (f.payload[1] << 8));
}

Wrench Sensor::measureOnce() {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureOpen();
  const auto bytes = encodeFrame(config_.address, 0x04, {});
  writeAll(bytes.data(), bytes.size());
  Frame f;
  const int64_t deadline = deadlineMs(config_.timeout_ms);
  while (remainingMs(deadline) > 0 && readFrame(f, remainingMs(deadline)))
    if (f.address == config_.address && f.command == 0x04 && f.payload.size() >= 24) return parseWrench(f);
  throw Error("timeout waiting for single measurement");
}

void Sensor::startContinuous() {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureOpen();
  const auto bytes = encodeFrame(config_.address, 0x02, {});
  writeAll(bytes.data(), bytes.size());
}

void Sensor::stopContinuous() {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureOpen();
  std::array<uint8_t, 50> zeros{};
  writeAll(zeros.data(), zeros.size());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // Discard measurement frames accumulated during the required quiet period.
  // Otherwise a high-rate stream can delay the stop acknowledgement past the
  // normal command timeout.
  if (tcflush(fd_, TCIFLUSH) != 0)
    throw Error("tcflush failed: " + std::string(std::strerror(errno)));
  const auto bytes = encodeFrame(config_.address, 0x03, {});
  writeAll(bytes.data(), bytes.size());
  Frame f;
  const int64_t deadline = deadlineMs(std::max(config_.timeout_ms, 1000));
  while (remainingMs(deadline) > 0 && readFrame(f, remainingMs(deadline)))
    if (f.address == config_.address && f.command == 0x03) {
      if (f.payload.empty() || f.payload[0] != 0x01) throw Error("sensor failed to stop continuous mode");
      return;
    }
  throw Error("timeout waiting for stop response");
}

bool Sensor::readMeasurement(Wrench& out, int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  Frame f;
  const int timeout = timeout_ms < 0 ? config_.timeout_ms : timeout_ms;
  const int64_t deadline = deadlineMs(timeout);
  while (remainingMs(deadline) > 0 && readFrame(f, remainingMs(deadline))) {
    if (f.address == config_.address && (f.command == 0x02 || f.command == 0x04) && f.payload.size() >= 24) {
      out = parseWrench(f);
      return true;
    }
  }
  return false;
}

bool Sensor::setMeasurementFrequency(uint8_t level) { if (level > 4) throw Error("measurement frequency must be 0..4"); return transactAck(0x05, {level}); }
bool Sensor::setBaudrate(uint32_t baudrate) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureOpen();
  (void)baudConstant(baudrate);  // Validate before changing the live port.
  const std::vector<uint8_t> payload = {
      static_cast<uint8_t>(baudrate), static_cast<uint8_t>(baudrate >> 8),
      static_cast<uint8_t>(baudrate >> 16), static_cast<uint8_t>(baudrate >> 24)};
  const auto bytes = encodeFrame(config_.address, 0x06, payload);
  writeAll(bytes.data(), bytes.size());
  // The manual states that the new baud rate takes effect immediately after
  // accepting the command, so switch before reading the response.
  configurePort(baudrate);
  config_.baudrate = baudrate;
  Frame f;
  const int64_t deadline = deadlineMs(config_.timeout_ms);
  while (remainingMs(deadline) > 0 && readFrame(f, remainingMs(deadline))) {
    if (f.address == config_.address && f.command == 0x06)
      return f.payload.size() >= 4;
  }
  throw Error("timeout waiting for baud rate response");
}
bool Sensor::restoreUserSettings() {
  const Frame f = transact(0x07, {}, -1, -2);
  return !f.payload.empty() && f.payload[0] == 0x01;
}
bool Sensor::restoreFactorySettings() {
  const Frame f = transact(0x08, {}, -1, -2);
  return !f.payload.empty() && f.payload[0] == 0x01;
}
bool Sensor::saveUserSettings() { return transactAck(0x09); }
VersionInfo Sensor::getVersion() {
  const Frame f = transact(0x0A); if (f.payload.size() < 6) throw Error("invalid version response");
  return {f.payload[0], f.payload[1], f.payload[2], f.payload[3], f.payload[4], f.payload[5]};
}
bool Sensor::zeroCalibration() { return transactAck(0x0B); }
bool Sensor::resetAddress() {
  const Frame f = transact(0x0D, {}, -1, 0);
  const bool ok = !f.payload.empty() && f.payload[0] == 0x01;
  if (ok) config_.address = 0;
  return ok;
}
bool Sensor::setAddress(uint8_t address) {
  const Frame f = transact(0x0E, {address}, -1, address);
  const bool ok = !f.payload.empty() && f.payload[0] == 0x01;
  if (ok) config_.address = address;
  return ok;
}
bool Sensor::setLowPassFilter(uint8_t level) { if (level > 4) throw Error("low-pass filter level must be 0..4"); return transactAck(0x18, {level}); }

std::array<uint32_t, 6> Sensor::getOverloadCounts() {
  const Frame f = transact(0xD4); if (f.payload.size() < 25) throw Error("invalid overload count response");
  std::array<uint32_t, 6> result{}; for (size_t i = 0; i < 6; ++i) result[i] = readU32(f.payload, 1 + i * 4); return result;
}
std::array<int32_t, 6> Sensor::getOverloadPeaks() {
  const Frame f = transact(0xD8); if (f.payload.size() < 25) throw Error("invalid overload peak response");
  std::array<int32_t, 6> result{}; for (size_t i = 0; i < 6; ++i) result[i] = static_cast<int32_t>(readU32(f.payload, 1 + i * 4)); return result;
}

DualSensor::DualSensor(SerialConfig first, SerialConfig second)
    : first_(std::move(first)), second_(std::move(second)) {}

void DualSensor::open() {
  try {
    runParallel([&] { first_.open(); }, [&] { second_.open(); });
  } catch (...) {
    first_.close();
    second_.close();
    throw;
  }
}

void DualSensor::close() noexcept {
  first_.close();
  second_.close();
}

bool DualSensor::isOpen() const noexcept {
  return first_.isOpen() && second_.isOpen();
}

std::array<uint16_t, 2> DualSensor::getDeviceIds() {
  std::array<uint16_t, 2> ids{};
  runParallel([&] { ids[0] = first_.getDeviceId(); },
              [&] { ids[1] = second_.getDeviceId(); });
  return ids;
}

std::array<Wrench, 2> DualSensor::measureOnce() {
  std::array<Wrench, 2> values{};
  runParallel([&] { values[0] = first_.measureOnce(); },
              [&] { values[1] = second_.measureOnce(); });
  return values;
}

void DualSensor::startContinuous() {
  runParallel([&] { first_.startContinuous(); },
              [&] { second_.startContinuous(); });
}

void DualSensor::stopContinuous() {
  runParallel([&] { first_.stopContinuous(); },
              [&] { second_.stopContinuous(); });
}

bool DualSensor::readMeasurement(DualMeasurement& out, int timeout_ms) {
  std::array<bool, 2> received{};
  runParallel([&] { received[0] = first_.readMeasurement(out.first, timeout_ms); },
              [&] { received[1] = second_.readMeasurement(out.second, timeout_ms); });
  return received[0] && received[1];
}

}  // namespace hps6axis
