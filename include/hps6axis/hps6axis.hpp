#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace hps6axis {

class Error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct SerialConfig {
  std::string device = "/dev/ttyUSB0";
  uint32_t baudrate = 115200;
  int timeout_ms = 500;
  uint8_t address = 0;
};

struct Wrench {
  double fx = 0.0, fy = 0.0, fz = 0.0;
  double mx = 0.0, my = 0.0, mz = 0.0;
  uint8_t status = 0;  // 0: valid, 0xFF: sensor reports an exception
  // CLOCK_MONOTONIC sampled immediately after CRC validation and decoding.
  uint64_t monotonic_ns = 0;
};

struct VersionInfo {
  uint8_t year = 0, month = 0, day = 0;
  uint8_t major = 0, minor = 0, revision = 0;
};

struct DualMeasurement {
  Wrench first;
  Wrench second;
};

class Sensor {
 public:
  explicit Sensor(SerialConfig config = {});
  ~Sensor();
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;

  void open();
  void close() noexcept;
  bool isOpen() const noexcept;

  uint16_t getDeviceId();
  Wrench measureOnce();
  void startContinuous();
  void stopContinuous();
  // Reads the next measurement frame. Returns false only when timeout expires.
  bool readMeasurement(Wrench& out, int timeout_ms = -1);

  bool setMeasurementFrequency(uint8_t level);  // Deprecated by the sensor protocol.
  bool setBaudrate(uint32_t baudrate);
  bool restoreUserSettings();
  bool restoreFactorySettings();
  bool saveUserSettings();
  VersionInfo getVersion();
  bool zeroCalibration();
  bool resetAddress();
  bool setAddress(uint8_t address);
  bool setLowPassFilter(uint8_t level);
  std::array<uint32_t, 6> getOverloadCounts();
  // Peak values are returned as the signed raw int32 values reported by the sensor.
  std::array<int32_t, 6> getOverloadPeaks();

  uint8_t address() const noexcept { return config_.address; }
  const SerialConfig& config() const noexcept { return config_; }

 private:
  struct Frame {
    uint8_t address = 0;
    uint8_t reserved = 0;
    uint8_t command = 0;
    std::vector<uint8_t> payload;
  };

  SerialConfig config_;
  int fd_ = -1;
  mutable std::mutex mutex_;

  void ensureOpen() const;
  void configurePort(uint32_t baudrate);
  void writeAll(const uint8_t* data, size_t size);
  bool readByte(uint8_t& value, int timeout_ms);
  bool readFrame(Frame& frame, int timeout_ms);
  Frame transact(uint8_t command, const std::vector<uint8_t>& payload = {},
                 int timeout_ms = -1, int expected_address = -1);
  bool transactAck(uint8_t command, const std::vector<uint8_t>& payload = {});
  static std::vector<uint8_t> encodeFrame(uint8_t address, uint8_t command,
                                           const std::vector<uint8_t>& payload);
  static uint16_t crc16Ccitt(const uint8_t* data, size_t size);
  static Wrench parseWrench(const Frame& frame);
};

// Two sensors connected to independent serial ports. Reads and lifecycle
// commands are dispatched concurrently because each port has its own bus.
class DualSensor {
 public:
  explicit DualSensor(
      SerialConfig first = {},
      SerialConfig second = SerialConfig{"/dev/ttyUSB1", 115200, 500, 0});
  ~DualSensor() = default;
  DualSensor(const DualSensor&) = delete;
  DualSensor& operator=(const DualSensor&) = delete;

  void open();
  void close() noexcept;
  bool isOpen() const noexcept;
  std::array<uint16_t, 2> getDeviceIds();
  std::array<Wrench, 2> measureOnce();
  void startContinuous();
  void stopContinuous();
  bool readMeasurement(DualMeasurement& out, int timeout_ms = -1);

  Sensor& first() noexcept { return first_; }
  Sensor& second() noexcept { return second_; }
  const Sensor& first() const noexcept { return first_; }
  const Sensor& second() const noexcept { return second_; }

 private:
  Sensor first_;
  Sensor second_;
};

}  // namespace hps6axis
