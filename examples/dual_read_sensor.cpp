#include "hps6axis/hps6axis.hpp"

#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void handleSignal(int) { stop_requested = 1; }

void printWrench(int index, const hps6axis::Wrench& w) {
  std::cout << "sensor[" << index << "] F[N] " << std::fixed << std::setprecision(3)
            << w.fx << ' ' << w.fy << ' ' << w.fz
            << "  M[Nm] " << w.mx << ' ' << w.my << ' ' << w.mz
            << "  status=0x" << std::hex << static_cast<int>(w.status) << std::dec << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  const std::string first_device = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  const std::string second_device = argc > 2 ? argv[2] : "/dev/ttyUSB1";
  bool continuous = false;
  std::unique_ptr<hps6axis::DualSensor> sensors;
  try {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    sensors = std::make_unique<hps6axis::DualSensor>(
        hps6axis::SerialConfig{first_device, 115200, 500, 0},
        hps6axis::SerialConfig{second_device, 115200, 500, 0});
    sensors->open();
    const auto ids = sensors->getDeviceIds();
    std::cout << "sensor[0] device id: 0x" << std::hex << ids[0]
              << ", sensor[1] device id: 0x" << ids[1] << std::dec << '\n';
    sensors->startContinuous();
    continuous = true;
    hps6axis::DualMeasurement measurement;
    while (!stop_requested && sensors->readMeasurement(measurement, 1000)) {
      printWrench(0, measurement.first);
      printWrench(1, measurement.second);
    }
    if (continuous) sensors->stopContinuous();
  } catch (const std::exception& e) {
    if (continuous && sensors) {
      try { sensors->stopContinuous(); } catch (...) { /* preserve original error */ }
    }
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
