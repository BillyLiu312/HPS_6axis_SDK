#include "hps6axis/hps6axis.hpp"

#include <cstdlib>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void handleSignal(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char** argv) {
  bool continuous = false;
  std::unique_ptr<hps6axis::Sensor> sensor;
  try {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    hps6axis::SerialConfig cfg;
    if (argc > 1) cfg.device = argv[1];
    sensor = std::make_unique<hps6axis::Sensor>(cfg);
    sensor->open();
    std::cout << "device id: 0x" << std::hex << sensor->getDeviceId() << std::dec << '\n';
    sensor->startContinuous();
    continuous = true;
    hps6axis::Wrench wrench;
    while (!stop_requested && sensor->readMeasurement(wrench, 1000)) {
      std::cout << std::fixed << std::setprecision(3)
                << "F[N] " << wrench.fx << ' ' << wrench.fy << ' ' << wrench.fz
                << "  M[Nm] " << wrench.mx << ' ' << wrench.my << ' ' << wrench.mz
                << "  status=0x" << std::hex << static_cast<int>(wrench.status) << std::dec << '\n';
    }
    if (continuous) sensor->stopContinuous();
  } catch (const std::exception& e) {
    // Leave the sensor usable after an error occurring during streaming.
    // A signal is handled in the main loop so this cleanup remains synchronous.
    if (continuous && sensor) {
      try { sensor->stopContinuous(); } catch (...) { /* preserve original error */ }
    }
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
