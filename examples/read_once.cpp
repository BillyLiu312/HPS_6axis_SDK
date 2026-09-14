#include "hps6axis/hps6axis.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  try {
    hps6axis::SerialConfig cfg;
    if (argc > 1) cfg.device = argv[1];
    hps6axis::Sensor sensor(cfg);
    sensor.open();
    const auto id = sensor.getDeviceId();
    const auto wrench = sensor.measureOnce();
    std::cout << "device id: 0x" << std::hex << id << std::dec << '\n'
              << std::fixed << std::setprecision(3)
              << "F[N] " << wrench.fx << ' ' << wrench.fy << ' ' << wrench.fz
              << "  M[Nm] " << wrench.mx << ' ' << wrench.my << ' ' << wrench.mz
              << "  status=0x" << std::hex << static_cast<int>(wrench.status) << std::dec << '\n';
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
