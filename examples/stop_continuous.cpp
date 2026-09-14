#include "hps6axis/hps6axis.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
  try {
    hps6axis::SerialConfig cfg;
    if (argc > 1) cfg.device = argv[1];
    hps6axis::Sensor sensor(cfg);
    sensor.open();
    sensor.stopContinuous();
    std::cout << "continuous measurement stopped\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
