#include "hps6axis/hps6axis.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

void handleSignal(int) { stop_requested = 1; }

bool setNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

std::string jsonLine(const hps6axis::Wrench& w, uint16_t device_id) {
  const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\"timestamp_ms\":" << now
      << ",\"device_id\":" << device_id
      << ",\"status\":" << static_cast<unsigned>(w.status)
      << ",\"fx\":" << w.fx << ",\"fy\":" << w.fy << ",\"fz\":" << w.fz
      << ",\"mx\":" << w.mx << ",\"my\":" << w.my << ",\"mz\":" << w.mz
      << "}\n";
  return out.str();
}

void closeClients(std::vector<int>& clients) {
  for (const int fd : clients) close(fd);
  clients.clear();
}

void acceptClients(int server_fd, std::vector<int>& clients) {
  while (true) {
    sockaddr_in peer{};
    socklen_t length = sizeof(peer);
    const int fd = accept(server_fd, reinterpret_cast<sockaddr*>(&peer), &length);
    if (fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
      throw hps6axis::Error("accept failed: " + std::string(std::strerror(errno)));
    }
    // Keep stalled clients from blocking sensor acquisition indefinitely.
    const timeval timeout{0, 100000};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    clients.push_back(fd);
  }
}

void publish(const std::string& line, std::vector<int>& clients) {
  for (auto it = clients.begin(); it != clients.end();) {
    const int fd = *it;
    size_t sent = 0;
    bool ok = true;
    while (sent < line.size()) {
      const ssize_t n = send(fd, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
      if (n > 0) {
        sent += static_cast<size_t>(n);
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        ok = false;
        break;
      }
    }
    if (ok) {
      ++it;
    } else {
      close(fd);
      it = clients.erase(it);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string device = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  const int port = argc > 2 ? std::atoi(argv[2]) : 9000;
  const std::string bind_address = argc > 3 ? argv[3] : "127.0.0.1";
  if (port < 1 || port > 65535) {
    std::cerr << "port must be in the range 1..65535\n";
    return EXIT_FAILURE;
  }

  int server_fd = -1;
  std::vector<int> clients;
  bool continuous = false;
  std::unique_ptr<hps6axis::Sensor> sensor;
  try {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) throw hps6axis::Error("socket failed: " + std::string(std::strerror(errno)));
    int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1)
      throw hps6axis::Error("invalid bind address: " + bind_address);
    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
      throw hps6axis::Error("bind " + bind_address + ":" + std::to_string(port) +
                            " failed: " + std::strerror(errno));
    if (listen(server_fd, 8) < 0 || !setNonBlocking(server_fd))
      throw hps6axis::Error("failed to listen on local port");

    sensor = std::make_unique<hps6axis::Sensor>(hps6axis::SerialConfig{device, 115200, 500, 0});
    sensor->open();
    const uint16_t device_id = sensor->getDeviceId();
    sensor->startContinuous();
    continuous = true;
    std::cout << "publishing sensor 0x" << std::hex << device_id << std::dec
              << " on " << bind_address << ':' << port << '\n';

    while (!stop_requested) {
      acceptClients(server_fd, clients);
      hps6axis::Wrench wrench;
      if (sensor->readMeasurement(wrench, 1000))
        publish(jsonLine(wrench, device_id), clients);
    }
    if (continuous) sensor->stopContinuous();
    closeClients(clients);
    close(server_fd);
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    if (continuous && sensor) {
      try { sensor->stopContinuous(); } catch (...) { /* preserve original error */ }
    }
    closeClients(clients);
    if (server_fd >= 0) close(server_fd);
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
