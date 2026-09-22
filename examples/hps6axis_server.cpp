#include "hps6axis/hps6axis.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
constexpr size_t kFrameQueueCapacity = 512;

void handleSignal(int) { stop_requested = 1; }

struct PublishedFrame {
  uint64_t generation = 0;
  std::string line;
};

class FrameHub {
 public:
  void publish(std::string line) {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back({++generation_, std::move(line)});
    if (frames_.size() > kFrameQueueCapacity) frames_.pop_front();
    condition_.notify_one();
  }

  uint64_t latestGeneration() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  bool nextAfter(uint64_t& cursor, PublishedFrame& result) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.empty() || cursor >= frames_.back().generation) return false;
    if (cursor + 1 < frames_.front().generation) {
      // The client fell behind the bounded queue: skip stale samples and send
      // the newest complete JSON line.
      result = frames_.back();
      return true;
    }
    const size_t index = static_cast<size_t>(cursor + 1 - frames_.front().generation);
    result = frames_[index];
    return true;
  }

  void waitForFrame(uint64_t generation, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, timeout, [&] {
      return generation_ != generation || stop_requested;
    });
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<PublishedFrame> frames_;
  uint64_t generation_ = 0;
};

struct Client {
  int fd = -1;
  uint64_t cursor = 0;
  PublishedFrame pending;
  size_t offset = 0;
};

bool setNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

std::string jsonLine(const hps6axis::Wrench& w, uint16_t device_id,
                     uint64_t sequence) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\"schema\":\"hps6axis.wrench.v1\""
      << ",\"monotonic_ns\":" << w.monotonic_ns
      << ",\"sequence\":" << sequence
      << ",\"device_id\":" << device_id
      << ",\"status\":" << static_cast<unsigned>(w.status)
      << ",\"fx\":" << w.fx << ",\"fy\":" << w.fy << ",\"fz\":" << w.fz
      << ",\"mx\":" << w.mx << ",\"my\":" << w.my << ",\"mz\":" << w.mz
      << "}\n";
  return out.str();
}

void sleepUntilRetry() {
  for (int i = 0; i < 10 && !stop_requested; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void acquire(const std::string& device, FrameHub& hub) {
  uint64_t sequence = 0;
  while (!stop_requested) {
    try {
      hps6axis::Sensor sensor({device, 115200, 500, 0});
      sensor.open();

      uint16_t device_id = 0;
      try {
        device_id = sensor.getDeviceId();
      } catch (const std::exception&) {
        // Recover a sensor left streaming by a killed previous process.
        sensor.stopContinuous();
        device_id = sensor.getDeviceId();
      }

      sensor.startContinuous();
      std::cerr << "sensor 0x" << std::hex << device_id << std::dec
                << " connected on " << device << '\n';

      int consecutive_timeouts = 0;
      while (!stop_requested) {
        hps6axis::Wrench wrench;
        if (!sensor.readMeasurement(wrench, 1000)) {
          if (++consecutive_timeouts >= 3)
            throw hps6axis::Error("three consecutive measurement timeouts");
          continue;
        }
        consecutive_timeouts = 0;
        hub.publish(jsonLine(wrench, device_id, sequence++));
      }
      sensor.stopContinuous();
    } catch (const std::exception& e) {
      if (!stop_requested)
        std::cerr << "sensor unavailable on " << device << ": " << e.what()
                  << "; retrying\n";
      sleepUntilRetry();
    }
  }
}

int createServer(const std::string& bind_address, int port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) throw hps6axis::Error("socket failed: " + std::string(std::strerror(errno)));
  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1) {
    close(fd);
    throw hps6axis::Error("invalid bind address: " + bind_address);
  }
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    const std::string message = std::strerror(errno);
    close(fd);
    throw hps6axis::Error("bind " + bind_address + ':' + std::to_string(port) +
                         " failed: " + message);
  }
  if (listen(fd, 8) < 0 || !setNonBlocking(fd)) {
    close(fd);
    throw hps6axis::Error("failed to listen on TCP socket");
  }
  return fd;
}

void acceptClients(int server_fd, FrameHub& hub, std::vector<Client>& clients) {
  while (true) {
    const int fd = accept(server_fd, nullptr, nullptr);
    if (fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
      throw hps6axis::Error("accept failed: " + std::string(std::strerror(errno)));
    }
    if (!setNonBlocking(fd)) {
      close(fd);
      continue;
    }
    int buffer_size = 8192;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
    clients.push_back(Client{fd, hub.latestGeneration(), {}, 0});
  }
}

void closeClients(std::vector<Client>& clients) {
  for (const Client& client : clients) close(client.fd);
  clients.clear();
}

void serveClients(int server_fd, FrameHub& hub) {
  std::vector<Client> clients;
  try {
    while (!stop_requested) {
      acceptClients(server_fd, hub, clients);
      bool made_progress = false;
      for (auto it = clients.begin(); it != clients.end();) {
        Client& client = *it;
        if (client.pending.line.empty()) hub.nextAfter(client.cursor, client.pending);
        if (client.pending.line.empty()) {
          ++it;
          continue;
        }
        const ssize_t count = send(client.fd,
            client.pending.line.data() + client.offset,
            client.pending.line.size() - client.offset,
            MSG_DONTWAIT | MSG_NOSIGNAL);
        if (count > 0) {
          client.offset += static_cast<size_t>(count);
          made_progress = true;
          if (client.offset == client.pending.line.size()) {
            client.cursor = client.pending.generation;
            client.pending = {};
            client.offset = 0;
          }
          ++it;
        } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
          ++it;
        } else {
          close(client.fd);
          it = clients.erase(it);
        }
      }
      if (!made_progress)
        hub.waitForFrame(hub.latestGeneration(), std::chrono::milliseconds(10));
    }
  } catch (...) {
    closeClients(clients);
    throw;
  }
  closeClients(clients);
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
  FrameHub hub;
  std::thread acquisition_thread;
  try {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    server_fd = createServer(bind_address, port);
    acquisition_thread = std::thread(acquire, std::cref(device), std::ref(hub));
    std::cout << "publishing " << device << " on " << bind_address << ':' << port << '\n';
    serveClients(server_fd, hub);
    if (acquisition_thread.joinable()) acquisition_thread.join();
    close(server_fd);
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    stop_requested = 1;
    if (acquisition_thread.joinable()) acquisition_thread.join();
    if (server_fd >= 0) close(server_fd);
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
