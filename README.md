# HPS 6-axis RS-485 SDK

Ubuntu/Linux C++17 SDK for the Hypersen six-axis force/torque sensor described by `RS485_ZH.pdf`.

## Build

```bash
sudo apt install build-essential cmake
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Run

The USB-to-RS485 adapter normally appears as `/dev/ttyUSB0` or `/dev/ttyACM0`.
The protocol uses 115200 baud, 8 data bits, even parity, one stop bit (8E1).

```bash
./build/hps6axis_read /dev/ttyUSB0
```

For a first hardware check, use the one-shot utility. It does not start continuous output:

```bash
./build/hps6axis_once /dev/ttyUSB0
```

If a previous process was terminated while the sensor was streaming, recover it with:

```bash
./build/hps6axis_stop /dev/ttyUSB0
```

The continuous example handles `Ctrl-C` and sends the protocol-required stop sequence before exiting.

## Local TCP publisher

`hps6axis_server` publishes each continuous measurement as one JSON object per line on
TCP. The optional arguments are the serial device, TCP port, and bind address:

```bash
./build/hps6axis_server /dev/ttyUSB0 9000
```

A local client can consume the stream with:

```bash
nc 127.0.0.1 9000
```

Example message:

```json
{"schema":"hps6axis.wrench.v1","monotonic_ns":123456789,"sequence":42,"device_id":18174,"status":0,"fx":0.160000,"fy":-0.190000,"fz":-0.890000,"mx":0.005000,"my":0.000000,"mz":0.004000}
```

`monotonic_ns` is sampled with `CLOCK_MONOTONIC` immediately after a complete frame
passes CRC validation and its six values are decoded. `sequence` starts at zero for
each server process and increments only for successfully decoded frames. CRC failures,
incomplete frames, and timeouts produce no JSON record. The hardware status byte is
published unchanged (`0` means normal and any nonzero value indicates a sensor error).
Forces are native sensor coordinates in N and moments are native sensor coordinates in
Nm; this process performs no bias, transform, filtering, contact estimation, or tokenization.

Serial acquisition and TCP delivery run independently. Each client has a bounded path
through a 512-frame queue; a slow client skips stale records rather than blocking serial
reads. If USB is unplugged, the TCP listener and connected clients remain alive while the
server retries the serial port once per second and resumes without a deploy-side restart.

By default the server binds only to loopback. To allow a second machine on the LAN to
connect, bind to the robot's LAN address (preferred) or `0.0.0.0`:

```bash
./build/hps6axis_server /dev/ttyUSB0 9000 192.168.1.20
# or listen on all IPv4 interfaces:
./build/hps6axis_server /dev/ttyUSB0 9000 0.0.0.0
```

Then connect from the other machine using the robot's address:

```bash
nc 192.168.1.20 9000
```

If the robot firewall is enabled, allow only the client machine rather than opening the
port to the whole LAN:

```bash
sudo ufw allow from 192.168.1.30 to any port 9000 proto tcp
```

The protocol is unauthenticated and unencrypted; use it on a trusted network or tunnel
it through SSH/VPN. A remote machine cannot start a process on the robot merely by
connecting to this port. To start the service remotely, use SSH, for example:

```bash
ssh irmv@192.168.1.20 \
  'nohup /home/irmv/Workspace/Somaforce/HPS_6axis_SDK/build/hps6axis_server /dev/ttyUSB0 9000 0.0.0.0 >/tmp/hps6axis-server.log 2>&1 &'
```

To embed the SDK directly in another C++17 program:

```cpp
#include "hps6axis/hps6axis.hpp"

int main() {
  hps6axis::Sensor sensor({"/dev/ttyUSB0", 115200, 500, 0});
  sensor.open();
  sensor.startContinuous();
  hps6axis::Wrench value;
  while (sensor.readMeasurement(value, 1000)) {
    // value.fx, value.fy, value.fz are in N;
    // value.mx, value.my, value.mz are in Nm.
  }
  sensor.stopContinuous();
}
```

Link the application with the built `hps6axis` library and `pthread`:

```bash
c++ -std=c++17 app.cpp -Iinclude -Lbuild -lhps6axis -lpthread -o app
```

## Two sensors on `/dev/ttyUSB0` and `/dev/ttyUSB1`

Use `DualSensor` when each sensor has its own USB-RS485 adapter. Reads and lifecycle
commands are dispatched concurrently on separate worker threads:

```bash
./build/hps6axis_dual_read /dev/ttyUSB0 /dev/ttyUSB1
```

The example prints each sensor's device ID and paired measurements. `DualSensor::readMeasurement`
returns only after both ports have produced a frame (or the timeout expires). The two sensors
may both use address `0x00` because they are connected to independent serial ports.

For deployment, run one independent server process per side:

```bash
./build/hps6axis_server /dev/ttyUSB1 9000 0.0.0.0  # left
./build/hps6axis_server /dev/ttyUSB0 9001 0.0.0.0  # right
```

Or launch both and stop both together with:

```bash
./scripts/start_dual_servers.sh <left-ttyUSB-number> <right-ttyUSB-number> [bind-address]

# Current wiring: left=/dev/ttyUSB1, right=/dev/ttyUSB0
./scripts/start_dual_servers.sh 1 0 0.0.0.0
```

The publisher intentionally does not use `DualSensor::readMeasurement`; a failure or
reconnect on one side cannot delay acquisition or publication on the other side.

The script deliberately requires the left and right ttyUSB numbers on every start. It
checks that both device nodes exist and rejects using the same number for both hands.

The process needs read/write permission for the serial device. On Ubuntu, add the user to `dialout` and log in again:

```bash
sudo usermod -aG dialout "$USER"
```

## Library API

Include `hps6axis/hps6axis.hpp`, construct `hps6axis::Sensor` with a `SerialConfig`, and call `open()`.
`measureOnce()` returns one `Wrench`; for streaming call `startContinuous()` and repeatedly call `readMeasurement()`.
Configuration, version, zero calibration, address, baud rate, low-pass filter, and overload information commands are also exposed.

`getOverloadCounts()` returns six unsigned counters. `getOverloadPeaks()` returns the six signed raw peak values because the manual does not define a physical-unit scale for that command.

Commands that restore saved or factory settings can change the sensor address and baud rate. Recreate or update `SerialConfig` accordingly before issuing subsequent commands.

Before zero calibration, allow the powered sensor to thermally stabilize for 10–20 minutes as recommended by the manual.

## CH341 driver on Jetson

This repository also contains the upstream Linux CH341 driver source in `driver/ch341.c`.
The module has been built successfully for the current Orin kernel (`5.15.199-tegra`, `aarch64`).
If your system does not create `/dev/ttyUSB0`, install and load it as root:

```bash
cd /home/irmv/Workspace/Somaforce/HPS_6axis_SDK
make -C /lib/modules/$(uname -r)/build M=$PWD/driver modules
sudo install -D -m 0644 driver/ch341.ko /lib/modules/$(uname -r)/extra/ch341.ko
sudo depmod -a
sudo modprobe ch341
sudo usermod -aG dialout "$USER"
```

Unplug and reconnect the CH340 adapter, then verify:

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

For a temporary test, `sudo insmod driver/ch341.ko` can be used after `usbserial` is loaded.
The module must be rebuilt whenever the running Jetson kernel version changes.

If the kernel log shows `brltty` claiming the interface and then `ttyUSB0` disappearing,
disable the braille service because it probes CH340 devices as generic USB serial hardware:

```bash
sudo systemctl disable --now brltty.service brltty-udev.service
sudo systemctl mask brltty.service brltty-udev.service
```

Unplug/reconnect the adapter after this change. If BRLTTY is required on the machine,
remove only the CH340 device from its probing rules instead of disabling the services globally.
