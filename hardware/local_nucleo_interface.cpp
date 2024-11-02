// local_nucleo_interface.cpp

#include "loki_hw_interface/local_nucleo_interface.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <libserial/SerialPort.h>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace LibSerial;

LocalNucleoInterface::LocalNucleoInterface(int timeout_ms) : timeout_ms_(timeout_ms) { connect(); }

LocalNucleoInterface::~LocalNucleoInterface() { close(); }

void LocalNucleoInterface::connect() {
  std::string port_name = find_nucleo_port();
  if (!port_name.empty()) {
    serial_port_.Open(port_name);
    serial_port_.SetBaudRate(BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(Parity::PARITY_NONE);
    serial_port_.SetStopBits(StopBits::STOP_BITS_1);
    serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
  } else {
    throw std::runtime_error("Could not find Nucleo board. Please ensure it is connected.");
  }
}

// Scans /dev/ttyACM* ports to find the connected Nucleo board
std::string LocalNucleoInterface::find_nucleo_port() {
  // Scan /dev/ttyACM0 to /dev/ttyACM5
  for (int i = 0; i <= 5; ++i) {
    std::string port = "/dev/ttyACM" + std::to_string(i);
    SerialPort test_port;
    try {
      test_port.Open(port);
      test_port.Close();
      return port;
    } catch (...) {
      // Port not available or cannot be opened
      continue;
    }
  }
  return "";
}

void LocalNucleoInterface::close() {
  if (serial_port_.IsOpen()) {
    stop_all_steppers();
    serial_port_.Close();
  }
}

void LocalNucleoInterface::send_command(char command_byte, const std::vector<uint8_t> &data_bytes) {
  if (!serial_port_.IsOpen()) {
    throw std::runtime_error("Serial port is not open!");
  }

  // Prepare the command buffer
  std::vector<uint8_t> buffer;
  buffer.push_back('M');
  buffer.push_back(static_cast<uint8_t>(command_byte));
  buffer.insert(buffer.end(), data_bytes.begin(), data_bytes.end());

  serial_port_.Write(buffer);
}

std::vector<uint8_t> LocalNucleoInterface::receive_data(size_t num_bytes) {
  if (!serial_port_.IsOpen()) {
    throw std::runtime_error("Serial port is not open!");
  }

  std::vector<uint8_t> data;
  data.reserve(num_bytes);

  size_t bytes_read = 0;
  while (bytes_read < num_bytes) {
    uint8_t byte;
    serial_port_.ReadByte(byte, timeout_ms_); // Pass size_t directly
    data.push_back(byte);
    ++bytes_read;
  }
  return data;
}

template <typename T> void append_to_vector_le(std::vector<uint8_t> &vec, T value) {
  uint8_t *ptr = reinterpret_cast<uint8_t *>(&value);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  vec.insert(vec.end(), ptr, ptr + sizeof(T));
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  for (int i = sizeof(T) - 1; i >= 0; --i) {
    vec.push_back(ptr[i]);
  }
#else
#error "Unknown byte order"
#endif
}

template <typename T> T read_from_vector_le(const std::vector<uint8_t> &vec, size_t offset) {
  T value;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(&value, &vec[offset], sizeof(T));
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  uint8_t *ptr = reinterpret_cast<uint8_t *>(&value);
  for (size_t i = 0; i < sizeof(T); ++i) {
    ptr[i] = vec[offset + sizeof(T) - 1 - i];
  }
#else
#error "Unknown byte order"
#endif
  return value;
}

void LocalNucleoInterface::set_servo_angle(int channel, float angle) {
  if (channel != 0 && channel != 1) {
    throw std::invalid_argument("Invalid channel number. Valid channels are 0 and 1.");
  }

  // Map angle to PWM off_time
  float off_time = TIMING_MIN + (angle / 180.0f) * (TIMING_MAX - TIMING_MIN);
  off_time =
      std::max(static_cast<float>(TIMING_MIN), std::min(off_time, static_cast<float>(TIMING_MAX)));

  uint8_t ch = static_cast<uint8_t>(channel);
  uint16_t on_time = 0;
  uint16_t off_time_int = static_cast<uint16_t>(off_time);

  std::vector<uint8_t> data;
  data.push_back(ch);
  append_to_vector_le<uint16_t>(data, on_time);
  append_to_vector_le<uint16_t>(data, off_time_int);

  send_command('s', data);
}

std::tuple<double, double, double, double, double, double>
LocalNucleoInterface::read_position_and_velocity() {
  send_command('a', {}); // Send command 'a' with no additional data

  size_t num_bytes = 6 * sizeof(double); // 6 doubles: 3 positions and 3 velocities
  std::vector<uint8_t> data = receive_data(num_bytes);
  if (data.size() != num_bytes) {
    throw std::runtime_error("Failed to receive position and velocity data.");
  }

  double encoder1 = read_from_vector_le<double>(data, 0);
  double encoder2 = read_from_vector_le<double>(data, sizeof(double));
  double encoder3 = read_from_vector_le<double>(data, 2 * sizeof(double));
  double velocity1 = read_from_vector_le<double>(data, 3 * sizeof(double));
  double velocity2 = read_from_vector_le<double>(data, 4 * sizeof(double));
  double velocity3 = read_from_vector_le<double>(data, 5 * sizeof(double));

  return std::make_tuple(encoder1, encoder2, encoder3, velocity1, velocity2, velocity3);
}

void LocalNucleoInterface::set_wheel_speeds(const std::tuple<int, int, int> &speeds) {
  int speed1 = std::get<0>(speeds);
  int speed2 = std::get<1>(speeds);
  int speed3 = std::get<2>(speeds);

  std::vector<uint8_t> data;
  append_to_vector_le<int32_t>(data, static_cast<int32_t>(speed1));
  append_to_vector_le<int32_t>(data, static_cast<int32_t>(speed2));
  append_to_vector_le<int32_t>(data, static_cast<int32_t>(speed3));

  send_command('u', data);
}

void LocalNucleoInterface::stop_all_steppers() {
  send_command('x', {}); // Send command 'x' with no additional data
}
