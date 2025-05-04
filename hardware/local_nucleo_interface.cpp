#include <cstring>
#include <stdexcept>

#include <libserial/SerialPort.h>

#include "kvasir_hw_interface/local_nucleo_interface.hpp"

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

void LocalNucleoInterface::send_command(char command_byte, std::span<const uint8_t> data_bytes) {
  if (!serial_port_.IsOpen()) {
    throw std::runtime_error("Serial port is not open!");
  }

  serial_port_.WriteByte('M');
  serial_port_.WriteByte(command_byte);
  for (uint8_t byte : data_bytes)
    serial_port_.WriteByte(byte);
}

void LocalNucleoInterface::receive_data(std::span<uint8_t> buffer) {
  if (!serial_port_.IsOpen()) {
    throw std::runtime_error("Serial port is not open!");
  }

  size_t bytes_read = 0;
  while (bytes_read < buffer.size()) {
    serial_port_.ReadByte(buffer[bytes_read], timeout_ms_);
    ++bytes_read;
  }
}

template <typename T> void write_to_span_le(std::span<uint8_t> buffer, size_t &offset, T value) {
  static_assert(std::is_trivially_copyable_v<T>, "Value must be trivially copyable");
  if (buffer.size() < offset + sizeof(T)) {
    throw std::runtime_error("Buffer overflow in write_to_buffer_le");
  }

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(buffer.data() + offset, &value, sizeof(T));
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  uint8_t *p = reinterpret_cast<uint8_t *>(&value);
  for (size_t i = 0; i < sizeof(T); ++i) {
    buffer[offset + i] = p[sizeof(T) - 1 - i];
  }
#else
#error "Unknown byte order"
#endif

  offset += sizeof(T);
}

template <typename T> T read_from_span_le(std::span<const uint8_t> vec, size_t offset) {
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

void LocalNucleoInterface::set_servo_ccrs(const std::array<uint16_t, SERVO_COUNT> ccrs) {
  std::array<uint8_t, SERVO_COUNT * sizeof(uint16_t)> data;
  size_t data_offset = 0;
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    write_to_span_le<uint16_t>(data, data_offset, ccrs[i]);
  }

  send_command('s', data);
}

void LocalNucleoInterface::set_servo_angles(const std::array<double, SERVO_COUNT> thetas) {
  std::array<uint16_t, SERVO_COUNT> ccrs;
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    ccrs[i] =
        static_cast<uint16_t>(ARR *
                              ((thetas[i] / 180.0) * (MAX_SERVO_DUTY_CYCLE - MIN_SERVO_DUTY_CYCLE) +
                               MIN_SERVO_DUTY_CYCLE) /
                              100.0);
  }
  set_servo_ccrs(ccrs);
}

void LocalNucleoInterface::set_wheel_speeds(const std::array<int32_t, WHEEL_COUNT> &speeds) {
  std::array<uint8_t, WHEEL_COUNT * sizeof(int32_t)> data;
  size_t data_offset = 0;
  for (size_t i = 0; i < WHEEL_COUNT; ++i) {
    write_to_span_le<int32_t>(data, data_offset, speeds[i]);
  }

  send_command('u', data);
}

void LocalNucleoInterface::stop_all_steppers() { send_command('x', {}); }
