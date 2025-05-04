#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <string>

#include <libserial/SerialPort.h>

class LocalNucleoInterface {
public:
  static constexpr size_t SERVO_COUNT = 6;
  static constexpr size_t WHEEL_COUNT = 4;

  static constexpr double ARR = 20000;

  LocalNucleoInterface(int timeout_ms = 10);
  ~LocalNucleoInterface();

  void set_servo_ccrs(const std::array<uint16_t, SERVO_COUNT> ccrs);
  void set_servo_angles(const std::array<double, SERVO_COUNT> angles);
  void set_wheel_speeds(const std::array<int32_t, WHEEL_COUNT> &speeds);

  void stop_all_steppers();
  void close();

  void send_command(char, std::span<const uint8_t>);
  void receive_data(std::span<uint8_t>);

private:
  void connect();
  std::string find_nucleo_port();

  LibSerial::SerialPort serial_port_;
  int timeout_ms_;

  static constexpr double MAX_SERVO_DUTY_CYCLE = 20.0;
  static constexpr double MIN_SERVO_DUTY_CYCLE = 2.0;
};
