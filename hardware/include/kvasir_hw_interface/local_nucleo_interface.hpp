#pragma once

#include <libserial/SerialPort.h>
#include <string>
#include <string_view>
#include <tuple>
#include <span>

class LocalNucleoInterface {
public:
  LocalNucleoInterface(int timeout_ms = 10);
  ~LocalNucleoInterface();

  void set_servo_ccrs(const uint16_t, const uint16_t);
  void set_servo_angles(const double, const double);
  std::tuple<double, double, double, double, double, double> read_position_and_velocity();
  void set_wheel_speeds(const std::tuple<int, int, int> &speeds);
  void print_lcd(const uint8_t line, const std::string_view msg);

  void stop_all_steppers();
  void close();

  void send_command(char command_byte, std::span<const uint8_t> data_bytes);
  void receive_data(std::span<uint8_t> num_bytes);

private:
  void connect();
  std::string find_nucleo_port();

  LibSerial::SerialPort serial_port_;
  int timeout_ms_;

  static constexpr double MAX_SERVO_DUTY_CYCLE = 20.0;
  static constexpr double MIN_SERVO_DUTY_CYCLE = 2.0;
};
