#pragma once

#include <libserial/SerialPort.h>
#include <string>
#include <tuple>

class LocalNucleoInterface {
public:
  LocalNucleoInterface(int timeout_ms = 10);
  ~LocalNucleoInterface();

  void set_servo_angle(int channel, float angle);
  std::tuple<double, double, double, double, double, double> read_position_and_velocity();
  void set_wheel_speeds(const std::tuple<int, int, int> &speeds);
  void stop_all_steppers();
  void close();

  void send_command(char command_byte, const std::vector<uint8_t> &data_bytes = {});
  std::vector<uint8_t> receive_data(size_t num_bytes);

private:
  void connect();
  std::string find_nucleo_port();

  LibSerial::SerialPort serial_port_;
  int timeout_ms_;

  // Servo timings to convert angle to PWM off time.
  static constexpr int TIMING_MIN = 100;
  static constexpr int TIMING_MAX = 470;
};
