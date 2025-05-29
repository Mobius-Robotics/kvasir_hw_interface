#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <string>

#include <libserial/SerialPort.h>

class LocalNucleoInterface {
public:
  static constexpr size_t TIM1_SERVOS = 4;
  static constexpr size_t TIM2_SERVOS = 2;
  static constexpr size_t SERVO_COUNT = TIM1_SERVOS + TIM2_SERVOS;
  static constexpr size_t WHEEL_COUNT = 4;

  /// @brief The radius of the wheel in meters.
  static constexpr double WHEEL_RADIUS = 35.9e-3;

  /// @brief The distance between the center of the robot and the wheel on the x axis, in meters.
  static constexpr double WHEEL_L_X = 228.0e-3 / 2;
  /// @brief The distance between the center of the robot and the wheel on the y axis, in meters.
  static constexpr double WHEEL_L_Y = 102.0e-3 / 2;

  // Convenience constants for the inverse kinematics.
  static constexpr double WHEEL_L_SUM = WHEEL_L_X + WHEEL_L_Y;
  static constexpr double WHEEL_INVERSE_RADIUS = 1 / WHEEL_RADIUS;

  /// @brief The max value of the timer counter in the Nucleo board, for the servo motors.
  static constexpr double ARR = 20000;

  /// @brief Signs to apply to wheel speeds to handle interior/exterior motor position.
  static constexpr double WHEEL_SIGNS[WHEEL_COUNT] = {-1, 1, -1, 1};

  struct __attribute__((packed)) Status {
      bool pullstart;
      bool interlock;
      bool endstops[2];
  };

  LocalNucleoInterface(int timeout_ms = 10);
  ~LocalNucleoInterface();

  /// @brief Set the duty cycle of the servo motors via directly setting the CCR registers.
  void set_servo_ccrs(const std::array<uint16_t, SERVO_COUNT> ccrs);

  /// @brief Set the angles of the servo motors, calculating duty cycles.
  void set_servo_angles(const std::array<double, SERVO_COUNT> angles);

  /// @brief Set the wheel speeds of the robot.
  /// @param speeds The wheel speeds in rad/s.
  void set_wheel_speeds(const std::array<int32_t, WHEEL_COUNT> &speeds);

  /// @brief Set the body velocity of the robot.
  void set_body_velocity(const double x_dot, const double y_dot, const double theta_dot);

  /// @brief Move the elevator to a specific position.
  void move_elevator(const int16_t position);

  /// @brief Extend the plank arm.
  void extend_arm();

  /// @brief Retract the plank arm.
  void retract_arm();

  /// @brief Extend the tin pushers, specifying which ones to extend.
  void extend_pusher(bool pushers[TIM1_SERVOS]);

  /// @brief Retract the tin pushers.
  void retract_pusher();

  /// @brief Read the status of the Nucleo board.
  Status read_status();

  /// @brief Stop all the motors.
  void stop_all_steppers();

  /// @brief Close the serial port.
  void close();

  /// @brief Send a command to the Nucleo board.
  /// @param opcode The command opcode.
  /// @param payload The payload to send with the command.
  void send_command(char opcode, std::span<const uint8_t> payload);

  /// @brief Receive data from the Nucleo board.
  /// @param data The buffer to receive the data into.
  void receive_data(std::span<uint8_t> data);

private:
  void connect();
  std::string find_nucleo_port();

  LibSerial::SerialPort serial_port_;
  int timeout_ms_;

  static constexpr double MAX_SERVO_DUTY_CYCLE = 20.0;
  static constexpr double MIN_SERVO_DUTY_CYCLE = 2.0;
};
