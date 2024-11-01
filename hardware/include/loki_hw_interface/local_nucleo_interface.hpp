#pragma once

#include <libserial/SerialPort.h>
#include <string>
#include <tuple>

class LocalNucleoInterface {
public:
    LocalNucleoInterface(int baud_rate = 115200, int timeout_ms = 10);
    ~LocalNucleoInterface();

    void set_servo_angle(int channel, float angle);
    std::tuple<double, double, double, double, double, double> read_position_and_velocity();
    void set_wheel_speeds(const std::tuple<int, int, int>& speeds);
    void stop_all_steppers();
    void close();

private:
    void connect();
    std::string find_nucleo_port();
    void send_command(char command_byte, const std::vector<uint8_t>& data_bytes = {});
    std::vector<uint8_t> receive_data(size_t num_bytes);

    LibSerial::SerialPort serial_port_;
    int baud_rate_;
    int timeout_ms_;

    static constexpr int TIMING_MIN = 100;
    static constexpr int TIMING_MAX = 470;
};
