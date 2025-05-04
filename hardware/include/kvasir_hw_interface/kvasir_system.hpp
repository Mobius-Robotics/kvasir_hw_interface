// Copyright 2021
// ROS 2 Control Development Team
//
// Modifications copyright 2024
// PurpleMyst
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Modifications:
// - Updated the hardware interface to communicate with the Kvasir robot hardware via serial
// connection.
// - Removed unnecessary methods and adjusted the implementation to fit the Kvasir robot's
// architecture.
// - Implemented custom methods for reading positions and velocities directly in radians and radians
// per second.
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "kvasir_hw_interface/visibility_control.h"
#include "rclcpp/macros.hpp"

#include "local_nucleo_interface.hpp"

namespace kvasir_hw_interface {

class KvasirHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KvasirHardware);

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  KVASIR_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KVASIR_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  KVASIR_HW_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

private:
  struct Config {
    std::string wheel1_name = "";
    std::string wheel2_name = "";
    std::string wheel3_name = "";
    int baud_rate = 115200;
    int timeout_ms = 10;
  } cfg_;

  std::unique_ptr<LocalNucleoInterface> comms_;

  std::array<double, LocalNucleoInterface::WHEEL_COUNT> wheel_positions_;
  std::array<double, LocalNucleoInterface::WHEEL_COUNT> wheel_velocities_;
  std::array<double, LocalNucleoInterface::WHEEL_COUNT> wheel_commands_;
};

} // namespace kvasir_hw_interface
