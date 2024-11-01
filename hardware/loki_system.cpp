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
// - Updated the hardware interface to communicate with the Loki robot hardware via serial connection.
// - Removed unnecessary methods and adjusted the implementation to fit the Loki robot's architecture.
// - Implemented custom methods for reading positions and velocities directly in radians and radians per second.
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "loki_hw_interface/loki_system.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace loki_hw_interface {

hardware_interface::CallbackReturn LokiHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters
  cfg_.wheel1_name = info_.hardware_parameters["wheel1_name"];
  cfg_.wheel2_name = info_.hardware_parameters["wheel2_name"];
  cfg_.wheel3_name = info_.hardware_parameters["wheel3_name"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // Initialize positions, velocities, and commands
  wheel_positions_ = std::make_tuple(0.0, 0.0, 0.0);
  wheel_velocities_ = std::make_tuple(0.0, 0.0, 0.0);
  wheel_commands_ = std::make_tuple(0.0, 0.0, 0.0);

  // Check joints
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"),
                   "Joint '%s' has '%s' command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"),
                   "Joint '%s' has %zu state interfaces. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"),
                   "Joint '%s' has '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"),
                   "Joint '%s' has '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LokiHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.wheel1_name, hardware_interface::HW_IF_POSITION, &std::get<0>(wheel_positions_)));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.wheel1_name, hardware_interface::HW_IF_VELOCITY, &std::get<0>(wheel_velocities_)));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.wheel2_name, hardware_interface::HW_IF_POSITION, &std::get<1>(wheel_positions_)));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.wheel2_name, hardware_interface::HW_IF_VELOCITY, &std::get<1>(wheel_velocities_)));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.wheel3_name, hardware_interface::HW_IF_POSITION, &std::get<2>(wheel_positions_)));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.wheel3_name, hardware_interface::HW_IF_VELOCITY, &std::get<2>(wheel_velocities_)));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LokiHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      cfg_.wheel1_name, hardware_interface::HW_IF_VELOCITY, &std::get<0>(wheel_commands_)));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      cfg_.wheel2_name, hardware_interface::HW_IF_VELOCITY, &std::get<1>(wheel_commands_)));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      cfg_.wheel3_name, hardware_interface::HW_IF_VELOCITY, &std::get<2>(wheel_commands_)));

  return command_interfaces;
}

hardware_interface::CallbackReturn LokiHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Configuring ...please wait...");

  try {
    comms_ = std::make_unique<LocalNucleoInterface>(cfg_.baud_rate, cfg_.timeout_ms);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"), "Failed to connect to hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LokiHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Cleaning up ...please wait...");

  if (comms_) {
    comms_->close();
    comms_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LokiHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Activating ...please wait...");

  if (!comms_) {
    RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"), "Hardware interface not configured properly.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LokiHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Deactivating ...please wait...");

  if (comms_) {
    comms_->stop_all_steppers();
  }

  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LokiHardware::on_error(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_ERROR(rclcpp::get_logger("LokiHardware"), "An error occurred, cleaning up resources.");

  // Clean up resources
  if (comms_) {
    comms_->close();
    comms_.reset();
  }

  // Reset commands and states
  wheel_positions_ = std::make_tuple(0.0, 0.0, 0.0);
  wheel_velocities_ = std::make_tuple(0.0, 0.0, 0.0);
  wheel_commands_ = std::make_tuple(0.0, 0.0, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("LokiHardware"), "Cleaned up after error. Ready for reconfiguration.");

  // Return SUCCESS to allow for reconfiguration
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LokiHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!comms_) {
    RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"), "Hardware interface not connected.");
    return hardware_interface::return_type::ERROR;
  }

  try {
    auto data = comms_->read_position_and_velocity();
    double position1, position2, position3;
    double velocity1, velocity2, velocity3;

    std::tie(position1, position2, position3, velocity1, velocity2, velocity3) = data;

    wheel_positions_ = std::make_tuple(position1, position2, position3);
    wheel_velocities_ = std::make_tuple(velocity1, velocity2, velocity3);

  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"), "Failed to read from hardware: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LokiHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!comms_) {
    RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"), "Hardware interface not connected.");
    return hardware_interface::return_type::ERROR;
  }

  try {
    double cmd1_rad_per_sec = std::get<0>(wheel_commands_);
    double cmd2_rad_per_sec = std::get<1>(wheel_commands_);
    double cmd3_rad_per_sec = std::get<2>(wheel_commands_);

    comms_->set_wheel_speeds(std::make_tuple(cmd1_rad_per_sec, cmd2_rad_per_sec, cmd3_rad_per_sec));
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("LokiHardware"), "Failed to write to hardware: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace loki_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(loki_hw_interface::LokiHardware, hardware_interface::SystemInterface)

