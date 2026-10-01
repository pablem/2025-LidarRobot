// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffbot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev_left = std::stoi(info_.hardware_parameters["enc_counts_per_rev_left"]);
  cfg_.enc_counts_per_rev_right = std::stoi(info_.hardware_parameters["enc_counts_per_rev_right"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

  if (info_.hardware_parameters.count("battery_voltage_min") > 0)
  {
    cfg_.battery_voltage_min = std::stod(info_.hardware_parameters["battery_voltage_min"]);
  }
  if (info_.hardware_parameters.count("battery_voltage_max") > 0)
  {
    cfg_.battery_voltage_max = std::stod(info_.hardware_parameters["battery_voltage_max"]);
  }
  if (info_.hardware_parameters.count("battery_runtime_full_min") > 0)
  {
    cfg_.battery_runtime_full_min = std::stod(info_.hardware_parameters["battery_runtime_full_min"]);
  }
  if (info_.hardware_parameters.count("battery_publish_period") > 0)
  {
    cfg_.battery_publish_period = std::stod(info_.hardware_parameters["battery_publish_period"]);
  }
  if (info_.hardware_parameters.count("max_wheel_vel") > 0)
  {
    cfg_.max_wheel_vel = std::stod(info_.hardware_parameters["max_wheel_vel"]);
  }
  if (info_.hardware_parameters.count("max_rejected_reads") > 0)
  {
    cfg_.max_rejected_reads = std::stoi(info_.hardware_parameters["max_rejected_reads"]);
  }


  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev_left);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev_right);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  battery_node_ = std::make_shared<rclcpp::Node>("diffdrive_battery");
  battery_pub_ = battery_node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::QoS(10));
  battery_time_pub_ = battery_node_->create_publisher<std_msgs::msg::Float32>(
    "battery_time_remaining", rclcpp::QoS(10));
  battery_read_initialized_ = false;

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  enc_initialized_ = false;
  rejected_reads_ = 0;
  pending_dt_ = 0.0;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int enc_l = 0;
  int enc_r = 0;
  const bool enc_ok = comms_.read_encoder_values(enc_l, enc_r);

  // lectura periodica beteria 
  if (!battery_read_initialized_ ||
      (time - last_battery_read_).seconds() >= cfg_.battery_publish_period)
  {
    last_battery_read_ = time;
    battery_read_initialized_ = true;
    publish_battery_state(time);
  }

  // Tiempo desde la última lectura aceptada (incluye ciclos descartados), para
  // que el delta de encoder acumulado no se convierta en un pico de velocidad.
  double delta_seconds = period.seconds() + pending_dt_;

  if (!enc_ok)
  {
    // Respuesta corrupta o timeout: se conservan pos/vel del ciclo anterior.
    pending_dt_ = delta_seconds;
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("DiffDriveArduinoHardware"), steady_clock_, 5000,
      "Lectura de encoders inválida, descartada");
    return hardware_interface::return_type::OK;
  }

  if (!enc_initialized_)
  {
    // Primera lectura: solo fija la referencia.
    wheel_l_.enc = enc_l;
    wheel_r_.enc = enc_r;
    enc_initialized_ = true;
    pending_dt_ = 0.0;
    return hardware_interface::return_type::OK;
  }

  const double d_pos_l = (enc_l - wheel_l_.enc) * wheel_l_.rads_per_count;
  const double d_pos_r = (enc_r - wheel_r_.enc) * wheel_r_.rads_per_count;
  const double max_d_pos = cfg_.max_wheel_vel * delta_seconds;

  if (std::abs(d_pos_l) > max_d_pos || std::abs(d_pos_r) > max_d_pos)
  {
    if (++rejected_reads_ < cfg_.max_rejected_reads)
    {
      // Salto físicamente imposible: lectura descartada.
      pending_dt_ = delta_seconds;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("DiffDriveArduinoHardware"), steady_clock_, 5000,
        "Salto de encoder imposible descartado (L %+.1f rad, R %+.1f rad en %.3f s)",
        d_pos_l, d_pos_r, delta_seconds);
      return hardware_interface::return_type::OK;
    }
    // El salto persiste (p. ej. el ESP32 se reinició y sus contadores volvieron a 0):
    // se adopta la lectura como nueva referencia sin mover pos, así la odometría
    // no registra el salto.
    RCLCPP_WARN(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Encoders re-sincronizados tras %d lecturas descartadas (L %d→%d, R %d→%d)",
      rejected_reads_, wheel_l_.enc, enc_l, wheel_r_.enc, enc_r);
    wheel_l_.enc = enc_l;
    wheel_r_.enc = enc_r;
    wheel_l_.vel = 0.0;
    wheel_r_.vel = 0.0;
    rejected_reads_ = 0;
    pending_dt_ = 0.0;
    return hardware_interface::return_type::OK;
  }

  rejected_reads_ = 0;
  pending_dt_ = 0.0;

  // pos se acumula por deltas (no enc * rads_per_count) para que una
  // re-sincronización no la haga saltar.
  wheel_l_.enc = enc_l;
  wheel_l_.pos += d_pos_l;
  wheel_l_.vel = d_pos_l / delta_seconds;

  wheel_r_.enc = enc_r;
  wheel_r_.pos += d_pos_r;
  wheel_r_.vel = d_pos_r / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

void DiffDriveArduinoHardware::publish_battery_state(const rclcpp::Time & time)
{
  float voltage = comms_.read_battery_voltage();

  double range = cfg_.battery_voltage_max - cfg_.battery_voltage_min;
  double percentage = range > 0.0 ? (voltage - cfg_.battery_voltage_min) / range : 0.0;
  percentage = std::clamp(percentage, 0.0, 1.0);

  const float nan = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = time;
  msg.header.frame_id = "base_link";
  msg.voltage = voltage;
  msg.temperature = nan;
  msg.current = nan;
  msg.charge = nan;
  msg.capacity = nan;
  msg.design_capacity = nan;
  msg.percentage = static_cast<float>(percentage);
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  msg.present = true;
  // cell_voltage queda vacío: solo se mide la tensión total del pack, no por celda.
  battery_pub_->publish(msg);

  std_msgs::msg::Float32 time_msg;
  time_msg.data = static_cast<float>(percentage * cfg_.battery_runtime_full_min);
  battery_time_pub_->publish(time_msg);
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
