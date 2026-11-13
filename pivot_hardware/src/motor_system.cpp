// Copyright 2026 PIVOT contributors
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "pivot_hardware/motor_model.hpp"

namespace pivot_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class YahboomMotorSystem : public gz_ros2_control::GazeboSimSystemInterface
{
  struct Wheel
  {
    std::string name;
    sim::Entity entity;
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;  // last applied actuator torque, not a torque sensor
    double command = 0.0;
    bool claimed = false;
    bool sampled = false;
  };
  MotorModel model_;
  std::vector<Wheel> wheels_;
  sim::EntityComponentManager * ecm_ = nullptr;
  bool active_ = false;
  bool have_time_ = false;
  int64_t last_read_ns_ = 0;

public:
  bool initSim(
    rclcpp::Node::SharedPtr & node, std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & info,
    sim::EntityComponentManager & ecm, unsigned int /*update_rate*/) override
  {
    nh_ = node;
    ecm_ = &ecm;
    try {
      const auto load = [&info](const std::string & name, double & value) {
          const auto it = info.hardware_parameters.find(name);
          if (it != info.hardware_parameters.end()) {
            size_t end = 0;
            value = std::stod(it->second, &end);
            if (end != it->second.size()) {
              throw std::invalid_argument("Invalid motor parameter: " + name);
            }
          }
        };
      load("nominal_voltage", model_.nominal_voltage);
      load("supply_voltage", model_.supply_voltage);
      load("stall_torque", model_.stall_torque);
      load("reference_rpm", model_.reference_rpm);
      load("torque_limit", model_.torque_limit);
      load("encoder_counts", model_.encoder_counts);
      model_.validate();
      if (info.joints.size() != 2) {
        throw std::invalid_argument("PIVOT motor system requires exactly two wheel joints");
      }
      wheels_.reserve(2);
      for (const auto & joint : info.joints) {
        if ((joint.name != "left_wheel_joint" && joint.name != "right_wheel_joint") ||
          joints.count(joint.name) == 0 || joint.command_interfaces.size() != 1 ||
          joint.command_interfaces.front().name != "effort")
        {
          throw std::invalid_argument("Missing wheel or unsupported command interface");
        }
        if (joint.state_interfaces.size() != 3) {
          throw std::invalid_argument("Require position, velocity and effort states");
        }
        for (const auto & name : {"position", "velocity", "effort"}) {
          if (std::none_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
              [name](const auto & state) {return state.name == name;}))
          {
            throw std::invalid_argument("Missing state interface");
          }
        }
        const auto entity = joints.at(joint.name);
        wheels_.push_back(Wheel{joint.name, entity});
        if (!ecm.Component<sim::components::JointPosition>(entity)) {
          ecm.CreateComponent(entity, sim::components::JointPosition());
        }
        if (!ecm.Component<sim::components::JointVelocity>(entity)) {
          ecm.CreateComponent(entity, sim::components::JointVelocity());
        }
        if (!ecm.Component<sim::components::JointForceCmd>(entity)) {
          ecm.CreateComponent(entity, sim::components::JointForceCmd({0.0}));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(nh_->get_logger(), "Yahboom motor configuration: %s", error.what());
      return false;
    }
    RCLCPP_INFO(nh_->get_logger(),
      "MD520Z30 approximation: %.2f V, sustained limit %.6f Nm, "
      "stall %.6f Nm, provisional speed endpoint %.1f rpm, %.0f encoder counts/rev",
      model_.supply_voltage, model_.torque_limit, model_.stall_torque,
      model_.reference_rpm, model_.encoder_counts);
    return true;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> result;
    for (auto & wheel : wheels_) {
      result.emplace_back(wheel.name, "position", &wheel.position);
      result.emplace_back(wheel.name, "velocity", &wheel.velocity);
      result.emplace_back(wheel.name, "effort", &wheel.effort);
    }
    return result;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> result;
    for (auto & wheel : wheels_) {
      result.emplace_back(wheel.name, "effort", &wheel.command);
    }
    return result;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    active_ = true;
    for (auto & wheel : wheels_) {wheel.command = 0.0;}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    for (auto & wheel : wheels_) {
      wheel.command = wheel.effort = 0.0;
      wheel.claimed = false;
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    for (auto & wheel : wheels_) {
      const auto name = wheel.name + "/effort";
      if (std::find(stop.begin(), stop.end(), name) != stop.end()) {
        wheel.claimed = false;
        wheel.command = 0.0;
      }
      if (std::find(start.begin(), start.end(), name) != start.end()) {
        wheel.claimed = true;
        wheel.command = 0.0;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    const double dt = have_time_ ? (time.nanoseconds() - last_read_ns_) * 1e-9 : 0.0;
    // Ignore repeat reads at the same timestamp; preserve the previous velocity.
    if (have_time_ && dt == 0.0) {return hardware_interface::return_type::OK;}
    for (auto & wheel : wheels_) {
      if (!active_) {
        ecm_->Component<sim::components::JointForceCmd>(wheel.entity)->Data() = {0.0};
      }
      const auto * position = ecm_->Component<sim::components::JointPosition>(wheel.entity);
      if (!position || position->Data().empty() || !std::isfinite(position->Data()[0])) {
        wheel.position = wheel.velocity = std::numeric_limits<double>::quiet_NaN();
        wheel.sampled = false;
        continue;
      }
      const double quantized = model_.position(position->Data()[0]);
      wheel.velocity = wheel.sampled && dt > 0.0 && dt <= 0.1 ?
        (quantized - wheel.position) / dt : 0.0;
      wheel.position = quantized;
      wheel.sampled = true;
      if (dt < 0.0) {wheel.command = 0.0;}
    }
    last_read_ns_ = time.nanoseconds();
    have_time_ = true;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto & wheel : wheels_) {
      // Evaluate against physics velocity, never quantized encoder velocity.
      const auto * velocity = ecm_->Component<sim::components::JointVelocity>(wheel.entity);
      wheel.effort = active_ && wheel.claimed && wheel.sampled &&
        velocity && !velocity->Data().empty() ?
        model_.torque(wheel.command, velocity->Data()[0]) : 0.0;
      ecm_->Component<sim::components::JointForceCmd>(wheel.entity)->Data() = {wheel.effort};
    }
    return hardware_interface::return_type::OK;
  }
};
}  // namespace pivot_hardware

PLUGINLIB_EXPORT_CLASS(pivot_hardware::YahboomMotorSystem,
  gz_ros2_control::GazeboSimSystemInterface)
