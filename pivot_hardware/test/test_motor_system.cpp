// Copyright 2026 PIVOT contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>
#include <memory>
#include <gtest/gtest.h>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "pivot_hardware/motor_model.hpp"

TEST(MotorSystem, InstalledPluginEnforcesLimitsAndSamplesEncoders)
{
  rclcpp::init(0, nullptr);
  {
    pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader(
      "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface");
    auto system = loader.createSharedInstance("pivot_hardware/YahboomMotorSystem");
    auto node = std::make_shared<rclcpp::Node>("pivot_motor_system_test");
    sim::EntityComponentManager ecm;
    std::map<std::string, sim::Entity> joints;
    hardware_interface::HardwareInfo info;
    info.name = "GazeboSystem";
    info.type = "system";
    info.hardware_plugin_name = "pivot_hardware/YahboomMotorSystem";
    for (const auto & name : {"left_wheel_joint", "right_wheel_joint"}) {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      joint.type = "joint";
      hardware_interface::InterfaceInfo effort;
      effort.name = "effort";
      joint.command_interfaces.push_back(effort);
      for (const auto & state_name : {"position", "velocity", "effort"}) {
        hardware_interface::InterfaceInfo state;
        state.name = state_name;
        joint.state_interfaces.push_back(state);
      }
      info.joints.push_back(joint);
      joints[name] = ecm.CreateEntity();
    }
    ASSERT_TRUE(system->initSim(node, joints, info, ecm, 200));
    EXPECT_EQ(system->on_init(info),
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    auto states = system->export_state_interfaces();
    auto commands = system->export_command_interfaces();
    ASSERT_EQ(commands.size(), 2u);
    ASSERT_EQ(states.size(), 6u);
    const auto left = joints.at("left_wheel_joint");
    const auto right = joints.at("right_wheel_joint");
    for (auto entity : {left, right}) {
      ecm.Component<sim::components::JointPosition>(entity)->Data() = {0.0};
      ecm.Component<sim::components::JointVelocity>(entity)->Data() = {0.0};
    }
    const auto at = [](int64_t ns) {return rclcpp::Time(ns, RCL_ROS_TIME);};
    const auto period = rclcpp::Duration::from_seconds(0.005);
    system->read(at(0), period);
    system->on_activate(rclcpp_lifecycle::State());
    system->perform_command_mode_switch({"left_wheel_joint/effort", "right_wheel_joint/effort"}, {});
    ASSERT_TRUE(commands[0].set_value(2.0));
    ASSERT_TRUE(commands[1].set_value(-2.0));
    system->write(at(0), period);
    const auto torque = [&ecm](sim::Entity entity) {
        return ecm.Component<sim::components::JointForceCmd>(entity)->Data()[0];
      };
    EXPECT_NEAR(torque(left), 0.32361945, 1e-12);
    EXPECT_NEAR(torque(right), -0.32361945, 1e-12);

    const double tick = 2 * pivot_hardware::kPi / 1320;
    ecm.Component<sim::components::JointPosition>(left)->Data() = {1.2 * tick};
    system->read(at(5000000), period);
    EXPECT_NEAR(states[0].get_value(), tick, 1e-12);
    EXPECT_NEAR(states[1].get_value(), tick / 0.005, 1e-12);
    system->read(at(5000000), period);
    EXPECT_NEAR(states[1].get_value(), tick / 0.005, 1e-12);

    // A different publisher sending huge effort still cannot bypass the hardware.
    ecm.Component<sim::components::JointVelocity>(left)->Data() = {100.0};
    system->write(at(6000000), period);
    EXPECT_DOUBLE_EQ(torque(left), 0.0);
    EXPECT_NEAR(torque(right), -0.32361945, 1e-12);
    ASSERT_TRUE(commands[0].set_value(-100.0));
    system->write(at(7000000), period);
    EXPECT_NEAR(torque(left), -0.32361945, 1e-12);
    ASSERT_TRUE(commands[0].set_value(0.0));
    system->write(at(8000000), period);
    EXPECT_DOUBLE_EQ(torque(left), 0.0);

    // Bad feedback, controller stop, reset and deactivation must remove drive.
    ASSERT_TRUE(commands[0].set_value(0.2));
    ecm.Component<sim::components::JointVelocity>(left)->Data() =
      {std::numeric_limits<double>::quiet_NaN()};
    system->write(at(9000000), period);
    EXPECT_DOUBLE_EQ(torque(left), 0.0);
    system->perform_command_mode_switch({}, {"right_wheel_joint/effort"});
    system->write(at(10000000), period);
    EXPECT_DOUBLE_EQ(torque(right), 0.0);
    system->read(at(0), period);
    EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);
    EXPECT_DOUBLE_EQ(commands[0].get_value(), 0.0);
    system->on_deactivate(rclcpp_lifecycle::State());
    system->write(at(1000000), period);
    EXPECT_DOUBLE_EQ(torque(left), 0.0);
    EXPECT_DOUBLE_EQ(torque(right), 0.0);
  }
  rclcpp::shutdown();
}
