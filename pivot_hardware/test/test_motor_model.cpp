// Copyright 2026 PIVOT contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "pivot_hardware/motor_model.hpp"

using pivot_hardware::MotorModel;
using pivot_hardware::kPi;

TEST(MotorModel, SustainedCapIsIndependentOfCommandMagnitude)
{
  MotorModel motor;
  motor.validate();
  EXPECT_DOUBLE_EQ(motor.torque(2.0, 0.0), motor.torque_limit);
  EXPECT_DOUBLE_EQ(motor.torque(-2.0, 0.0), -motor.torque_limit);
  EXPECT_DOUBLE_EQ(motor.torque(0.05, 0.0), 0.05);
}

TEST(MotorModel, ForwardAndReverseMotoringLoseTorqueWithSpeed)
{
  MotorModel motor;
  const double endpoint = motor.reference_rpm * 2 * kPi / 60;
  EXPECT_NEAR(motor.torque(2.0, endpoint / 2), motor.stall_torque / 2, 1e-12);
  EXPECT_NEAR(motor.torque(-2.0, -endpoint / 2), -motor.stall_torque / 2, 1e-12);
  EXPECT_NEAR(motor.torque(2.0, endpoint), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(motor.torque(2.0, endpoint * 2), 0.0);
  EXPECT_DOUBLE_EQ(motor.torque(-2.0, -endpoint * 2), 0.0);
  // Braking remains possible at overspeed, without exceeding the current proxy cap.
  EXPECT_DOUBLE_EQ(motor.torque(-2.0, endpoint * 2), -motor.torque_limit);
  EXPECT_DOUBLE_EQ(motor.torque(2.0, -endpoint * 2), motor.torque_limit);
}

TEST(MotorModel, LowerSupplyReducesAvailableTorqueAndEndpoint)
{
  MotorModel motor;
  motor.supply_voltage = 6.0;
  motor.validate();
  EXPECT_DOUBLE_EQ(motor.torque(2.0, 0.0), motor.stall_torque / 2);
  EXPECT_NEAR(motor.torque(2.0, motor.reference_rpm * kPi / 60), 0.0, 1e-12);
}

TEST(MotorModel, CoastAndInvalidMeasurementsProduceZero)
{
  MotorModel motor;
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(motor.torque(0.0, 100.0), 0.0);
  EXPECT_DOUBLE_EQ(motor.torque(nan, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(motor.torque(inf, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(motor.torque(0.1, nan), 0.0);
  EXPECT_DOUBLE_EQ(motor.torque(0.1, inf), 0.0);
}

TEST(MotorModel, EncoderQuantizesBothDirectionsAndMultipleRevolutions)
{
  MotorModel motor;
  const double tick = 2 * kPi / 1320;
  EXPECT_DOUBLE_EQ(motor.position(0.49 * tick), 0.0);
  EXPECT_NEAR(motor.position(0.51 * tick), tick, 1e-12);
  EXPECT_NEAR(motor.position(-0.51 * tick), -tick, 1e-12);
  EXPECT_NEAR(motor.position(20 * kPi + 0.2 * tick), 20 * kPi, 1e-12);
}

TEST(MotorModel, RejectsUnphysicalConfiguration)
{
  MotorModel motor;
  motor.reference_rpm = 0;
  EXPECT_THROW(motor.validate(), std::invalid_argument);
  motor = MotorModel{};
  motor.torque_limit = 2;
  EXPECT_THROW(motor.validate(), std::invalid_argument);
  motor = MotorModel{};
  motor.supply_voltage = 24;
  EXPECT_THROW(motor.validate(), std::invalid_argument);
  motor = MotorModel{};
  motor.encoder_counts = 1320.5;
  EXPECT_THROW(motor.validate(), std::invalid_argument);
  motor = MotorModel{};
  motor.stall_torque = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(motor.validate(), std::invalid_argument);
}
