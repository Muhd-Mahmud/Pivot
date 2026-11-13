// Copyright 2026 PIVOT contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pivot_hardware
{
constexpr double kPi = 3.14159265358979323846;

// All torques and speeds refer to the gearbox OUTPUT; do not multiply by 30 again.
struct MotorModel
{
  double nominal_voltage = 12.0;
  double supply_voltage = 12.0;
  double stall_torque = 0.4707192;
  double reference_rpm = 333.0;  // provisional no-load endpoint, not a verified curve
  double torque_limit = 0.32361945;  // conservative sustained ceiling
  double encoder_counts = 1320.0;

  void validate() const
  {
    for (double value : {nominal_voltage, supply_voltage, stall_torque,
        reference_rpm, torque_limit, encoder_counts})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Motor parameters must be finite and positive");
      }
    }
    if (supply_voltage > nominal_voltage || torque_limit > stall_torque ||
      encoder_counts != std::floor(encoder_counts))
    {
      throw std::invalid_argument("Overvoltage, excessive torque or fractional encoder counts");
    }
  }

  double torque(double requested, double raw_velocity) const
  {
    if (!std::isfinite(requested) || !std::isfinite(raw_velocity) || requested == 0.0) {
      return 0.0;  // zero command is coast; never invent braking at cutoff
    }
    const double direction = std::copysign(1.0, requested);
    const double speed_ratio = direction * raw_velocity / (reference_rpm * 2.0 * kPi / 60.0);
    // Linear voltage/back-EMF envelope. Opposing motion may brake up to the
    // sustained ceiling; same-direction motoring vanishes above the endpoint.
    const double available = std::max(0.0, stall_torque *
      (supply_voltage / nominal_voltage - speed_ratio));
    return direction * std::min({std::abs(requested), available, torque_limit});
  }

  double position(double raw_position) const
  {
    const double radians_per_count = 2.0 * kPi / encoder_counts;
    return std::round(raw_position / radians_per_count) * radians_per_count;
  }
};
}  // namespace pivot_hardware
