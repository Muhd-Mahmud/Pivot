#!/usr/bin/env python3
"""
PIVOT controller v3.3: pitch PID with direct speed torque damping.

The baseline pitch PID and the ROS interfaces remain unchanged. An extra
wheel-speed damping torque opposes measured wheel speed while the pitch
loop remains responsible for keeping the robot upright. Both motion and yaw
loops are OFF by default. Only use in Gazebo until physically validated.
"""

import math

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def quaternion_to_pitch(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, sinp)))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class BalanceNode(Node):
    def __init__(self):
        super().__init__('balance_node')

        # Existing tuning and safety parameters (unchanged).
        self.declare_parameter('kp', 8.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.4)
        self.declare_parameter('setpoint', 0.0)
        self.declare_parameter('max_tilt', 0.6)
        # MD520Z30 conservative sustained torque; Gazebo also enforces its
        # independent per-wheel motor envelope after the controller command.
        self.declare_parameter('max_effort', 0.32361945)

        # New parameters. Velocity control is intentionally disabled by default.
        # Wheel radius is a proposed PIVOT design dimension; verify against URDF.
        self.declare_parameter('wheel_radius', 0.0335)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)
        self.declare_parameter('enable_velocity_control', False)
        self.declare_parameter('speed_kp', 0.12)  # rad of pitch per (m/s) error
        self.declare_parameter(
            'velocity_pitch_sign', -1.0
        )  # physically inferred; verify
        self.declare_parameter(
            'max_pitch_offset', 0.02
        )  # rad (~1.15 deg); gentle first test
        # Additional direct wheel-speed damping, separate from pitch offset.
        # Positive forward wheel speed produces NEGATIVE corrective torque.
        # Nm per (m/s); tuning is preliminary and simulation-only.
        self.declare_parameter('speed_effort_kp', 0.40)
        self.declare_parameter('max_speed_effort', 0.40)  # Nm per wheel
        self.declare_parameter('max_speed_command', 0.10)  # m/s
        self.declare_parameter('max_speed_ramp', 0.10)  # m/s^2
        self.declare_parameter('command_timeout', 0.5)  # seconds
        self.declare_parameter('joint_state_timeout', 0.2)  # seconds

        # Optional zero-command station hold (requires velocity control too).
        # Gains are conservative starting points, NOT validated on this robot.
        self.declare_parameter('enable_station_hold', False)
        self.declare_parameter('position_kp', 0.20)  # m/s desired per m error
        self.declare_parameter('max_hold_speed', 0.03)  # m/s
        self.declare_parameter('wheel_separation', 0.158)  # m, from URDF

        # Optional differential yaw damping. OFF until sign is validated.
        self.declare_parameter('enable_yaw_damping', False)
        self.declare_parameter('yaw_kd', 0.05)  # Nm/(rad/s)
        self.declare_parameter('max_yaw_effort', 0.10)  # Nm, differential correction

        def p(name):
            return self.get_parameter(name).value

        self.kp = float(p('kp'))
        self.ki = float(p('ki'))
        self.kd = float(p('kd'))
        self.setpoint = float(p('setpoint'))
        self.max_tilt = float(p('max_tilt'))
        self.max_effort = float(p('max_effort'))

        self.wheel_radius = float(p('wheel_radius'))
        self.left_wheel_sign = float(p('left_wheel_sign'))
        self.right_wheel_sign = float(p('right_wheel_sign'))
        self.enable_velocity_control = bool(p('enable_velocity_control'))
        self.speed_kp = float(p('speed_kp'))
        self.velocity_pitch_sign = float(p('velocity_pitch_sign'))
        self.max_pitch_offset = float(p('max_pitch_offset'))
        self.speed_effort_kp = float(p('speed_effort_kp'))
        self.max_speed_effort = float(p('max_speed_effort'))
        self.max_speed_command = float(p('max_speed_command'))
        self.max_speed_ramp = float(p('max_speed_ramp'))
        self.command_timeout = float(p('command_timeout'))
        self.joint_state_timeout = float(p('joint_state_timeout'))
        self.enable_station_hold = bool(p('enable_station_hold'))
        self.position_kp = float(p('position_kp'))
        self.max_hold_speed = float(p('max_hold_speed'))
        self.wheel_separation = float(p('wheel_separation'))
        self.enable_yaw_damping = bool(p('enable_yaw_damping'))
        self.yaw_kd = float(p('yaw_kd'))
        self.max_yaw_effort = float(p('max_yaw_effort'))
        if self.enable_station_hold and not self.enable_velocity_control:
            self.get_logger().warning(
                'Station hold is requested but velocity control is OFF. '
                'Station hold will remain inactive.'
            )

        self.integral = 0.0
        self.last_time = None
        self.last_report_time = None
        self.last_joint_time = None
        self.last_cmd_time = None
        self.target_speed = 0.0
        self.ramped_speed = 0.0
        self.measured_speed = 0.0
        self.have_wheel_feedback = False
        self.have_wheel_position = False
        self.wheel_position_m = 0.0
        self.yaw_rate_from_wheels = 0.0
        self.yaw_rate_imu = 0.0
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self.hold_reference_m = None
        self.prev_motion_request = False

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/wheel_effort_controller/commands', 10
        )
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.wheel_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos_profile_sensor_data
        )
        self.velocity_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10
        )

        # Validate before applying; update live state only after ROS accepts it.
        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.add_post_set_parameters_callback(self.after_parameter_update)

        self.get_logger().info(
            'Balance node started; forward velocity control %s. '
            'Station hold %s. velocity_pitch_sign=%.0f, speed_kp=%.3f, '
            'max_pitch_offset=%.3f. Direct speed torque damping available; '
            'yaw off by default.'
            % (
                'ENABLED' if self.enable_velocity_control else 'DISABLED',
                'ENABLED' if self.enable_station_hold else 'DISABLED',
                self.velocity_pitch_sign,
                self.speed_kp,
                self.max_pitch_offset,
            )
        )

    def on_parameter_update(self, params):
        """
        Apply opt-in outer-loop settings dynamically, without restarting Gazebo.

        Values are staged and checked before changing the live controller.
        The fast pitch PID parameters remain launch-time settings.
        """
        startup_only = {
            'kp',
            'ki',
            'kd',
            'setpoint',
            'max_tilt',
            'max_effort',
            'wheel_radius',
            'left_wheel_sign',
            'right_wheel_sign',
            'wheel_separation',
            'max_speed_command',
            'max_speed_ramp',
            'command_timeout',
            'joint_state_timeout',
        }
        bounds = {
            'speed_kp': (0.0, 1.0),
            'max_pitch_offset': (0.0, 0.10),
            'speed_effort_kp': (0.0, 2.0),
            'max_speed_effort': (0.0, 1.0),
            'position_kp': (0.0, 2.0),
            'max_hold_speed': (0.0, 0.15),
            'yaw_kd': (0.0, 1.0),
            'max_yaw_effort': (0.0, 0.30),
        }
        booleans = {
            'enable_velocity_control',
            'enable_station_hold',
            'enable_yaw_damping',
        }
        updates = {}
        for param in params:
            name, value = param.name, param.value
            if name in startup_only:
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} is startup-only; change launch configuration and restart',
                )
            if name in booleans:
                if not isinstance(value, bool):
                    return SetParametersResult(
                        successful=False, reason=f'{name} must be bool'
                    )
                updates[name] = value
            elif name in bounds:
                lo, hi = bounds[name]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or not lo <= value <= hi
                ):
                    return SetParametersResult(
                        successful=False, reason=f'{name} must be in [{lo}, {hi}]'
                    )
                updates[name] = float(value)
            elif name == 'velocity_pitch_sign':
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or value not in (-1.0, 1.0)
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='velocity_pitch_sign must be -1.0 or +1.0',
                    )
                updates[name] = float(value)

        velocity_enabled = updates.get(
            'enable_velocity_control', self.enable_velocity_control
        )
        station_enabled = updates.get('enable_station_hold', self.enable_station_hold)
        if station_enabled and not velocity_enabled:
            return SetParametersResult(
                successful=False,
                reason='enable_velocity_control first, then enable_station_hold',
            )

        # This validation callback must not mutate state: another callback can
        # still reject the parameter update. See ROS 2 Jazzy parameter API.
        return SetParametersResult(successful=True)

    def after_parameter_update(self, params):
        """Apply *accepted* parameter changes to the running controller."""
        live_names = {
            'enable_velocity_control',
            'enable_station_hold',
            'speed_kp',
            'velocity_pitch_sign',
            'max_pitch_offset',
            'speed_effort_kp',
            'max_speed_effort',
            'position_kp',
            'max_hold_speed',
            'enable_yaw_damping',
            'yaw_kd',
            'max_yaw_effort',
        }
        updates = {p.name: p.value for p in params if p.name in live_names}
        changed = {k: v for k, v in updates.items() if getattr(self, k) != v}
        for name, value in updates.items():
            setattr(self, name, value)
        if changed:
            self.ramped_speed = 0.0
            self.hold_reference_m = None
            self.prev_motion_request = False
            self.integral = 0.0
            self.get_logger().warning(
                'Controller settings applied: '
                + ', '.join(f'{name}={value}' for name, value in changed.items())
                + '; motion references reset'
            )

    def joint_callback(self, msg):
        if len(msg.velocity) != len(msg.name):
            return
        try:
            il = msg.name.index('left_wheel_joint')
            ir = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        vl = self.left_wheel_sign * msg.velocity[il]
        vr = self.right_wheel_sign * msg.velocity[ir]
        if not (math.isfinite(vl) and math.isfinite(vr)):
            return

        self.left_wheel_speed = self.wheel_radius * vl
        self.right_wheel_speed = self.wheel_radius * vr
        self.measured_speed = 0.5 * (self.left_wheel_speed + self.right_wheel_speed)
        if self.wheel_separation > 0.0:
            self.yaw_rate_from_wheels = (
                self.wheel_radius * (vr - vl) / self.wheel_separation
            )
        if len(msg.position) == len(msg.name):
            pl = self.left_wheel_sign * msg.position[il]
            pr = self.right_wheel_sign * msg.position[ir]
            if math.isfinite(pl) and math.isfinite(pr):
                self.wheel_position_m = 0.5 * self.wheel_radius * (pl + pr)
                self.have_wheel_position = True
        self.have_wheel_feedback = True
        self.last_joint_time = self.get_clock().now()

    def velocity_callback(self, msg):
        if not math.isfinite(msg.linear.x):
            return
        self.target_speed = clamp(
            msg.linear.x, -self.max_speed_command, self.max_speed_command
        )
        self.last_cmd_time = self.get_clock().now()
        # Angular.z is deliberately ignored until steering geometry and
        # wheel/motor direction have been validated.

    def imu_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            self.integral = 0.0
            self.ramped_speed = 0.0
            self.hold_reference_m = None
            self.prev_motion_request = False
            self.last_report_time = None
            if dt <= 0.0:  # Simulation clock jumped backward or repeated.
                self.last_joint_time = None
                self.last_cmd_time = None
                self.have_wheel_feedback = False
                self.have_wheel_position = False
            self.publish_effort(0.0)  # Do not retain the preceding motor torque.
            return

        q = msg.orientation
        components = (q.x, q.y, q.z, q.w)
        norm2 = sum(v * v for v in components)
        if not all(math.isfinite(v) for v in components) or norm2 < 0.5 or norm2 > 1.5:
            self.publish_effort(0.0)
            return
        # Normalize within the accepted range to avoid orientation drift.
        pitch = math.asin(clamp(2.0 * (q.w * q.y - q.z * q.x) / norm2, -1.0, 1.0))
        # Pitch alone misses a robot tipped over sideways. Check the body's
        # Z axis against world-up: cos(tilt) = R[2,2] of the quaternion.
        upright_cos = 1.0 - 2.0 * (q.x * q.x + q.y * q.y) / norm2
        upright_cos = clamp(upright_cos, -1.0, 1.0)
        if not (math.isfinite(pitch) and math.isfinite(msg.angular_velocity.y)):
            self.publish_effort(0.0)
            return

        if abs(pitch) > self.max_tilt or upright_cos < math.cos(self.max_tilt):
            self.integral = 0.0
            self.ramped_speed = 0.0
            self.hold_reference_m = None
            self.publish_effort(0.0)
            return

        self.yaw_rate_imu = (
            msg.angular_velocity.z if math.isfinite(msg.angular_velocity.z) else 0.0
        )
        desired_pitch = self.setpoint
        pitch_offset = 0.0
        speed_error = 0.0
        speed_effort = 0.0
        wheel_fresh = False
        if self.enable_velocity_control:
            command_fresh = (
                self.last_cmd_time is not None
                and 0.0
                <= (now - self.last_cmd_time).nanoseconds * 1e-9
                <= self.command_timeout
            )
            wheel_fresh = (
                self.have_wheel_feedback
                and self.last_joint_time is not None
                and 0.0
                <= (now - self.last_joint_time).nanoseconds * 1e-9
                <= self.joint_state_timeout
            )

            desired_speed = self.target_speed if command_fresh else 0.0

            # Station keeping is applied only in zero-command mode, not while
            # driving. When motion ceases, capture the current wheel position
            # rather than returning to the original launch position.
            motion_request = abs(desired_speed) > 1e-3
            if motion_request:
                self.hold_reference_m = None
            elif self.prev_motion_request:
                self.hold_reference_m = None
            self.prev_motion_request = motion_request

            if (
                self.enable_station_hold
                and not motion_request
                and self.have_wheel_position
                and wheel_fresh
            ):
                if self.hold_reference_m is None:
                    self.hold_reference_m = self.wheel_position_m
                pos_err = self.hold_reference_m - self.wheel_position_m
                hold_correction = clamp(
                    self.position_kp * pos_err,
                    -self.max_hold_speed,
                    self.max_hold_speed,
                )
                desired_speed += hold_correction
            change_limit = self.max_speed_ramp * dt
            self.ramped_speed += clamp(
                desired_speed - self.ramped_speed, -change_limit, change_limit
            )

            if wheel_fresh:
                speed_error = self.ramped_speed - self.measured_speed
                pitch_offset = clamp(
                    self.velocity_pitch_sign * self.speed_kp * speed_error,
                    -self.max_pitch_offset,
                    self.max_pitch_offset,
                )
                desired_pitch = self.setpoint + pitch_offset
                # Negative feedback in the wheel-effort domain. For a positive
                # desired_speed - measured_speed error, command forward effort.
                # This adds direct speed damping even when pitch offset saturates.
                speed_effort = clamp(
                    self.speed_effort_kp * speed_error,
                    -self.max_speed_effort,
                    self.max_speed_effort,
                )
            else:
                # No fresh wheel measurement: retain the original pitch-only PID.
                self.ramped_speed = 0.0

        error = pitch - desired_pitch
        self.integral = clamp(self.integral + error * dt, -1.0, 1.0)
        derivative = msg.angular_velocity.y
        pitch_effort = self.kp * error + self.ki * self.integral + self.kd * derivative
        effort = pitch_effort + speed_effort
        effort = clamp(effort, -self.max_effort, self.max_effort)

        # Split the original pitch torque only when explicitly enabled. For
        # +Y wheel axes, more right-wheel torque produces +Z yaw; therefore
        # oppose +Z yaw by adding to left and subtracting from right.
        yaw_correction = 0.0
        if self.enable_yaw_damping and math.isfinite(msg.angular_velocity.z):
            yaw_correction = clamp(
                self.yaw_kd * self.yaw_rate_imu,
                -self.max_yaw_effort,
                self.max_yaw_effort,
            )
        left_effort = clamp(effort + yaw_correction, -self.max_effort, self.max_effort)
        right_effort = clamp(effort - yaw_correction, -self.max_effort, self.max_effort)
        self.publish_effort(left_effort, right_effort)

        # Read-only diagnostics, every 0.5 seconds of simulation time.
        if (
            self.last_report_time is None
            or (now - self.last_report_time).nanoseconds * 1e-9 >= 0.5
        ):
            self.last_report_time = now
            self.get_logger().info(
                f'pitch={math.degrees(pitch):+.2f} deg, '
                f'wheel_speed={self.measured_speed:+.3f} m/s '
                f'(verify signs), pos={self.wheel_position_m:+.3f} m '
                f'(encoder estimate), yaw_rate={self.yaw_rate_from_wheels:+.2f} rad/s, '
                f'desired_pitch={math.degrees(desired_pitch):+.2f} deg, '
                f'pitch_offset={math.degrees(pitch_offset):+.2f} deg, '
                f'speed_error={speed_error:+.3f} m/s, wheel_fresh={wheel_fresh}, '
                f'pitch_effort={pitch_effort:+.3f}, '
                f'speed_effort={speed_effort:+.3f}, '
                f'vL={self.left_wheel_speed:+.3f} m/s, '
                f'vR={self.right_wheel_speed:+.3f} m/s, '
                f'imu_yaw_rate={self.yaw_rate_imu:+.3f} rad/s, '
                f'effort_L={left_effort:+.3f}, effort_R={right_effort:+.3f}, '
                f'velocity_ctrl={self.enable_velocity_control}, '
                f'station_hold={self.enable_station_hold}, '
                f'yaw_damping={self.enable_yaw_damping}'
            )

    def publish_effort(self, left_effort, right_effort=None):
        if right_effort is None:
            right_effort = left_effort
        msg = Float64MultiArray()
        msg.data = [float(left_effort), float(right_effort)]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
