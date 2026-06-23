"""Minimal sail aerodynamic load on the board (platform frame)."""

import math


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def clamp(value, low, high):
    return max(low, min(high, value))


def clamp_vec3(vector, limit):
    return [clamp(component, -limit, limit) for component in vector]


class MinimalSail:
    """
    Quasi-steady sail model in the board (platform) frame.

    Board axes: x = surge (bow), y = sway (starboard), z = heave (up).
    Wind direction is the compass angle wind blows *from* (world frame).
    Positive side force acts on +y (starboard); hiking to port (−y CoG) opposes it.

    Model: q = ½ρV²,  F ≈ q A Cl(sheet) [drive, side, 0],  τ = r_ce × F.
    """

    def __init__(
        self,
        wind_speed=6.0,
        wind_direction_rad=math.pi / 2.0,
        gust_amplitude=1.0,
        gust_period=5.0,
        sail_area=6.5,
        ce_surge=0.35,
        ce_height=1.85,
        sheet_angle_rad=0.55,
        sheet_min_rad=0.15,
        sheet_max_rad=1.15,
        sheet_rate=0.40,
        rho=1.225,
        cl_max=1.05,
        drive_fraction=0.25,
        force_scale=1.0,
        torque_scale=1.0,
        max_side_force=180.0,
        max_roll_torque=350.0,
        filter_hz=4.0,
        attitude_limit_rad=math.radians(35.0),
    ):
        self.wind_speed = wind_speed
        self.wind_direction_rad = wind_direction_rad
        self.gust_amplitude = gust_amplitude
        self.gust_period = gust_period
        self.sail_area = sail_area
        self.ce_surge = ce_surge
        self.ce_height = ce_height
        self.sheet_angle_rad = sheet_angle_rad
        self.sheet_min_rad = sheet_min_rad
        self.sheet_max_rad = sheet_max_rad
        self.sheet_rate = sheet_rate
        self.rho = rho
        self.cl_max = cl_max
        self.drive_fraction = drive_fraction
        self.force_scale = force_scale
        self.torque_scale = torque_scale
        self.max_side_force = max_side_force
        self.max_roll_torque = max_roll_torque
        self.filter_hz = filter_hz
        self.attitude_limit_rad = attitude_limit_rad
        self.enabled = True
        self.filtered_force = [0.0, 0.0, 0.0]
        self.filtered_torque = [0.0, 0.0, 0.0]
        self.last_force = [0.0, 0.0, 0.0]
        self.last_torque = [0.0, 0.0, 0.0]
        self.last_wind_speed = wind_speed
        self.last_apparent_wind_deg = 0.0

    @staticmethod
    def print_controls():
        print(
            "Sail trim:\n"
            "  R / C       Sheet in / out (more / less power)\n"
            "  L           Toggle sail load on/off\n"
            "  (Sail load opposes hiking — trim and hike to balance heel)"
        )

    def _pressed(self, active_keys, char):
        code = ord(char.lower())
        return code in active_keys or ord(char.upper()) in active_keys

    def update(self, dt, active_keys):
        if self._pressed(active_keys, "r"):
            self.sheet_angle_rad -= self.sheet_rate * dt
        if self._pressed(active_keys, "c"):
            self.sheet_angle_rad += self.sheet_rate * dt
        self.sheet_angle_rad = clamp(
            self.sheet_angle_rad, self.sheet_min_rad, self.sheet_max_rad
        )

    def _wind_speed_at(self, time_s):
        if self.gust_amplitude <= 0.0 or self.gust_period <= 0.0:
            return self.wind_speed
        omega = 2.0 * math.pi / self.gust_period
        return self.wind_speed + self.gust_amplitude * math.sin(omega * time_s)

    @staticmethod
    def _lift_coefficient(sheet_angle_rad, cl_max):
        optimum = 0.55
        span = 0.55
        normalized = (sheet_angle_rad - optimum) / span
        return clamp(cl_max * math.exp(-normalized * normalized), 0.0, cl_max)

    def _low_pass_vec3(self, previous, sample, alpha):
        return [
            previous[index] + alpha * (sample[index] - previous[index])
            for index in range(3)
        ]

    def _clamp_attitude(self, deck_rpy):
        return tuple(
            clamp(angle, -self.attitude_limit_rad, self.attitude_limit_rad)
            for angle in deck_rpy
        )

    def wrench(self, time_s, deck_rpy, dt):
        """
        Return (force, torque) at the platform origin from sail pressure at the CE.

        deck_rpy: (roll, pitch, yaw) deck attitude relative to neutral (rad).
        """
        if not self.enabled:
            self.filtered_force = [0.0, 0.0, 0.0]
            self.filtered_torque = [0.0, 0.0, 0.0]
            self.last_force = self.filtered_force
            self.last_torque = self.filtered_torque
            return self.last_force, self.last_torque

        roll, pitch, yaw = self._clamp_attitude(deck_rpy)
        wind_speed = self._wind_speed_at(time_s)
        self.last_wind_speed = wind_speed

        wind_vector = [
            math.cos(self.wind_direction_rad) * wind_speed,
            math.sin(self.wind_direction_rad) * wind_speed,
            0.0,
        ]

        cy, sy = math.cos(yaw), math.sin(yaw)
        wind_board = [
            cy * wind_vector[0] + sy * wind_vector[1],
            -sy * wind_vector[0] + cy * wind_vector[1],
            0.0,
        ]

        wind_speed_board = math.hypot(wind_board[0], wind_board[1])
        if wind_speed_board < 1e-3:
            self.filtered_force = [0.0, 0.0, 0.0]
            self.filtered_torque = [0.0, 0.0, 0.0]
            self.last_force = self.filtered_force
            self.last_torque = self.filtered_torque
            self.last_apparent_wind_deg = 0.0
            return self.last_force, self.last_torque

        flow_dir = math.atan2(-wind_board[1], -wind_board[0])
        self.last_apparent_wind_deg = math.degrees(flow_dir)

        cl = self._lift_coefficient(self.sheet_angle_rad, self.cl_max)
        dynamic_pressure = 0.5 * self.rho * wind_speed_board * wind_speed_board
        magnitude = dynamic_pressure * self.sail_area * cl

        drive = magnitude * math.cos(flow_dir) * self.drive_fraction
        side = magnitude * math.sin(flow_dir)

        # Fade out with heel instead of flipping sign past 90° (prevents runaway capsizes).
        heel_factor = max(0.0, math.cos(roll)) * max(0.0, math.cos(pitch))
        side *= heel_factor

        force = [drive * heel_factor, side, 0.0]
        ce_position = [self.ce_surge, 0.0, self.ce_height]
        torque = cross(ce_position, force)

        force = [component * self.force_scale for component in force]
        torque = [component * self.torque_scale for component in torque]
        force[1] = clamp(force[1], -self.max_side_force, self.max_side_force)
        torque[0] = clamp(torque[0], -self.max_roll_torque, self.max_roll_torque)

        alpha = clamp(dt * self.filter_hz, 0.0, 1.0)
        self.filtered_force = self._low_pass_vec3(self.filtered_force, force, alpha)
        self.filtered_torque = self._low_pass_vec3(self.filtered_torque, torque, alpha)

        self.last_force = list(self.filtered_force)
        self.last_torque = list(self.filtered_torque)
        return self.last_force, self.last_torque

    def status_line(self):
        state = "ON" if self.enabled else "OFF"
        return (
            f"Sail {state} wind={self.last_wind_speed:.1f}m/s "
            f"AWA={self.last_apparent_wind_deg:+.0f}° "
            f"sheet={math.degrees(self.sheet_angle_rad):.0f}° "
            f"F_side={self.last_force[1]:+.0f}N "
            f"τ_roll={self.last_torque[0]:+.0f}N·m"
        )


def combine_pose_deltas(delta_a, delta_b):
    return tuple(left + right for left, right in zip(delta_a, delta_b))
