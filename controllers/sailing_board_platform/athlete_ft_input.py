"""Force/torque sensor athlete input — drop-in replacement for keyboard wrench()."""

import math


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def clamp(value, low, high):
    return max(low, min(high, value))


def _deadband_vec3(vector, deadband):
    return [
        0.0 if abs(component) < deadband else component
        for component in vector
    ]


class AthleteFTInput:
    """
    Reads a 6-axis wrench from a force/torque sensor (or force + lever-arm estimate).

    ``read_wrench`` must return ``(force, torque)`` in the deck/platform frame.
    Bias is captured at startup (or via ``calibrate()``) and subtracted each step.
    """

    def __init__(
        self,
        read_wrench,
        force_deadband=1.5,
        torque_deadband=0.08,
        filter_hz=8.0,
        calibration_samples=30,
    ):
        self.read_wrench = read_wrench
        self.force_deadband = force_deadband
        self.torque_deadband = torque_deadband
        self.filter_hz = filter_hz
        self.calibration_samples = calibration_samples
        self.force_bias = [0.0, 0.0, 0.0]
        self.torque_bias = [0.0, 0.0, 0.0]
        self.filtered_force = [0.0, 0.0, 0.0]
        self.filtered_torque = [0.0, 0.0, 0.0]
        self.cog_offset = [0.0, 0.0, 0.0]
        self._calibration_buffer = []
        self._calibrated = False

    @staticmethod
    def print_controls():
        print(
            "Athlete F/T input:\n"
            "  Shift weight / hike on the board — wrench drives admittance.\n"
            "  P           Toggle waves on/off (buoyancy stays active)"
        )

    def calibrate(self):
        """Average recent samples into a static bias (call at neutral stance)."""
        if not self._calibration_buffer:
            return
        count = len(self._calibration_buffer)
        self.force_bias = [
            sum(sample[0][axis] for sample in self._calibration_buffer) / count
            for axis in range(3)
        ]
        self.torque_bias = [
            sum(sample[1][axis] for sample in self._calibration_buffer) / count
            for axis in range(3)
        ]
        self._calibration_buffer.clear()
        self._calibrated = True

    def _low_pass(self, previous, sample, alpha):
        return [
            previous[index] + alpha * (sample[index] - previous[index])
            for index in range(3)
        ]

    def update(self, dt):
        raw_force, raw_torque = self.read_wrench()

        if not self._calibrated:
            self._calibration_buffer.append((list(raw_force), list(raw_torque)))
            if len(self._calibration_buffer) >= self.calibration_samples:
                self.calibrate()

        force = [
            raw_force[index] - self.force_bias[index] for index in range(3)
        ]
        torque = [
            raw_torque[index] - self.torque_bias[index] for index in range(3)
        ]

        alpha = clamp(dt * self.filter_hz, 0.0, 1.0)
        self.filtered_force = self._low_pass(self.filtered_force, force, alpha)
        self.filtered_torque = self._low_pass(self.filtered_torque, torque, alpha)

        self.filtered_force = _deadband_vec3(
            self.filtered_force, self.force_deadband
        )
        self.filtered_torque = _deadband_vec3(
            self.filtered_torque, self.torque_deadband
        )
        return set()

    def wrench(self):
        """Return (force, torque) in the platform frame — same contract as keyboard input."""
        return list(self.filtered_force), list(self.filtered_torque)

    @property
    def input_label(self):
        return "F/T sensor"


class AthleteFTFromForce:
    """
    Webots-friendly adapter: TouchSensor force-3d + lever-arm torque estimate.

    Matches the keyboard path where torque comes from ``weight × cog_offset``
    plus applied moments; here torque is ``r × F`` from measured force at the sensor.
    """

    def __init__(
        self,
        force_reader,
        lever_arm,
        force_deadband=1.5,
        torque_deadband=0.08,
        filter_hz=8.0,
        calibration_samples=30,
    ):
        self.lever_arm = list(lever_arm)

        def read_wrench():
            force = list(force_reader())
            torque = cross(self.lever_arm, force)
            return force, torque

        self._ft = AthleteFTInput(
            read_wrench,
            force_deadband=force_deadband,
            torque_deadband=torque_deadband,
            filter_hz=filter_hz,
            calibration_samples=calibration_samples,
        )

    def print_controls(self):
        AthleteFTInput.print_controls()

    def update(self, dt):
        return self._ft.update(dt)

    def wrench(self):
        return self._ft.wrench()

    @property
    def cog_offset(self):
        return self._ft.cog_offset

    @property
    def input_label(self):
        return self._ft.input_label

    @property
    def calibrated(self):
        return self._ft._calibrated
