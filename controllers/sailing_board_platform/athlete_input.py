"""Keyboard-driven athlete input: CoG shifts and applied moments."""

import math

from controller import Keyboard


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def clamp(value, low, high):
    return max(low, min(high, value))


class AthleteKeyboardInput:
    """
    Simulates athlete actions on the sailing board via keyboard.

    CoG shift  -> weight transfer (creates roll/pitch moment from gravity).
    Moment keys -> direct applied torque (hiking, trim, twist).
    """

    def __init__(
        self,
        keyboard,
        mass,
        gravity=9.81,
        neutral_cog_z=1.50,
        cog_rate=0.45,
        cog_limits=(0.55, 0.80, 0.30),
        cog_return_time=0.65,
        moment_rate=220.0,
        moment_limit=700.0,
        moment_return_time=0.45,
    ):
        self.keyboard = keyboard
        self.mass = mass
        self.gravity = gravity
        self.neutral_cog_z = neutral_cog_z
        self.cog_rate = cog_rate
        self.cog_limits = cog_limits
        self.cog_return_time = cog_return_time
        self.moment_rate = moment_rate
        self.moment_limit = moment_limit
        self.moment_return_time = moment_return_time
        self.cog_offset = [0.0, 0.0, 0.0]
        self.applied_moment = [0.0, 0.0, 0.0]
        self._active_keys = set()
        self._previous_keys = set()
        self._last_visual_cog = None

    @staticmethod
    def print_controls():
        print(
            "Athlete keyboard (focus 3D view):\n"
            "  CoG shift   A/D = sway    W/S = surge    Q/E = heave\n"
            "  Moment      Left/Right = roll    Up/Down = pitch    Z/X = yaw\n"
            "  P           Toggle waves on/off (buoyancy stays active)\n"
            "  L           Toggle sail load on/off"
        )

    def poll_keys(self):
        """Rebuild pressed-key state; return keys newly pressed this step."""
        keys_now = set()
        for _ in range(7):
            key = self.keyboard.getKey()
            if key == -1:
                break
            keys_now.add(key & Keyboard.KEY)
        newly_pressed = keys_now - self._previous_keys
        self._previous_keys = keys_now
        self._active_keys = keys_now
        return newly_pressed

    def _pressed(self, char):
        code = ord(char.lower())
        return code in self._active_keys or ord(char.upper()) in self._active_keys

    def _pressed_special(self, code):
        return code in self._active_keys

    @staticmethod
    def _relax_toward_zero(value, time_constant, dt):
        if abs(value) < 1e-5:
            return 0.0
        return value * math.exp(-dt / time_constant)

    def _relax_cog_axis(self, axis_index, active, dt):
        if not active:
            self.cog_offset[axis_index] = self._relax_toward_zero(
                self.cog_offset[axis_index], self.cog_return_time, dt
            )

    def _relax_moment_axis(self, axis_index, active, dt):
        if not active:
            self.applied_moment[axis_index] = self._relax_toward_zero(
                self.applied_moment[axis_index], self.moment_return_time, dt
            )

    def update(self, dt):
        newly_pressed = self.poll_keys()

        if self._pressed("a"):
            self.cog_offset[1] -= self.cog_rate * dt
        if self._pressed("d"):
            self.cog_offset[1] += self.cog_rate * dt
        if self._pressed("w"):
            self.cog_offset[0] += self.cog_rate * dt
        if self._pressed("s"):
            self.cog_offset[0] -= self.cog_rate * dt
        if self._pressed("q"):
            self.cog_offset[2] += self.cog_rate * dt
        if self._pressed("e"):
            self.cog_offset[2] -= self.cog_rate * dt

        if self._pressed_special(Keyboard.LEFT):
            self.applied_moment[0] -= self.moment_rate * dt
        if self._pressed_special(Keyboard.RIGHT):
            self.applied_moment[0] += self.moment_rate * dt
        if self._pressed_special(Keyboard.UP):
            self.applied_moment[1] += self.moment_rate * dt
        if self._pressed_special(Keyboard.DOWN):
            self.applied_moment[1] -= self.moment_rate * dt
        if self._pressed("z"):
            self.applied_moment[2] += self.moment_rate * dt
        if self._pressed("x"):
            self.applied_moment[2] -= self.moment_rate * dt

        self._relax_cog_axis(1, self._pressed("a") or self._pressed("d"), dt)
        self._relax_cog_axis(0, self._pressed("w") or self._pressed("s"), dt)
        self._relax_cog_axis(2, self._pressed("q") or self._pressed("e"), dt)
        self._relax_moment_axis(
            0,
            self._pressed_special(Keyboard.LEFT)
            or self._pressed_special(Keyboard.RIGHT),
            dt,
        )
        self._relax_moment_axis(
            1,
            self._pressed_special(Keyboard.UP)
            or self._pressed_special(Keyboard.DOWN),
            dt,
        )
        self._relax_moment_axis(2, self._pressed("z") or self._pressed("x"), dt)

        self.cog_offset[0] = clamp(self.cog_offset[0], -self.cog_limits[0], self.cog_limits[0])
        self.cog_offset[1] = clamp(self.cog_offset[1], -self.cog_limits[1], self.cog_limits[1])
        self.cog_offset[2] = clamp(self.cog_offset[2], -self.cog_limits[2], self.cog_limits[2])
        for index in range(3):
            self.applied_moment[index] = clamp(
                self.applied_moment[index], -self.moment_limit, self.moment_limit
            )
        return newly_pressed

    def wrench(self):
        """Return (force, torque) in the platform frame from athlete input."""
        weight = [0.0, 0.0, -self.mass * self.gravity]
        # weight × r so the deck leans toward the shifted CoG (not away from it).
        cog_moment = cross(weight, self.cog_offset)
        torque = [
            cog_moment[index] + self.applied_moment[index] for index in range(3)
        ]
        return [0.0, 0.0, 0.0], torque

    def update_visual(self, supervisor, node_def="ATHLETE_FT_SENSOR"):
        if supervisor is None or not supervisor.supervisor:
            return
        cog = tuple(self.cog_offset)
        if cog == self._last_visual_cog:
            return
        self._last_visual_cog = cog
        node = supervisor.getFromDef(node_def)
        if node is None:
            return
        translation = node.getField("translation")
        translation.setSFVec3f([
            self.cog_offset[0],
            self.cog_offset[1],
            self.neutral_cog_z + self.cog_offset[2],
        ])
