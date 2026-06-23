# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Webots controller for the Stewart platform sailing-board simulator.

Sea motion (SEA_STATE) drives waves on top of buoyancy. Keyboard athlete input
(CoG shift + applied moments) perturbs the deck; CoG returns smoothly at rest.

Focus the 3D view before using keyboard controls.

Deck pose is measured with GPS (deck_gps) and InertialUnit (deck_inertial_unit).
Both are zeroed to the neutral deck pose at startup.
"""

from controller import Supervisor, TouchSensor
import math

from admittance import AdmittanceControl
from athlete_input import AthleteKeyboardInput
from buoyancy import BuoyancyDynamics
from force_plotter import try_create_plotter

TIME_STEP = 64
NUM_PISTONS = 6
PISTON_MIN = -0.4
PISTON_MAX = 0.4
NEUTRAL_LEG_LENGTH = 2.8
ATHLETE_MASS = 70.0
NEUTRAL_COG_Z = 1.50

# Athlete range — approx. full hiking / trim / pump on a windsurf-style board.
# Order for limits: surge (x), sway (y), heave (z).
COG_LIMITS = (0.55, 0.80, 0.30)
COG_RATE = 0.45
MOMENT_LIMIT = 700.0
MOMENT_RATE = 220.0

PLOT_HISTORY_SECONDS = 30.0
ENABLE_LIVE_PLOTS = True
PLOT_UPDATE_INTERVAL = 0.25
CONSOLE_LOG_INTERVAL = 1.0

# Diagonal admittance gains: pose_delta = gain * wrench component.
# Tuned so full CoG hike + moment keys can reach ~25–35° heel.
TRANSLATION_ADMITTANCE = (0.0012, 0.0012, 0.0008)
ROTATION_ADMITTANCE = (0.0010, 0.0010, 0.0005)

# Buoyancy: track_stiffness follows target, buoyancy_stiffness restores to waterline.
# Order: surge, sway, heave, roll, pitch, yaw.
TRACK_STIFFNESS = (10.0, 10.0, 14.0, 9.0, 9.0, 6.0)
BUOYANCY_STIFFNESS = (5.0, 5.0, 9.0, 6.0, 6.0, 4.0)
HYDRO_DAMPING = (1.8, 1.8, 2.5, 0.9, 0.9, 0.7)

# Geometry extracted from stewart_platform.wbt at the neutral configuration.
BASE_ANCHORS = [
    [-1.20951, 1.57453, 0.120577],
    [-1.20951, -1.57453, 0.120577],
    [-0.75883, -1.83473, 0.120577],
    [1.96834, -0.260201, 0.120577],
    [1.96834, 0.260201, 0.120577],
    [-0.75883, 1.83473, 0.120577],
]

PLATFORM_NEUTRAL_TRANSLATION = [0.514172, 0.235049, 0.917719]
PLATFORM_NEUTRAL_ROTATION = (
    [0.7176690794221499, -0.12654500989810902, 0.6847900794486234],
    2.65306,
)

PLATFORM_LOCAL_ANCHORS = [
    [1.4685, 1.0452, -1.2626],
    [1.4097, 1.3792, -1.0668],
    [1.4097, 1.3792, 0.4113],
    [1.4686, 1.0452, 0.6071],
    [1.6908, -0.2154, -0.1320],
    [1.6908, -0.2154, -0.5236],
]

SEA_STATE = {
    "heave": [
        (0.08, 6.0, 0.0),
        (0.04, 3.5, 1.2),
    ],
    "surge": [
        (0.03, 7.0, 0.5),
    ],
    "sway": [
        (0.025, 5.5, 2.0),
    ],
    "roll": [
        (0.10, 5.0, 0.0),
        (0.04, 2.8, 0.8),
    ],
    "pitch": [
        (0.07, 4.5, 1.5),
        (0.03, 8.0, 0.0),
    ],
    "yaw": [
        (0.02, 9.0, 0.0),
    ],
}


def axis_angle_to_matrix(axis, angle):
    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / length, y / length, z / length
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ]


def mat_vec(matrix, vector):
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def mat_mul(left, right):
    return [
        [sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def euler_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def clamp(value, low, high):
    return max(low, min(high, value))


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def estimate_torque(force, lever_arm):
    return cross(lever_arm, force)


def athlete_lever_arm(cog_offset, neutral_cog_z):
    return [cog_offset[0], cog_offset[1], neutral_cog_z + cog_offset[2]]


def wave_sum(components, time):
    total = 0.0
    for amplitude, period, phase in components:
        omega = 2.0 * math.pi / period
        total += amplitude * math.sin(omega * time + phase)
    return total


def format_pose_degrees(pose):
    surge, sway, heave, roll, pitch, yaw = pose
    return (
        f"Δsurge={surge:+.3f}m Δsway={sway:+.3f}m Δheave={heave:+.3f}m "
        f"roll={math.degrees(roll):+.1f}° pitch={math.degrees(pitch):+.1f}° "
        f"yaw={math.degrees(yaw):+.1f}°"
    )


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def read_position_delta(gps, neutral_xyz):
    x, y, z = gps.getValues()
    return (
        x - neutral_xyz[0],
        y - neutral_xyz[1],
        z - neutral_xyz[2],
    )


def read_orientation_delta(inertial_unit, neutral_rpy):
    roll, pitch, yaw = inertial_unit.getRollPitchYaw()
    return (
        wrap_angle(roll - neutral_rpy[0]),
        wrap_angle(pitch - neutral_rpy[1]),
        wrap_angle(yaw - neutral_rpy[2]),
    )


def update_hud(
    supervisor,
    waves_enabled,
    athlete,
    command_pose,
    athlete_pose,
    measured_pose=None,
):
    if not supervisor.supervisor:
        return
    wave_state = "WAVES ON" if waves_enabled else "WAVES OFF"
    lines = [
        f"{wave_state}  |  buoyancy on",
        f"CoG [{athlete.cog_offset[0]:+.2f}, {athlete.cog_offset[1]:+.2f}, "
        f"{athlete.cog_offset[2]:+.2f}] m",
        f"Cmd deck: {format_pose_degrees(command_pose)}",
    ]
    if measured_pose is not None:
        lines.append(f"Meas deck: {format_pose_degrees(measured_pose)}")
        surge_err = (measured_pose[0] - command_pose[0]) * 1000.0
        sway_err = (measured_pose[1] - command_pose[1]) * 1000.0
        heave_err = (measured_pose[2] - command_pose[2]) * 1000.0
        roll_err = math.degrees(wrap_angle(measured_pose[3] - command_pose[3]))
        pitch_err = math.degrees(wrap_angle(measured_pose[4] - command_pose[4]))
        yaw_err = math.degrees(wrap_angle(measured_pose[5] - command_pose[5]))
        lines.append(
            f"Track err: Δxyz=({surge_err:+.0f},{sway_err:+.0f},{heave_err:+.0f})mm "
            f"rpy=({roll_err:+.1f},{pitch_err:+.1f},{yaw_err:+.1f})°"
        )
    supervisor.setLabel(
        0,
        "\n".join(lines),
        0.01,
        0.95,
        0.12,
        0xFFFFFF,
        0.35,
    )


class StewartInverseKinematics:
    """Maps a platform pose delta to the six piston slider positions."""

    def __init__(self):
        self.neutral_rotation = axis_angle_to_matrix(*PLATFORM_NEUTRAL_ROTATION)

    def compute(self, surge, sway, heave, roll, pitch, yaw):
        rotation = mat_mul(euler_rpy(roll, pitch, yaw), self.neutral_rotation)
        center = [
            PLATFORM_NEUTRAL_TRANSLATION[i] + [surge, sway, heave][i]
            for i in range(3)
        ]

        positions = []
        for index in range(NUM_PISTONS):
            anchor_world = mat_vec(rotation, PLATFORM_LOCAL_ANCHORS[index])
            anchor_world = [anchor_world[i] + center[i] for i in range(3)]
            delta = [anchor_world[i] - BASE_ANCHORS[index][i] for i in range(3)]
            leg_length = math.sqrt(sum(component * component for component in delta))
            positions.append(leg_length - NEUTRAL_LEG_LENGTH)

        return [clamp(position, PISTON_MIN, PISTON_MAX) for position in positions]


class SeaWaveMotion:
    """Generates six-DOF sea motion as pose deltas relative to the neutral platform."""

    def __init__(self, sea_state):
        self.sea_state = sea_state

    def pose_at(self, time):
        return (
            wave_sum(self.sea_state["surge"], time),
            wave_sum(self.sea_state["sway"], time),
            wave_sum(self.sea_state["heave"], time),
            wave_sum(self.sea_state["roll"], time),
            wave_sum(self.sea_state["pitch"], time),
            wave_sum(self.sea_state["yaw"], time),
        )


def find_pistons(robot):
    pistons = []
    for index in range(NUM_PISTONS):
        piston = robot.getDevice(f"piston{index}")
        if piston is None:
            raise RuntimeError(f"Device 'piston{index}' not found.")
        pistons.append(piston)
    return pistons


def read_force_vector(sensor):
    values = sensor.getValues()
    return [values[0], values[1], values[2]]


def find_athlete_force_sensor(robot, time_step):
    sensor = robot.getDevice("athlete_ft_sensor")
    if sensor is None:
        raise RuntimeError("Device 'athlete_ft_sensor' not found.")
    if sensor.getType() != TouchSensor.FORCE3D:
        raise RuntimeError("Device 'athlete_ft_sensor' must be a force-3d TouchSensor.")
    sensor.enable(time_step)
    return sensor


def find_deck_inertial_unit(robot, time_step):
    inertial_unit = robot.getDevice("deck_inertial_unit")
    if inertial_unit is None:
        raise RuntimeError("Device 'deck_inertial_unit' not found.")
    inertial_unit.enable(time_step)
    return inertial_unit


def find_deck_gps(robot, time_step):
    gps = robot.getDevice("deck_gps")
    if gps is None:
        raise RuntimeError("Device 'deck_gps' not found.")
    gps.enable(time_step)
    return gps


def count_saturated_pistons(piston_positions):
    margin = 1e-3
    return sum(
        1
        for position in piston_positions
        if position <= PISTON_MIN + margin or position >= PISTON_MAX - margin
    )


def main():
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())
    if time_step <= 0:
        time_step = TIME_STEP
    dt = time_step / 1000.0

    robot.getKeyboard().enable(time_step)
    pistons = find_pistons(robot)
    athlete_ft = find_athlete_force_sensor(robot, time_step)
    deck_imu = find_deck_inertial_unit(robot, time_step)
    deck_gps = find_deck_gps(robot, time_step)
    kinematics = StewartInverseKinematics()
    sea = SeaWaveMotion(SEA_STATE)
    admittance = AdmittanceControl(TRANSLATION_ADMITTANCE, ROTATION_ADMITTANCE)
    buoyancy = BuoyancyDynamics(TRACK_STIFFNESS, BUOYANCY_STIFFNESS, HYDRO_DAMPING)
    athlete = AthleteKeyboardInput(
        robot.getKeyboard(),
        mass=ATHLETE_MASS,
        neutral_cog_z=NEUTRAL_COG_Z,
        cog_rate=COG_RATE,
        cog_limits=COG_LIMITS,
        moment_rate=MOMENT_RATE,
        moment_limit=MOMENT_LIMIT,
    )
    plotter = try_create_plotter(
        history_seconds=PLOT_HISTORY_SECONDS,
        enabled=ENABLE_LIVE_PLOTS,
    )

    AthleteKeyboardInput.print_controls()
    if not ENABLE_LIVE_PLOTS:
        print("Performance tip: live plots are off by default (matplotlib slows Webots).")

    waves_enabled = True
    deck_neutral_xyz = None
    imu_neutral_rpy = None
    simulation_time = 0.0
    sensor_ready_time = dt
    next_plot_time = sensor_ready_time
    next_log_time = sensor_ready_time
    try:
        while robot.step(time_step) != -1:
            newly_pressed = athlete.update(dt)
            if ord("p") in newly_pressed or ord("P") in newly_pressed:
                waves_enabled = not waves_enabled
                print(f"Waves {'enabled' if waves_enabled else 'disabled'} (buoyancy still active).")

            athlete.update_visual(robot)

            athlete_force, athlete_torque = athlete.wrench()
            wave_pose = sea.pose_at(simulation_time) if waves_enabled else (0.0,) * 6
            athlete_pose = admittance.pose_delta(athlete_force, athlete_torque)
            command_pose = buoyancy.step(dt, wave_pose, athlete_pose)
            piston_positions = kinematics.compute(*command_pose)
            for index, position in enumerate(piston_positions):
                pistons[index].setPosition(position)

            measured_pose = None
            if simulation_time >= sensor_ready_time:
                if deck_neutral_xyz is None:
                    deck_neutral_xyz = list(deck_gps.getValues())
                    imu_neutral_rpy = list(deck_imu.getRollPitchYaw())
                position_delta = read_position_delta(deck_gps, deck_neutral_xyz)
                orientation_delta = read_orientation_delta(deck_imu, imu_neutral_rpy)
                measured_pose = position_delta + orientation_delta

            update_hud(
                robot,
                waves_enabled,
                athlete,
                command_pose,
                athlete_pose,
                measured_pose,
            )

            if simulation_time >= sensor_ready_time:
                lever_arm = athlete_lever_arm(athlete.cog_offset, NEUTRAL_COG_Z)
                measured_force = read_force_vector(athlete_ft)
                measured_torque = estimate_torque(measured_force, lever_arm)

                if plotter is not None and simulation_time >= next_plot_time:
                    plotter.update(simulation_time, measured_force, measured_torque)
                    next_plot_time += PLOT_UPDATE_INTERVAL

                if simulation_time >= next_log_time:
                    saturated = count_saturated_pistons(piston_positions)
                    print(
                        f"cmd   {format_pose_degrees(command_pose)}  "
                        f"meas  {format_pose_degrees(measured_pose)}  "
                        f"pistons saturated={saturated}/6"
                    )
                    next_log_time += CONSOLE_LOG_INTERVAL

            simulation_time += dt
    finally:
        if plotter is not None:
            plotter.close()


if __name__ == "__main__":
    main()
