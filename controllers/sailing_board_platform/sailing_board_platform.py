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

Sea motion (SEA_STATE) drives waves on top of buoyancy. Athlete input (keyboard
or F/T sensor) perturbs the deck via admittance control.

Deck pose is measured with GPS (deck_gps) and InertialUnit (deck_inertial_unit).
Both are zeroed to the neutral deck pose at startup.
"""

from controller import Supervisor, TouchSensor, Keyboard
import math
import os

from admittance import AdmittanceControl
from athlete_ft_input import AthleteFTFromForce
from athlete_input import AthleteKeyboardInput
from buoyancy import BuoyancyDynamics
from force_plotter import try_create_plotter
from hardware_config import load_preset
from session_logger import SessionLogger

# --- Runtime configuration ---------------------------------------------------
PLATFORM_PRESET = "sim_full"  # "sim_full" | "desktop_hardware"
ATHLETE_INPUT_MODE = "keyboard"  # "keyboard" | "ft_sensor"

PLOT_HISTORY_SECONDS = 30.0
ENABLE_LIVE_PLOTS = True
PLOT_UPDATE_INTERVAL = 0.25
CONSOLE_LOG_INTERVAL = 1.0
ENABLE_CSV_LOGGING = True
CSV_LOG_INTERVAL = 0.05
CSV_LOG_DIR = "logs"

CFG = load_preset(PLATFORM_PRESET)
TIME_STEP = CFG["time_step"]
NUM_PISTONS = CFG["num_pistons"]
PISTON_MIN = CFG["piston_min"]
PISTON_MAX = CFG["piston_max"]
NEUTRAL_LEG_LENGTH = CFG["neutral_leg_length"]
ATHLETE_MASS = CFG["athlete_mass"]
NEUTRAL_COG_Z = CFG["neutral_cog_z"]
COG_LIMITS = CFG["cog_limits"]
COG_RATE = CFG["cog_rate"]
MOMENT_LIMIT = CFG["moment_limit"]
MOMENT_RATE = CFG["moment_rate"]
TRANSLATION_ADMITTANCE = CFG["translation_admittance"]
ROTATION_ADMITTANCE = CFG["rotation_admittance"]
TRACK_STIFFNESS = CFG["track_stiffness"]
BUOYANCY_STIFFNESS = CFG["buoyancy_stiffness"]
HYDRO_DAMPING = CFG["hydro_damping"]
BASE_ANCHORS = CFG["base_anchors"]
PLATFORM_NEUTRAL_TRANSLATION = CFG["platform_neutral_translation"]
PLATFORM_NEUTRAL_ROTATION = CFG["platform_neutral_rotation"]
PLATFORM_LOCAL_ANCHORS = CFG["platform_local_anchors"]
SEA_STATE = CFG["sea_state"]


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


def athlete_input_label(athlete):
    if hasattr(athlete, "input_label"):
        return athlete.input_label
    return "keyboard"


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
        f"{wave_state}  |  buoyancy on  |  {athlete_input_label(athlete)}",
    ]

    if ATHLETE_INPUT_MODE == "keyboard":
        lines.append(
            f"CoG [{athlete.cog_offset[0]:+.2f}, {athlete.cog_offset[1]:+.2f}, "
            f"{athlete.cog_offset[2]:+.2f}] m"
        )
    else:
        force, torque = athlete.wrench()
        lines.append(
            f"F/T [{force[0]:+.1f}, {force[1]:+.1f}, {force[2]:+.1f}] N  "
            f"τ [{torque[0]:+.1f}, {torque[1]:+.1f}, {torque[2]:+.1f}] N·m"
        )

    lines.append(f"Cmd deck: {format_pose_degrees(command_pose)}")
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


def create_athlete_input(robot, athlete_ft_sensor):
    if ATHLETE_INPUT_MODE == "ft_sensor":
        lever_arm = [0.0, 0.0, NEUTRAL_COG_Z]
        return AthleteFTFromForce(
            force_reader=lambda: read_force_vector(athlete_ft_sensor),
            lever_arm=lever_arm,
            force_deadband=CFG["ft_force_deadband"],
            torque_deadband=CFG["ft_torque_deadband"],
            filter_hz=CFG["ft_filter_hz"],
        )

    return AthleteKeyboardInput(
        robot.getKeyboard(),
        mass=ATHLETE_MASS,
        neutral_cog_z=NEUTRAL_COG_Z,
        cog_rate=COG_RATE,
        cog_limits=COG_LIMITS,
        moment_rate=MOMENT_RATE,
        moment_limit=MOMENT_LIMIT,
    )


def main():
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())
    if time_step <= 0:
        time_step = TIME_STEP
    dt = time_step / 1000.0

    keyboard = robot.getKeyboard()
    keyboard.enable(time_step)
    pistons = find_pistons(robot)
    athlete_ft_sensor = find_athlete_force_sensor(robot, time_step)
    deck_imu = find_deck_inertial_unit(robot, time_step)
    deck_gps = find_deck_gps(robot, time_step)
    kinematics = StewartInverseKinematics()
    sea = SeaWaveMotion(SEA_STATE)
    admittance = AdmittanceControl(TRANSLATION_ADMITTANCE, ROTATION_ADMITTANCE)
    buoyancy = BuoyancyDynamics(TRACK_STIFFNESS, BUOYANCY_STIFFNESS, HYDRO_DAMPING)
    athlete = create_athlete_input(robot, athlete_ft_sensor)
    plotter = try_create_plotter(
        history_seconds=PLOT_HISTORY_SECONDS,
        enabled=ENABLE_LIVE_PLOTS,
    )
    log_dir = os.path.join(os.path.dirname(__file__), CSV_LOG_DIR)
    session_logger = SessionLogger(
        log_dir=log_dir,
        enabled=ENABLE_CSV_LOGGING,
        interval_s=CSV_LOG_INTERVAL,
        prefix=f"{PLATFORM_PRESET}_{ATHLETE_INPUT_MODE}",
    )

    print(f"Platform preset: {PLATFORM_PRESET} ({CFG['description']})")
    print(f"Athlete input: {ATHLETE_INPUT_MODE}")
    if session_logger.path:
        print(f"CSV logging: {session_logger.path}")

    athlete.print_controls()
    if not ENABLE_LIVE_PLOTS:
        print("Performance tip: live plots are off (matplotlib slows Webots).")

    waves_enabled = True
    deck_neutral_xyz = None
    imu_neutral_rpy = None
    simulation_time = 0.0
    sensor_ready_time = dt
    next_plot_time = sensor_ready_time
    next_log_time = sensor_ready_time
    previous_keys = set()
    try:
        while robot.step(time_step) != -1:
            newly_pressed = set()
            if ATHLETE_INPUT_MODE == "keyboard":
                newly_pressed = athlete.update(dt)
            else:
                athlete.update(dt)
                keys_now = set()
                for _ in range(7):
                    key = keyboard.getKey()
                    if key == -1:
                        break
                    keys_now.add(key & Keyboard.KEY)
                newly_pressed = keys_now - previous_keys
                previous_keys = keys_now

            if ord("p") in newly_pressed or ord("P") in newly_pressed:
                waves_enabled = not waves_enabled
                print(f"Waves {'enabled' if waves_enabled else 'disabled'} (buoyancy still active).")

            if ATHLETE_INPUT_MODE == "keyboard":
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
                if plotter is not None and simulation_time >= next_plot_time:
                    plotter.update(simulation_time, athlete_force, athlete_torque)
                    next_plot_time += PLOT_UPDATE_INTERVAL

                saturated = count_saturated_pistons(piston_positions)
                session_logger.maybe_log(
                    simulation_time,
                    waves_enabled,
                    ATHLETE_INPUT_MODE,
                    athlete_force,
                    athlete_torque,
                    command_pose,
                    measured_pose,
                    saturated,
                )

                if simulation_time >= next_log_time:
                    print(
                        f"cmd   {format_pose_degrees(command_pose)}  "
                        f"meas  {format_pose_degrees(measured_pose)}  "
                        f"pistons saturated={saturated}/6"
                    )
                    next_log_time += CONSOLE_LOG_INTERVAL

            simulation_time += dt
    finally:
        session_logger.close()
        if plotter is not None:
            plotter.close()


if __name__ == "__main__":
    main()
