"""Scaled hardware presets for bench and desktop prototypes."""

HARDWARE_SCALE = 0.15
DESKTOP_BASE_SPAN_M = 0.60

# Full-scale geometry from stewart_platform.wbt (metres).
_FULL_BASE_ANCHORS = [
    [-1.20951, 1.57453, 0.120577],
    [-1.20951, -1.57453, 0.120577],
    [-0.75883, -1.83473, 0.120577],
    [1.96834, -0.260201, 0.120577],
    [1.96834, 0.260201, 0.120577],
    [-0.75883, 1.83473, 0.120577],
]

_FULL_PLATFORM_NEUTRAL_TRANSLATION = [0.514172, 0.235049, 0.917719]

_FULL_PLATFORM_NEUTRAL_ROTATION = (
    [0.7176690794221499, -0.12654500989810902, 0.6847900794486234],
    2.65306,
)

_FULL_PLATFORM_LOCAL_ANCHORS = [
    [1.4685, 1.0452, -1.2626],
    [1.4097, 1.3792, -1.0668],
    [1.4097, 1.3792, 0.4113],
    [1.4686, 1.0452, 0.6071],
    [1.6908, -0.2154, -0.1320],
    [1.6908, -0.2154, -0.5236],
]

_FULL_SEA_STATE = {
    "heave": [(0.08, 6.0, 0.0), (0.04, 3.5, 1.2)],
    "surge": [(0.03, 7.0, 0.5)],
    "sway": [(0.025, 5.5, 2.0)],
    "roll": [(0.10, 5.0, 0.0), (0.04, 2.8, 0.8)],
    "pitch": [(0.07, 4.5, 1.5), (0.03, 8.0, 0.0)],
    "yaw": [(0.02, 9.0, 0.0)],
}


def scale_point(point, scale):
    return [component * scale for component in point]


def scale_points(points, scale):
    return [scale_point(point, scale) for point in points]


def scale_sea_state(sea_state, linear_scale, angular_scale=1.0):
    scaled = {}
    linear_axes = ("surge", "sway", "heave")
    angular_axes = ("roll", "pitch", "yaw")
    for axis, components in sea_state.items():
        factor = linear_scale if axis in linear_axes else angular_scale
        scaled[axis] = [
            (amplitude * factor, period, phase)
            for amplitude, period, phase in components
        ]
    return scaled


def scale_tuple(values, scale):
    return tuple(value * scale for value in values)


SIM_FULL = {
    "name": "sim_full",
    "description": "Full Webots Stewart platform (~3.7 m footprint)",
    "hardware_scale": 1.0,
    "time_step": 64,
    "num_pistons": 6,
    "piston_min": -0.4,
    "piston_max": 0.4,
    "neutral_leg_length": 2.8,
    "athlete_mass": 70.0,
    "neutral_cog_z": 1.50,
    "cog_limits": (0.55, 0.80, 0.30),
    "cog_rate": 0.45,
    "moment_limit": 700.0,
    "moment_rate": 220.0,
    "translation_admittance": (0.0012, 0.0012, 0.0008),
    "rotation_admittance": (0.0010, 0.0010, 0.0005),
    "track_stiffness": (10.0, 10.0, 14.0, 9.0, 9.0, 6.0),
    "buoyancy_stiffness": (5.0, 5.0, 9.0, 6.0, 6.0, 4.0),
    "hydro_damping": (1.8, 1.8, 2.5, 0.9, 0.9, 0.7),
    "base_anchors": _FULL_BASE_ANCHORS,
    "platform_neutral_translation": _FULL_PLATFORM_NEUTRAL_TRANSLATION,
    "platform_neutral_rotation": _FULL_PLATFORM_NEUTRAL_ROTATION,
    "platform_local_anchors": _FULL_PLATFORM_LOCAL_ANCHORS,
    "sea_state": _FULL_SEA_STATE,
    "ft_force_deadband": 5.0,
    "ft_torque_deadband": 2.0,
    "ft_filter_hz": 8.0,
}

_DESKTOP_SCALE = HARDWARE_SCALE

DESKTOP_HARDWARE = {
    "name": "desktop_hardware",
    "description": (
        f"Desktop prototype target (~{DESKTOP_BASE_SPAN_M:.2f} m base span, "
        f"scale={_DESKTOP_SCALE})"
    ),
    "hardware_scale": _DESKTOP_SCALE,
    "time_step": 32,
    "num_pistons": 6,
    "piston_min": -0.4 * _DESKTOP_SCALE,
    "piston_max": 0.4 * _DESKTOP_SCALE,
    "neutral_leg_length": 2.8 * _DESKTOP_SCALE,
    "athlete_mass": 70.0,
    "neutral_cog_z": 1.50 * _DESKTOP_SCALE,
    "cog_limits": tuple(limit * _DESKTOP_SCALE for limit in (0.55, 0.80, 0.30)),
    "cog_rate": 0.45 * _DESKTOP_SCALE,
    "moment_limit": 120.0,
    "moment_rate": 80.0,
    "translation_admittance": (
        0.0012 * _DESKTOP_SCALE,
        0.0012 * _DESKTOP_SCALE,
        0.0008 * _DESKTOP_SCALE,
    ),
    "rotation_admittance": (0.00012, 0.00012, 0.00006),
    "track_stiffness": (14.0, 14.0, 18.0, 12.0, 12.0, 8.0),
    "buoyancy_stiffness": (7.0, 7.0, 11.0, 8.0, 8.0, 5.0),
    "hydro_damping": (2.4, 2.4, 3.2, 1.2, 1.2, 0.9),
    "base_anchors": scale_points(_FULL_BASE_ANCHORS, _DESKTOP_SCALE),
    "platform_neutral_translation": scale_point(
        _FULL_PLATFORM_NEUTRAL_TRANSLATION, _DESKTOP_SCALE
    ),
    "platform_neutral_rotation": _FULL_PLATFORM_NEUTRAL_ROTATION,
    "platform_local_anchors": scale_points(
        _FULL_PLATFORM_LOCAL_ANCHORS, _DESKTOP_SCALE
    ),
    "sea_state": scale_sea_state(_FULL_SEA_STATE, _DESKTOP_SCALE, angular_scale=0.85),
    "ft_force_deadband": 1.5,
    "ft_torque_deadband": 0.08,
    "ft_filter_hz": 10.0,
}

PRESETS = {
    "sim_full": SIM_FULL,
    "desktop_hardware": DESKTOP_HARDWARE,
}


def load_preset(name="sim_full"):
    if name not in PRESETS:
        raise ValueError(f"Unknown preset '{name}'. Choose from {list(PRESETS)}.")
    return dict(PRESETS[name])


def desktop_anchor_table():
    """Human-readable 600 mm-class anchor coordinates for fabrication drawings."""
    preset = DESKTOP_HARDWARE
    lines = [
        f"Desktop geometry (scale={preset['hardware_scale']}, "
        f"target span≈{DESKTOP_BASE_SPAN_M} m)",
        "",
        "Base anchors (m, world frame):",
    ]
    for index, anchor in enumerate(preset["base_anchors"]):
        lines.append(
            f"  B{index}: [{anchor[0]:+.4f}, {anchor[1]:+.4f}, {anchor[2]:+.4f}]"
        )
    lines.append("")
    lines.append("Platform local anchors (m, body frame):")
    for index, anchor in enumerate(preset["platform_local_anchors"]):
        lines.append(
            f"  P{index}: [{anchor[0]:+.4f}, {anchor[1]:+.4f}, {anchor[2]:+.4f}]"
        )
    lines.append("")
    center = preset["platform_neutral_translation"]
    lines.append(
        f"Neutral platform centre: [{center[0]:+.4f}, {center[1]:+.4f}, {center[2]:+.4f}] m"
    )
    lines.append(
        f"Piston stroke: [{preset['piston_min']:+.4f}, {preset['piston_max']:+.4f}] m"
    )
    return "\n".join(lines)
