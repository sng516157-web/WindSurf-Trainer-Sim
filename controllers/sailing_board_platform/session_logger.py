"""CSV session logger for F/T wrench and deck pose (sim validation + hardware bring-up)."""

import csv
import os
from datetime import datetime


FIELDNAMES = [
    "time_s",
    "waves_enabled",
    "input_mode",
    "fx_N",
    "fy_N",
    "fz_N",
    "tx_Nm",
    "ty_Nm",
    "tz_Nm",
    "cmd_surge_m",
    "cmd_sway_m",
    "cmd_heave_m",
    "cmd_roll_rad",
    "cmd_pitch_rad",
    "cmd_yaw_rad",
    "meas_surge_m",
    "meas_sway_m",
    "meas_heave_m",
    "meas_roll_rad",
    "meas_pitch_rad",
    "meas_yaw_rad",
    "pistons_saturated",
]


class SessionLogger:
    """Append pose and wrench samples to CSV for offline replay and tuning."""

    def __init__(self, log_dir="logs", enabled=True, interval_s=0.05, prefix="session"):
        self.enabled = enabled
        self.interval_s = interval_s
        self._next_log_time = 0.0
        self._file = None
        self._writer = None
        self.path = None

        if not enabled:
            return

        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"{prefix}_{stamp}.csv")
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()

    def maybe_log(
        self,
        time_s,
        waves_enabled,
        input_mode,
        force,
        torque,
        command_pose,
        measured_pose,
        pistons_saturated,
    ):
        if not self.enabled or self._writer is None:
            return
        if time_s < self._next_log_time:
            return

        row = {
            "time_s": f"{time_s:.4f}",
            "waves_enabled": int(bool(waves_enabled)),
            "input_mode": input_mode,
            "fx_N": f"{force[0]:.4f}",
            "fy_N": f"{force[1]:.4f}",
            "fz_N": f"{force[2]:.4f}",
            "tx_Nm": f"{torque[0]:.4f}",
            "ty_Nm": f"{torque[1]:.4f}",
            "tz_Nm": f"{torque[2]:.4f}",
            "cmd_surge_m": f"{command_pose[0]:.6f}",
            "cmd_sway_m": f"{command_pose[1]:.6f}",
            "cmd_heave_m": f"{command_pose[2]:.6f}",
            "cmd_roll_rad": f"{command_pose[3]:.6f}",
            "cmd_pitch_rad": f"{command_pose[4]:.6f}",
            "cmd_yaw_rad": f"{command_pose[5]:.6f}",
            "pistons_saturated": pistons_saturated,
        }

        if measured_pose is None:
            for axis in ("surge", "sway", "heave", "roll", "pitch", "yaw"):
                key = (
                    f"meas_{axis}_m"
                    if axis in ("surge", "sway", "heave")
                    else f"meas_{axis}_rad"
                )
                row[key] = ""
        else:
            row["meas_surge_m"] = f"{measured_pose[0]:.6f}"
            row["meas_sway_m"] = f"{measured_pose[1]:.6f}"
            row["meas_heave_m"] = f"{measured_pose[2]:.6f}"
            row["meas_roll_rad"] = f"{measured_pose[3]:.6f}"
            row["meas_pitch_rad"] = f"{measured_pose[4]:.6f}"
            row["meas_yaw_rad"] = f"{measured_pose[5]:.6f}"

        self._writer.writerow(row)
        self._file.flush()
        self._next_log_time = time_s + self.interval_s

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
