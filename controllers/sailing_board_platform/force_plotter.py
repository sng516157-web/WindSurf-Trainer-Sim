"""Rolling live plots for athlete force/torque readings."""

from collections import deque

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


class ForcePlotter:
    """Maintains rolling time-series plots of 3D force and estimated torque."""

    def __init__(self, history_seconds=30.0, title="Athlete F/T sensor"):
        if plt is None:
            raise RuntimeError("matplotlib is required for live plots")

        self.history_seconds = history_seconds
        self.times = deque()
        self.forces = [deque(), deque(), deque()]
        self.torques = [deque(), deque(), deque()]

        plt.ion()
        self.figure, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        manager = self.figure.canvas.manager
        if manager is not None and hasattr(manager, "set_window_title"):
            manager.set_window_title(title)
        self.force_axes = axes[0]
        self.torque_axes = axes[1]

        force_labels = ("Fx", "Fy", "Fz")
        torque_labels = ("Mx", "My", "Mz")
        colors = ("#e45756", "#54a24b", "#4c78a8")

        self.force_lines = [
            self.force_axes.plot([], [], label=label, color=color)[0]
            for label, color in zip(force_labels, colors)
        ]
        self.torque_lines = [
            self.torque_axes.plot([], [], label=label, color=color, linestyle="--")[0]
            for label, color in zip(torque_labels, colors)
        ]

        self.force_axes.set_ylabel("Force [N]")
        self.torque_axes.set_ylabel("Torque [N·m]")
        self.torque_axes.set_xlabel("Time [s]")
        self.force_axes.set_title("Forces — measured (sensor)")
        self.torque_axes.set_title("Torques — estimated from sensor (r × F)")
        self.force_axes.grid(True, alpha=0.3)
        self.torque_axes.grid(True, alpha=0.3)
        self.force_axes.legend(loc="upper right")
        self.torque_axes.legend(loc="upper right")
        self.figure.tight_layout()

    def _trim(self, current_time):
        while self.times and current_time - self.times[0] > self.history_seconds:
            self.times.popleft()
            for series in self.forces + self.torques:
                series.popleft()

    def update(self, time, force, torque):
        self.times.append(time)
        for index in range(3):
            self.forces[index].append(force[index])
            self.torques[index].append(torque[index])
        self._trim(time)

        time_data = list(self.times)
        for index, line in enumerate(self.force_lines):
            line.set_data(time_data, list(self.forces[index]))
        for index, line in enumerate(self.torque_lines):
            line.set_data(time_data, list(self.torques[index]))

        for axis in (self.force_axes, self.torque_axes):
            axis.relim()
            axis.autoscale_view()

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def close(self):
        if plt is not None:
            plt.close(self.figure)


def try_create_plotter(history_seconds=30.0, enabled=True):
    if not enabled:
        print("force_plotter: live plots disabled (set ENABLE_LIVE_PLOTS = True to enable).")
        return None
    if plt is None:
        print("force_plotter: matplotlib not installed; console logging only.")
        return None
    try:
        return ForcePlotter(history_seconds=history_seconds)
    except Exception as error:
        print(f"force_plotter: could not open plot window ({error}).")
        return None
