"""Rolling live plots for athlete force/torque readings."""

from collections import deque

plt = None


def _get_pyplot():
    """Pick a GUI backend before pyplot loads (needed under Webots on Windows)."""
    global plt
    if plt is not None:
        return plt

    import matplotlib

    for backend in ("TkAgg", "Qt5Agg", "QtAgg", "WXAgg"):
        try:
            matplotlib.use(backend, force=True)
            break
        except (ImportError, ValueError):
            continue

    import matplotlib.pyplot as _plt

    plt = _plt
    return plt


class ForcePlotter:
    """Rolling plots of command wrench plus optional sensor readings."""

    def __init__(self, history_seconds=30.0, title="WindSurf Trainer F/T"):
        plot = _get_pyplot()
        if plot is None:
            raise RuntimeError("matplotlib is required for live plots")

        self._plt = plot
        self.history_seconds = history_seconds
        self.times = deque()
        self.forces = [deque(), deque(), deque()]
        self.torques = [deque(), deque(), deque()]
        self.sensor_forces = [deque(), deque(), deque()]

        plot.ion()
        self.figure, axes = plot.subplots(2, 1, figsize=(9, 6), sharex=True)
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
            self.torque_axes.plot([], [], label=label, color=color)[0]
            for label, color in zip(torque_labels, colors)
        ]
        self.sensor_force_lines = [
            self.force_axes.plot(
                [], [], color=color, linestyle=":", linewidth=1.2, alpha=0.55
            )[0]
            for color in colors
        ]

        self.force_axes.set_ylabel("Force [N]")
        self.torque_axes.set_ylabel("Torque [N·m]")
        self.torque_axes.set_xlabel("Time [s]")
        self.force_axes.set_title("Forces — command (solid) / sensor (dotted)")
        self.torque_axes.set_title("Torques — command (athlete + sail)")
        self.force_axes.grid(True, alpha=0.3)
        self.torque_axes.grid(True, alpha=0.3)
        self.force_axes.legend(loc="upper right")
        self.torque_axes.legend(loc="upper right")
        self.figure.tight_layout()
        try:
            self.figure.show()
        except AttributeError:
            pass
        self._plt.pause(0.001)

    def _trim(self, current_time):
        while self.times and current_time - self.times[0] > self.history_seconds:
            self.times.popleft()
            for series in self.forces + self.torques + self.sensor_forces:
                series.popleft()

    def update(self, time, force, torque, sensor_force=None):
        self.times.append(time)
        for index in range(3):
            self.forces[index].append(force[index])
            self.torques[index].append(torque[index])
            if sensor_force is not None:
                self.sensor_forces[index].append(sensor_force[index])
            elif self.sensor_forces[index]:
                self.sensor_forces[index].append(self.sensor_forces[index][-1])
            else:
                self.sensor_forces[index].append(0.0)
        self._trim(time)

        time_data = list(self.times)
        for index, line in enumerate(self.force_lines):
            line.set_data(time_data, list(self.forces[index]))
        for index, line in enumerate(self.torque_lines):
            line.set_data(time_data, list(self.torques[index]))
        for index, line in enumerate(self.sensor_force_lines):
            line.set_data(time_data, list(self.sensor_forces[index]))

        for axis in (self.force_axes, self.torque_axes):
            axis.relim()
            axis.autoscale_view()

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        self._plt.pause(0.001)

    def close(self):
        if self._plt is not None:
            self._plt.close(self.figure)


def try_create_plotter(history_seconds=30.0, enabled=True):
    if not enabled:
        print("force_plotter: live plots disabled (set ENABLE_LIVE_PLOTS = True to enable).")
        return None
    try:
        _get_pyplot()
    except ImportError:
        print("force_plotter: matplotlib not installed; console logging only.")
        return None
    try:
        return ForcePlotter(history_seconds=history_seconds)
    except Exception as error:
        print(f"force_plotter: could not open plot window ({error}).")
        return None
