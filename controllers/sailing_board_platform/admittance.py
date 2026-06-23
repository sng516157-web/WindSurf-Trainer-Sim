"""Maps athlete wrench to platform pose offsets (admittance control)."""


def clamp(value, low, high):
    return max(low, min(high, value))


class AdmittanceControl:
    """Diagonal admittance: wrench -> small pose delta on top of sea motion."""

    def __init__(self, translation_gains, rotation_gains, pose_delta_limits=None):
        self.translation_gains = translation_gains
        self.rotation_gains = rotation_gains
        self.pose_delta_limits = pose_delta_limits

    def pose_delta(self, force, torque):
        delta = (
            self.translation_gains[0] * force[0],
            self.translation_gains[1] * force[1],
            self.translation_gains[2] * force[2],
            self.rotation_gains[0] * torque[0],
            self.rotation_gains[1] * torque[1],
            self.rotation_gains[2] * torque[2],
        )
        if self.pose_delta_limits is None:
            return delta
        return tuple(
            clamp(component, -limit, limit)
            for component, limit in zip(delta, self.pose_delta_limits)
        )
