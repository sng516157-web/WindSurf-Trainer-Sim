"""Maps athlete wrench to platform pose offsets (admittance control)."""


class AdmittanceControl:
    """Diagonal admittance: wrench -> small pose delta on top of sea motion."""

    def __init__(self, translation_gains, rotation_gains):
        self.translation_gains = translation_gains
        self.rotation_gains = rotation_gains

    def pose_delta(self, force, torque):
        return (
            self.translation_gains[0] * force[0],
            self.translation_gains[1] * force[1],
            self.translation_gains[2] * force[2],
            self.rotation_gains[0] * torque[0],
            self.rotation_gains[1] * torque[1],
            self.rotation_gains[2] * torque[2],
        )
