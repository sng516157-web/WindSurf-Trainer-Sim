"""Second-order buoyancy and hydrodynamic damping for the floating deck."""


def combine_poses(base_pose, delta_pose):
    return tuple(base + delta for base, delta in zip(base_pose, delta_pose))


class BuoyancyDynamics:
    """
    Keeps the deck floating on a calm waterline with spring-damper dynamics.

    Waves move the equilibrium pose; athlete input offsets the target. Buoyancy
    always restores toward the current equilibrium (calm surface when waves off).
    """

    def __init__(
        self,
        track_stiffness,
        buoyancy_stiffness,
        damping,
    ):
        self.track_stiffness = track_stiffness
        self.buoyancy_stiffness = buoyancy_stiffness
        self.damping = damping
        self.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def step(self, dt, equilibrium_pose, athlete_pose_delta):
        target_pose = combine_poses(equilibrium_pose, athlete_pose_delta)

        for index in range(6):
            error_to_target = target_pose[index] - self.pose[index]
            error_to_equilibrium = equilibrium_pose[index] - self.pose[index]
            acceleration = (
                self.track_stiffness[index] * error_to_target
                + self.buoyancy_stiffness[index] * error_to_equilibrium
                - self.damping[index] * self.velocity[index]
            )
            self.velocity[index] += acceleration * dt
            self.pose[index] += self.velocity[index] * dt

        return tuple(self.pose)

    def pose_tuple(self):
        return tuple(self.pose)
