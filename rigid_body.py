import numpy as np
import quaternion as npq
from scipy.integrate import solve_ivp


class RigidBody:
    def __init__(self, inertia_tensor, angular_velocity, quaternion=None):
        self.inertia_tensor = inertia_tensor
        principal_moment_of_inertias = np.linalg.eigvals(inertia_tensor)
        assert np.all(principal_moment_of_inertias > 0)
        for i in range(3):
            a, b, c = np.roll(principal_moment_of_inertias, i)
            assert a + b >= c

        self.angular_velocity = angular_velocity
        if quaternion is not None:
            self.quaternion = quaternion
        else:
            self.quaternion = np.quaternion(1, 0, 0, 0)

    def equations_of_motion(self, t, y):
        angular_velocity = y[:3]
        quaternion = np.quaternion(*y[3:7])
        angular_acceleration = np.linalg.inv(self.inertia_tensor) @ -np.cross(
            angular_velocity, self.inertia_tensor @ angular_velocity
        )
        quaternion_derivative = (
            0.5 * quaternion * np.quaternion(0, *angular_velocity)
        )
        return np.hstack(
            [angular_acceleration, quaternion_derivative.components]
        )

    def integrate(self, t_span, t_eval=None):
        y0 = np.hstack([self.angular_velocity, self.quaternion.components])
        return solve_ivp(
            self.equations_of_motion, t_span, y0, t_eval=t_eval, method="RK45"
        )
