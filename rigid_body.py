import numpy as np
import quaternion as npq
from scipy.integrate import solve_ivp


class RigidBody:
    def __init__(self, inertia_tensor, initial_angular_velocity, controller):
        self.inertia_tensor = inertia_tensor
        if self.is_invalid():
            raise ValueError("Inertia tensor is invalid")
        self.initial_angular_velocity = initial_angular_velocity
        self.initial_quaternion = np.quaternion(1, 0, 0, 0)
        self.controller = controller

    def is_invalid(self):
        principal_moment_of_inertia = np.linalg.eigvals(self.inertia_tensor)
        if not np.all(principal_moment_of_inertia > 0):
            return True
        for i in range(3):
            a, b, c = np.roll(principal_moment_of_inertia, i)
            if a + b < c:
                return True
        return False

    def equations_of_motion(self, time, state):
        angular_velocity = state[:3]
        quaternion = np.quaternion(*state[3:7])
        if self.controller is not None:
            torque = self.controller.calculates_torque_from(
                angular_velocity, quaternion
            )
        else:
            torque = np.array([0, 0, 0])
        angular_acceleration = np.linalg.inv(self.inertia_tensor) @ (
            torque
            - np.cross(
                angular_velocity, self.inertia_tensor @ angular_velocity
            )
        )
        quaternion_derivative = (
            0.5 * quaternion * np.quaternion(0, *angular_velocity)
        )
        return np.hstack(
            [angular_acceleration, quaternion_derivative.components]
        )

    def integrated_over(self, time_span, time_step):
        initial_state = np.hstack(
            [
                self.initial_angular_velocity,
                self.initial_quaternion.components,
            ]
        )
        solution = solve_ivp(
            fun=self.equations_of_motion,
            t_span=time_span,
            y0=initial_state,
            t_eval=np.arange(*time_span, time_step),
            method="RK45",
        )
        self.time_history = solution.t
        self.angular_velocity_history = solution.y.T[:, 0:3]
        self.quaternion_history = solution.y.T[:, 3:7]
