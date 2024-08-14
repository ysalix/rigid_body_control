import matplotlib.pyplot as plt
import numpy as np

from rigid_body import RigidBody

time_span = (0, 30)
time_step = 0.01

rigid_body = RigidBody(
    inertia_tensor=np.diag([0.2, 0.8, 1]),
    initial_angular_velocity=[0.5, 0.5, 0.3],
)
rigid_body.integrated_over(time_span, time_step)

plt.figure(dpi=300)
plt.plot(
    rigid_body.time_history,
    rigid_body.angular_velocity_history,
)
plt.xlim(time_span)
plt.legend(["$\omega_1$", "$\omega_2$", "$\omega_3$"])

plt.figure(dpi=300)
plt.plot(
    rigid_body.time_history,
    rigid_body.quaternion_history,
)
plt.xlim(time_span)
plt.legend(["$q_1$", "$q_2$", "$q_3$", "$q_4$"])
