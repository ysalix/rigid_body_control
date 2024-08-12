import matplotlib.pyplot as plt
import numpy as np

from rigid_body import RigidBody

rigid_body = RigidBody(
    inertia_tensor=np.diag([0.3, 0.8, 1]),
    angular_velocity=[0.5, 0.5, 0.3],
)

tmax = 30
dt = 0.01
solution = rigid_body.integrate(
    t_span=(0, tmax), t_eval=np.arange(0, tmax, dt)
)


plt.figure(dpi=300)
plt.plot(solution.t, solution.y.T[:, 0:3])
plt.xlim(0, tmax)
plt.legend(["$\omega_1$", "$\omega_2$", "$\omega_3$"])

plt.figure(dpi=300)
plt.plot(solution.t, solution.y.T[:, 3:7])
plt.xlim(0, tmax)
plt.legend(["$q_1$", "$q_2$", "$q_3$", "$q_4$"])
