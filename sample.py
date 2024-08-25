import numpy as np

from controller import Mortensen1968
from plot import Plot3D
from rigid_body import RigidBody

time_span = (0, 15)
time_step = 0.01

rigid_body = RigidBody(
    inertia_tensor=np.diag([0.3, 0.7, 1]),
    initial_angular_velocity=np.array([0.1, 5, 0]),
    controller=None,
)
plot = Plot3D(rigid_body)

results = rigid_body.integrated_over(time_span, time_step)
plot.show(results, time_step)
