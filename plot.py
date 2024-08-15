import matplotlib.pyplot as plt
import numpy as np
import quaternion as npq
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class Cuboid:
    def __init__(self, inertia_tensor):
        principal_moment_of_inertia = np.linalg.eigvals(inertia_tensor)
        dimensions = []
        for i in range(3):
            a, b, c = np.roll(principal_moment_of_inertia, i)
            if a + b - c:
                dimensions.append(1 / np.sqrt(a + b - c))
        dimensions /= np.linalg.norm(dimensions)

        self.vertices = (
            np.array(
                [
                    [-1, -1, -1],
                    [1, -1, -1],
                    [1, 1, -1],
                    [-1, 1, -1],
                    [-1, -1, 1],
                    [1, -1, 1],
                    [1, 1, 1],
                    [-1, 1, 1],
                ]
            )
            * dimensions
        )
        self.faces = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 3, 7, 4],
            [1, 2, 6, 5],
        ]
        self.face_colors = [
            "white",
            "yellow",
            "green",
            "blue",
            "orange",
            "red",
        ]

    def rotate(self, quaternion):
        rotation_matrix = npq.as_rotation_matrix(quaternion)
        return self.vertices @ rotation_matrix.T

    def polygon(self, quaternion):
        rotated_vertices = self.rotate(quaternion)
        return Poly3DCollection(
            [rotated_vertices[face] for face in self.faces],
            facecolors=self.face_colors,
            alpha=1,
            linewidths=1,
            edgecolors="black",
        )


class Plot3D:
    def __init__(self, rigid_body):
        self.quaternion_history = rigid_body.quaternion_history
        self.figure = plt.figure()
        self.axis = self.figure.add_subplot(111, projection="3d")
        self.cuboid = Cuboid(rigid_body.inertia_tensor)

    def show(self):
        def init():
            self.axis.set_xlim([-1, 1])
            self.axis.set_ylim([-1, 1])
            self.axis.set_zlim([-1, 1])

        def update(frame):
            for collection in self.axis.collections:
                collection.remove()
            self.axis.add_collection3d(
                self.cuboid.polygon(
                    npq.quaternion(*self.quaternion_history[frame])
                )
            )

        self.animation = FuncAnimation(
            self.figure,
            update,
            frames=len(self.quaternion_history),
            init_func=init,
            interval=10,
        )