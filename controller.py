import quaternion as npq


class Controller:
    def calculate_torque_from(self, angular_velocity, quaternion):
        pass


class Mortensen1968(Controller):
    def __init__(self, angular_velocity_gain, quaternion_gain):
        self.angular_velocity_gain = angular_velocity_gain
        self.quaternion_gain = quaternion_gain

    def calculate_torque_from(self, angular_velocity, quaternion):
        return (
            -self.angular_velocity_gain * angular_velocity
            - self.quaternion_gain * npq.as_vector_part(quaternion)
        )
