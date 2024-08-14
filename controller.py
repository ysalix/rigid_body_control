import quaternion as npq


class Controller:
    def calculates_torque_from(self, angular_velocity, quaternion):
        pass


class Mortensen1968(Controller):
    def calculates_torque_from(self, angular_velocity, quaternion):
        return -angular_velocity - npq.as_vector_part(quaternion)
