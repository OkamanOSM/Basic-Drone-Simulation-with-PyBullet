import pybullet as p

from config import (
    MOTOR_RADIUS,
    MOTOR_HEIGHT,
    MAX_RPM,
    THRUST_COEFF,
)


class Motor:

    def __init__(self, local_position):
        self.local_position = tuple(local_position)
        self.rpm = 0.0
        self.visual_id = None

    def create_visual(self):
        shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=MOTOR_RADIUS,
            length=MOTOR_HEIGHT,
            rgbaColor=[1, 0, 0, 1],
        )

        self.visual_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=shape,
            basePosition=self.local_position,
        )

    def set_rpm(self, rpm):
        self.rpm = max(0.0, min(float(rpm), MAX_RPM))

    def get_rpm(self):
        return self.rpm

    def thrust(self):
        return THRUST_COEFF * self.rpm**2
