
import pybullet as p

from config import (
    DRONE_MASS,
    BODY_LENGTH,
    BODY_WIDTH,
    BODY_HEIGHT,
    ARM_LENGTH,
    ARM_THICKNESS,
    MOTOR_RADIUS,
    MAX_RPM,
)
from motor import Motor
from mixer import QuadMixer


class Drone:

    def __init__(self, position=(0, 0, 1)):
        self.position = position
        self.body_id = None
        self.arm_ids = []
        self.motors = []
        self.mixer = QuadMixer()

        self.create_body()
        self.create_arms()
        self.create_motors()

    def create_body(self):
        half_extents = [
            BODY_LENGTH / 2,
            BODY_WIDTH / 2,
            BODY_HEIGHT / 2,
        ]

        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
        )

        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=[0.15, 0.15, 0.15, 1],
        )

        self.body_id = p.createMultiBody(
            baseMass=DRONE_MASS,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=self.position,
        )

    def create_arms(self):
        arm_x_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[
                ARM_LENGTH,
                ARM_THICKNESS,
                ARM_THICKNESS,
            ],
            rgbaColor=[0.35, 0.35, 0.35, 1],
        )

        arm_y_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[
                ARM_THICKNESS,
                ARM_LENGTH,
                ARM_THICKNESS,
            ],
            rgbaColor=[0.35, 0.35, 0.35, 1],
        )

        self.arm_ids = [
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=arm_x_shape,
            ),
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=arm_y_shape,
            ),
        ]

    def create_motors(self):
        positions = [
            ( ARM_LENGTH, 0, 0.03),
            (-ARM_LENGTH, 0, 0.03),
            (0,  ARM_LENGTH, 0.03),
            (0, -ARM_LENGTH, 0.03),
        ]

        for local_position in positions:
            motor = Motor(local_position)
            motor.create_visual()
            self.motors.append(motor)

    def set_control_input(self, throttle, roll, pitch, yaw):
        outputs = self.mixer.mix(
            throttle,
            roll,
            pitch,
            yaw,
        )

        for motor, output in zip(self.motors, outputs):
            motor.set_rpm(output * MAX_RPM)

    def update(self):
        pos, orn = p.getBasePositionAndOrientation(
            self.body_id
        )

        for arm in self.arm_ids:
            p.resetBasePositionAndOrientation(
                arm,
                pos,
                orn,
            )

        for motor in self.motors:
            local_pos = motor.local_position

            world_pos, _ = p.multiplyTransforms(
                pos,
                orn,
                local_pos,
                [0, 0, 0, 1],
            )

            p.resetBasePositionAndOrientation(
                motor.visual_id,
                world_pos,
                orn,
            )

    def apply_motor_forces(self):
        for motor in self.motors:
            force = motor.thrust()

            p.applyExternalForce(
                objectUniqueId=self.body_id,
                linkIndex=-1,
                forceObj=[0, 0, force],
                posObj=motor.local_position,
                flags=p.LINK_FRAME,
            )

    def get_position(self):
        pos, _ = p.getBasePositionAndOrientation(
            self.body_id
        )
        return pos

    def get_orientation(self):
        _, orn = p.getBasePositionAndOrientation(
            self.body_id
        )
        return orn
