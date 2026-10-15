
import time

import pybullet as p
import pybullet_data

from config import GRAVITY
from drone import Drone


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


physics_client = p.connect(p.GUI)

if physics_client < 0:
    raise RuntimeError("PyBullet GUI bağlantısı kurulamadı.")

try:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -GRAVITY)

    dt = 1.0 / 240.0
    p.setTimeStep(dt)

    p.loadURDF("plane.urdf")

    p.resetDebugVisualizerCamera(
        cameraDistance=2.5,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[0, 0, 1],
    )

    drone = Drone(position=(0, 0, 1))

    # Başlangıç gazı
    throttle = 0.50

    # Roll, pitch ve yaw komutlarının büyüklüğü
    control_strength = 0.05

    print("Klavye kontrolleri:")
    print("W / S       : İleri / geri")
    print("A / D       : Sola / sağa")
    print("Yukarı / aşağı ok : Gaz artır / azalt")
    print("Sol / sağ ok      : Yaw komutu")
    print("ESC         : Çıkış")

    while p.isConnected():
        start_time = time.perf_counter()
        keys = p.getKeyboardEvents()

        if 27 in keys and (keys[27] & p.KEY_WAS_TRIGGERED):
            break

        def is_down(key):
            return key in keys and bool(keys[key] & p.KEY_IS_DOWN)

        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        if is_down(ord("a")):
            roll = -control_strength
        if is_down(ord("d")):
            roll = control_strength

        if is_down(ord("w")):
            pitch = control_strength
        if is_down(ord("s")):
            pitch = -control_strength

        throttle_rate = 0.1  

        if is_down(p.B3G_UP_ARROW):
            throttle += throttle_rate * dt
        if is_down(p.B3G_DOWN_ARROW):
            throttle -= throttle_rate * dt

        throttle = clamp(throttle, 0.0, 1.0)

        if is_down(p.B3G_LEFT_ARROW):
            yaw = -control_strength
        if is_down(p.B3G_RIGHT_ARROW):
            yaw = control_strength

        drone.set_control_input(
            throttle=throttle,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

        drone.update()
        drone.apply_motor_forces()

        p.stepSimulation()

        elapsed = time.perf_counter() - start_time
        time.sleep(max(0.0, dt - elapsed))

finally:
    if p.isConnected():
        p.disconnect()
