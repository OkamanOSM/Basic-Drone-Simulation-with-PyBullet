class QuadMixer:

    def mix(self, throttle, roll, pitch, yaw):
        throttle = max(0.0, min(1.0, throttle))

        # Motor sırası:
        # M1: ön-sağ
        # M2: ön-sol
        # M3: arka-sol
        # M4: arka-sağ

        outputs = [
            throttle + pitch - roll - yaw,  # M1
            throttle + pitch + roll + yaw,  # M2
            throttle - pitch + roll - yaw,  # M3
            throttle - pitch - roll + yaw,  # M4
        ]

        # Basit sınırlandırma.
        return [
            max(0.0, min(1.0, output))
            for output in outputs
        ]
