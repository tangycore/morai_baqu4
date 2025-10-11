import numpy as np

class PIDController:
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.intergral = 0.0
        self.prev_error = 0.0

    def accel_control(self, curr_v, target_v, dt):
        error = target_v - curr_v
        if error <= 5:
            self.intergral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        out_acc = self.KP * error + self.KI * self.intergral + self.KD * derivative
        self.prev_error = error

        # --- 물리 한계 스케일링 ---
        ACCEL_MAX = 3.0  # [m/s²]
        BRAKE_MAX = 5.0  # [m/s²]
        THROTTLE_OFFSET = 0.05  # 정지 마찰 극복용

        if out_acc >= 0:
            accel = np.clip(THROTTLE_OFFSET + out_acc / ACCEL_MAX, 0.0, 1.0)
            brake = 0.0
        else:
            accel = 0.0
            brake = np.clip(-out_acc / BRAKE_MAX, 0.0, 1.0)

        return accel, brake, out_acc
