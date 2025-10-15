import numpy as np

class PIDController:
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.intergral = 0.0
        self.prev_error = 0.0
        self.integral_guard = 5.0 / 3.6  # 약 5km/h 이하 오차에서만 적분 항 누적

    def accel_control(self, curr_v, target_v, dt):
        error = target_v - curr_v
        if error <= self.integral_guard:
            self.intergral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        out_acc = self.KP * error + self.KI * self.intergral + self.KD * derivative
        self.prev_error = error

        # --- 물리 한계 스케일링 ---
        ACCEL_MAX = 6.0  # [m/s²]
        BRAKE_MAX = 8.0  # [m/s²]
        THROTTLE_OFFSET = 0.05  # 정지 마찰 극복용

        if out_acc >= 0:
            accel = np.clip(THROTTLE_OFFSET + out_acc / ACCEL_MAX, 0.0, 1.0)
            brake = 0.0
        else:
            accel = 0.0
            brake = np.clip(-out_acc / BRAKE_MAX, 0.0, 0.5)

        return accel, brake, out_acc
