#!/usr/bin/env python3
import sys, time
from pathlib import Path

# vendor/morai_udp를 import 가능하게 경로 추가
VENDOR = Path(__file__).resolve().parents[1] / "vendor"
sys.path.insert(0, str(VENDOR))                  # morai_udp 패키지
sys.path.insert(0, str(VENDOR / "morai_udp"))    # lib.* 를 top-level 로 보이게

import morai_udp as _morai_udp
sys.modules.setdefault('lib', _morai_udp)        # "lib" 이름 alias (lib.define.* 대응)

from morai_udp.network.UDP import Sender
from morai_udp.define.EgoCtrlCmd import EgoCtrlCmd

import rospy


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def main():
    rospy.init_node("ego_ctrl_cmd_node", anonymous=False)

    dst_ip     = rospy.get_param("~dst_ip", "192.168.0.100")
    dst_port   = int(rospy.get_param("~dst_port", 9123))       # 포트: 대회 공지값으로 교체
    tx_rate_hz = float(rospy.get_param("~tx_rate_hz", 30.0))

    # 디버그/초기값(알고리즘 미연동 시 테스트용)
    accel0 = float(rospy.get_param("~accel", 0.0))             # 0.0~1.0
    brake0 = float(rospy.get_param("~brake", 0.0))             # 0.0~1.0
    steer0 = float(rospy.get_param("~steer", 0.0))             # -1.0~1.0
    log_throttle_sec = float(rospy.get_param("~log_throttle_sec", 1.0))

    try:
        tx = Sender(dst_ip, dst_port)
    except Exception as e:
        rospy.logfatal(f"[EgoCtrlCmd] cannot open UDP sender to {dst_ip}:{dst_port} — {e}")
        raise

    rospy.loginfo(f"[EgoCtrlCmd] sending to {dst_ip}:{dst_port} at {tx_rate_hz:.1f} Hz")

    cmd = EgoCtrlCmd()
    cmd.cmd_type = 1

    rate = rospy.Rate(tx_rate_hz)
    last_log = 0.0

    while not rospy.is_shutdown():
        # 테스트/초기값(알고리즘 연동 전): 파라미터로 준 값을 보냄
        accel = clamp(accel0, 0.0, 1.0)
        brake = clamp(brake0, 0.0, 1.0)
        steer = clamp(steer0, -1.0, 1.0)

        if brake > 0.0:                   # 브레이크 우선
            accel = 0.0

        cmd.accel = accel
        cmd.brake = brake
        cmd.steer = steer

        try:
            tx.send(cmd)
        except Exception as e:
            rospy.logerr_throttle(1.0, f"[EgoCtrlCmd] send error: {e}")

        now = time.time()
        if now - last_log >= log_throttle_sec:
            rospy.loginfo(f"[EgoCtrlCmd] a={accel:.2f} b={brake:.2f} s={steer:.2f}")
            last_log = now

        rate.sleep()


if __name__ == "__main__":
    main()
