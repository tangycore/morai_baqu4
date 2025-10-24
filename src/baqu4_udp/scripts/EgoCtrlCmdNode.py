#!/usr/bin/env python3
import sys, time, math
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
from morai_msgs.msg import CtrlCmd

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def main():
    rospy.init_node("ego_ctrl_cmd_node", anonymous=False)

    dst_ip     = rospy.get_param("~ip", "192.168.0.100")
    dst_port   = int(rospy.get_param("~port", 9123))       # 포트: 대회 공지값으로 교체
    tx_rate_hz = float(rospy.get_param("~rate_hz", 30.0))
    cmd_topic = rospy.get_param("~cmd_topic", "/ctrl_cmd")
    
    # steering 스케일 변환(라디안 → 정규화[-1..1])
    steer_is_radian = bool(rospy.get_param("~steer_is_radian", True))
    max_steer_deg   = float(rospy.get_param("~max_steer_deg", 40.0))
    max_steer_rad   = math.radians(max_steer_deg)

    try:
        tx = Sender(dst_ip, dst_port)
    except Exception as e:
        rospy.logfatal(f"[EgoCtrlCmd] cannot open UDP sender to {dst_ip}:{dst_port} — {e}")
        raise

    rospy.loginfo(f"[EgoCtrlCmd] sending to {dst_ip}:{dst_port} at {tx_rate_hz:.1f} Hz")

    cmd = EgoCtrlCmd()
    cmd.cmd_type = 1                      # 규정: longi type 1번(accel, break)

    last_cmd = {"accel": None, "brake": None, "steer_norm": None}
    
    # control 출력 구독: accel, brake, steer만 사용
    def _on_ctrl(m: CtrlCmd):
        a = clamp(m.accel,  0.0, 1.0)
        b = clamp(m.brake,  0.0, 1.0)
        # 메시지의 steering은 라디안
        s_rad = getattr(m, "steering", 0.0)
        if steer_is_radian:
            s_norm = s_rad / max_steer_rad if max_steer_rad > 0 else 0.0
        else:
            s_norm = s_rad  # 이미 [-1..1]이면 그대로
        s_norm = clamp(s_norm, -1.0, 1.0)

        if b > 0.0:
            a = 0.0  # 브레이크 우선

        last_cmd["accel"], last_cmd["brake"], last_cmd["steer_norm"] = a, b, s_norm

    rospy.Subscriber(cmd_topic, CtrlCmd, _on_ctrl, queue_size=10)

    rate = rospy.Rate(tx_rate_hz)


    while not rospy.is_shutdown():
        a, b, s = last_cmd["accel"], last_cmd["brake"], last_cmd["steer_norm"]
        if a is None or b is None or s is None:
            rospy.logwarn_throttle(2.0, "[EgoCtrlCmd] waiting for /ctrl_cmd ...")
        else:
            cmd.accel = a
            cmd.brake = b
            cmd.steer = s
            try:
                tx.send(cmd)
            except Exception as e:
                rospy.logerr_throttle(1.0, f"[EgoCtrlCmd] send error: {e}")
                
        rate.sleep()


if __name__ == "__main__":
    main()
