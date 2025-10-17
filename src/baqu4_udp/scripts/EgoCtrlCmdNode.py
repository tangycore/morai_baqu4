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
from morai_msgs.msg import CtrlCmd

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def main():
    rospy.init_node("ego_ctrl_cmd_node", anonymous=False)

    dst_ip     = rospy.get_param("~dst_ip", "192.168.0.100")
    dst_port   = int(rospy.get_param("~dst_port", 9123))       # 포트: 대회 공지값으로 교체
    tx_rate_hz = float(rospy.get_param("~tx_rate_hz", 30.0))
    cmd_topic = rospy.get_param("~cmd_topic", "/ctrl_cmd")
    
    try:
        tx = Sender(dst_ip, dst_port)
    except Exception as e:
        rospy.logfatal(f"[EgoCtrlCmd] cannot open UDP sender to {dst_ip}:{dst_port} — {e}")
        raise

    rospy.loginfo(f"[EgoCtrlCmd] sending to {dst_ip}:{dst_port} at {tx_rate_hz:.1f} Hz")

    cmd = EgoCtrlCmd()
    cmd.cmd_type = 1                      # 규정: longi type 1번(accel, break, steering)

    last_cmd = {"accel": None, "brake": None, "steering": None}
    last_ts  = 0.0

    # control 출력 구독: accel, brake, steering만 사용
    def _on_ctrl(m: CtrlCmd):
        a = clamp(getattr(m, "accel", 0.0),  0.0, 1.0)
        b = clamp(getattr(m, "brake", 0.0),  0.0, 1.0)
        s = clamp(getattr(m, "steering", 0.0), -1.0, 1.0)
        if b > 0.0:                                                               # 브레이크 우선
            a = 0.0
        last_cmd["accel"], last_cmd["brake"], last_cmd["steering"] = a, b, s
        nonlocal last_ts
        last_ts = time.time()

    rospy.Subscriber(cmd_topic, CtrlCmd, _on_ctrl, queue_size=10)

    rate = rospy.Rate(tx_rate_hz)


    while not rospy.is_shutdown():
        a, b, s = last_cmd["accel"], last_cmd["brake"], last_cmd["steering"]
        if a is not None and b is not None and s is not None:
            cmd.accel = a
            cmd.brake = b
            cmd.steering = s
            try:
                tx.send(cmd)
            except Exception as e:
                rospy.logerr_throttle(1.0, f"[EgoCtrlCmd] send error: {e}")
        else:
            # 아직 control에서 첫 메시지를 못 받았으면 아무것도 보내지 않음
            rospy.logwarn_throttle(2.0, "[EgoCtrlCmd] waiting for /ctrl_cmd ...")

        rate.sleep()


if __name__ == "__main__":
    main()
