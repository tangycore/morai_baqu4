#!/usr/bin/env python3
import sys, time, math
from pathlib import Path

VENDOR = Path(__file__).resolve().parents[1] / "vendor"
sys.path.insert(0, str(VENDOR))
sys.path.insert(0, str(VENDOR / "morai_udp"))

import morai_udp as _morai_udp
sys.modules.setdefault('lib', _morai_udp)

from morai_udp.network.UDP import Sender
from morai_udp.define.EgoCtrlCmd import EgoCtrlCmd

import rospy
from morai_msgs.msg import CtrlCmd

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def main():
    rospy.init_node("ego_ctrl_cmd_node", anonymous=False)

    dst_ip     = rospy.get_param("~ip", "192.168.0.100")
    dst_port   = int(rospy.get_param("~port", 9123))
    tx_rate_hz = float(rospy.get_param("~rate_hz", 30.0))
    cmd_topic = rospy.get_param("~cmd_topic", "/ctrl_cmd")
    max_steer_deg = float(rospy.get_param("~max_steer_deg", 40.0))


    try:
        tx = Sender(dst_ip, dst_port)
    except Exception as e:
        rospy.logfatal(f"[EgoCtrlCmd] cannot open UDP sender to {dst_ip}:{dst_port} — {e}")
        raise

    rospy.loginfo(f"[EgoCtrlCmd] sending to {dst_ip}:{dst_port} at {tx_rate_hz:.1f} Hz")

    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2  # 2 = AutoMode (External)
    cmd.gear = 4       # 4 = D (주행) ← UDP 규격!
    cmd.cmd_type = 1   # 1 = Throttle 제어 (accel/brake/steering)

    last_cmd = {"accel": None, "brake": None, "steer_norm": None}
    
    def _on_ctrl(m: CtrlCmd):
        # a = clamp(m.accel,  0.0, 1.0)
        # b = clamp(m.brake,  0.0, 1.0)
        
        # # ROS 메시지의 steering은 라디안
        s_rad = getattr(m, "steering", 0.0)
        
        # ========== UDP 규격대로 정규화 ==========
        # 실제 제어 값 = 원하는 degree / 최대 조향각(degree)
        s_deg = math.degrees(s_rad)  # rad → degree
        s_norm = s_deg / max_steer_deg  # 정규화
        s_norm = clamp(s_norm, -1.0, 1.0)

        # if b > 0.0:
        #     a = 0.0  # 브레이크 우선

        last_cmd["accel"], last_cmd["brake"], last_cmd["steer_norm"] = m.accel, m.brake, s_norm

    rospy.Subscriber(cmd_topic, CtrlCmd, _on_ctrl, queue_size=10)

    rate = rospy.Rate(tx_rate_hz)

    while not rospy.is_shutdown():
        a, b, s = last_cmd["accel"], last_cmd["brake"], last_cmd["steer_norm"]
        if a is None or b is None or s is None:
            rospy.logwarn_throttle(2.0, "[EgoCtrlCmd] waiting for /ctrl_cmd ...")
        else:
            cmd.ctrl_mode = 2  # AutoMode
            cmd.gear = 4       # D (주행)
            cmd.accel = a
            cmd.brake = b
            cmd.steer = s
            
            # 디버깅 로그
            rospy.loginfo_throttle(1.0, f"[UDP] accel={a:.3f}, brake={b:.3f}, steer={s:.4f}")
            
            try:
                tx.send(cmd)
            except Exception as e:
                rospy.logerr_throttle(1.0, f"[EgoCtrlCmd] send error: {e}")
                
        rate.sleep()

if __name__ == "__main__":
    main()