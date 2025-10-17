#!/usr/bin/env python3
import sys
from pathlib import Path

# vendor/morai_udp를 import 가능하게 경로 추가
VENDOR = Path(__file__).resolve().parents[1] / "vendor"
sys.path.insert(0, str(VENDOR))                  # morai_udp 패키지
sys.path.insert(0, str(VENDOR / "morai_udp"))    # lib.* 를 top-level 로 보이게

import morai_udp as _morai_udp
sys.modules.setdefault('lib', _morai_udp)        # "lib" 이름 alias (lib.define.* 대응)

from morai_udp.network.UDP import Receiver
from morai_udp.define.CollisionData import CollisionData

import rospy
from morai_msgs.msg import CollisionData as CollisionDataMsg

def copy_into_msg(dst, src):
    d = src if isinstance(src, dict) else getattr(src, "__dict__", {})
    for k, v in d.items():
        if hasattr(dst, k):
            setattr(dst, k, v)
    return dst

def main():
    rospy.init_node("collision_data_listener", anonymous=False)
    ip   = rospy.get_param("~ip", "0.0.0.0")
    port = int(rospy.get_param("~port", 9122))              # 포트 추후 변경
    hz   = float(rospy.get_param("~rate_hz", 20.0))

    recv = Receiver(ip, port, CollisionData())
    pub  = rospy.Publisher("/morai/collision", CollisionDataMsg, queue_size=10)
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        data = recv.get_data()
        pub.publish(copy_into_msg(CollisionDataMsg(), data))
        rate.sleep()

if __name__ == "__main__":
    main()
