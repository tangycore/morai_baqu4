#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Odometry

class OdomTfBroadcaster:
    def __init__(self):
        rospy.init_node('odom_tf_broadcaster', anonymous=True)
        self.parent_frame = rospy.get_param('~parent_frame', 'odom')
        self.child_frame  = rospy.get_param('~child_frame',  'base_link')

        self.br = tf.TransformBroadcaster()               # ★ 1회 생성
        rospy.Subscriber('/odom', Odometry, self.cb, queue_size=20)

        rospy.spin()

    def cb(self, msg: Odometry):
        # 위치/자세 추출
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # 타임스탬프: 메시지에 있으면 그걸, 없으면 now()
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        # 동적 TF: parent -> child (관례: odom -> base_link)
        self.br.sendTransform(
            (p.x, p.y, 0.0),          # 보통 z=0
            (q.x, q.y, q.z, q.w),
            stamp,
            self.child_frame,
            self.parent_frame
        )

if __name__ == '__main__':
    try:
        OdomTfBroadcaster()
    except rospy.ROSInterruptException:
        pass
