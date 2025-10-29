#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
장애물 정보만 모니터링하는 독립 스크립트
Planning 로그와 분리되어 깔끔하게 확인 가능
"""

import rospy
from vision_msgs.msg import Detection3DArray
import numpy as np

class ObstacleMonitor:
    def __init__(self):
        rospy.init_node('obstacle_monitor', anonymous=True)
        
        self.ego_filter_radius = 2.0
        
        rospy.Subscriber("/cluster_result", Detection3DArray, self.callback)
        
        print("\n" + "="*80)
        print("🔍 OBSTACLE MONITOR - 장애물 정보 모니터링 시작")
        print("="*80 + "\n")
        
        rospy.spin()
    
    def callback(self, msg):
        """간결하고 깔끔한 출력"""
        
        print("\n" + "─"*80)
        print(f"📡 수신 시각: {rospy.Time.now().to_sec():.2f}s")
        print(f"📦 총 Detection 수: {len(msg.detections)}")
        print("─"*80)
        
        valid_obstacles = []
        filtered_obstacles = []
        
        for i, det in enumerate(msg.detections):
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            width = det.bbox.size.y
            height = det.bbox.size.x
            dist = np.hypot(x, y)
            
            if dist <= self.ego_filter_radius:
                filtered_obstacles.append({
                    'idx': i,
                    'x': x,
                    'y': y,
                    'z': z,
                    'width': width,
                    'height': height,
                    'dist': dist
                })
            else:
                valid_obstacles.append({
                    'idx': i,
                    'x': x,
                    'y': y,
                    'z': z,
                    'width': width,
                    'height': height,
                    'dist': dist
                })
        
        # 유효한 장애물 출력
        if valid_obstacles:
            print(f"\n✅ Planning에 전달될 장애물: {len(valid_obstacles)}개")
            print("┌─────┬──────────┬──────────┬──────────┬─────────┬─────────┬─────────┐")
            print("│ No. │    X     │    Y     │    Z     │  Width  │ Height  │  Dist   │")
            print("├─────┼──────────┼──────────┼──────────┼─────────┼─────────┼─────────┤")
            for obs in valid_obstacles:
                print(f"│ {obs['idx']:3d} │ {obs['x']:7.2f}m │ {obs['y']:7.2f}m │ {obs['z']:7.2f}m │ "
                      f"{obs['width']:6.2f}m │ {obs['height']:6.2f}m │ {obs['dist']:6.2f}m │")
            print("└─────┴──────────┴──────────┴──────────┴─────────┴─────────┴─────────┘")
        else:
            print("\n✅ Planning에 전달될 장애물: 0개")
        
        # 필터링된 장애물 출력
        if filtered_obstacles:
            print(f"\n❌ 필터링된 장애물 (ego vehicle): {len(filtered_obstacles)}개")
            print("┌─────┬──────────┬──────────┬─────────┐")
            print("│ No. │    X     │    Y     │  Dist   │")
            print("├─────┼──────────┼──────────┼─────────┤")
            for obs in filtered_obstacles:
                print(f"│ {obs['idx']:3d} │ {obs['x']:7.2f}m │ {obs['y']:7.2f}m │ {obs['dist']:6.2f}m │")
            print("└─────┴──────────┴──────────┴─────────┘")
        
        print("\n" + "─"*80 + "\n")

if __name__ == '__main__':
    try:
        ObstacleMonitor()
    except rospy.ROSInterruptException:
        pass