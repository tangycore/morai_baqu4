import numpy as np
import rospy
from math import cos, sin
import matplotlib.pyplot as plt

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class PurePursuit:
    def __init__(self, wheel_base, max_steer, K_LD=0.5, min_ld=2.0):
        self.wheel_base = wheel_base
        self.max_steer = max_steer
        self.K_LD = K_LD
        self.min_ld = min_ld

    def msg_to_xylist(self, traj_msg):
        xlist = []
        ylist = []

        for pose_stamped in traj_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            xlist.append(x)
            ylist.append(y)
        
        return np.array(xlist), np.array(ylist)

    def find_lookahead_point(self, state, traj):
        ld = max(self.K_LD * state[3] * 3.6, self.min_ld)
        dx = traj[0] - state[0] 
        dy = traj[1] - state[1]
        d = np.hypot(dx, dy)
        forward_mask = dx * np.cos(state[2]) + dy * np.sin(state[2]) > 0
        if np.any(forward_mask):
            valid_d = d[forward_mask]
            valid_idx = np.where(forward_mask)[0]
            idx = valid_idx[np.argmin(abs(valid_d - ld))]
        else:
            # 전방점이 없으면 마지막 점 fallback
            idx = len(traj[0]) - 1
        
        return idx
    
    def steer_control(self, state, traj):
        traj = self.msg_to_xylist(traj)
        ld_idx = self.find_lookahead_point(state, traj)
        print(f"ld_idx: {ld_idx}, len: {len(traj[0])}")
        dx = traj[0][ld_idx] - state[0]
        dy = traj[1][ld_idx] - state[1]
        plt.cla()
        plt.plot(traj[0], traj[1])
        plt.plot(traj[0][ld_idx], traj[1][ld_idx], 'xr', label='LookAhead')
        plt.plot(state[0], state[1], 'xb', label='Ego')
        plt.axis('equal')
        plt.title('pure pursuit')
        plt.legend()
        plt.pause(0.001)
        alpha = pi_2_pi(np.arctan2(dy, dx) - state[2])
        # if abs(alpha) < np.deg2rad(1.0):  # 작은 각 무시 (직진 안정화)
        #     alpha = 0.0
        print(f"local dx: {dx}, dy: {dy}")
        print(f"alpha: {np.rad2deg(alpha)}, yaw: {np.rad2deg(state[2])}, {np.rad2deg(np.arctan2(dy, dx))}")
        delta = np.arctan2(2 * self.wheel_base * np.sin(alpha), np.hypot(dx, dy))
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        gain = np.clip(state[3] / 5.0, 0.3, 1.0)
        delta *= gain
        return delta

# class PurePursuit:
#     def __init__(self, wheel_base, max_steer, K_LD=0.5, min_ld=2.0):
#         self.wheel_base = wheel_base
#         self.max_steer = max_steer
#         self.K_LD = K_LD
#         self.min_ld = min_ld

#         self.is_look_forward_point = False
#         self.forward_point = None
#         self.lfd = min_ld

#     def find_forward_point(self, state, traj_msg):
#         vehicle_x, vehicle_y, vehicle_yaw, velocity = state
#         self.is_look_forward_point = False
#         self.forward_point = None

#         self.lfd = max(self.K_LD * velocity, self.min_ld)

#         det_t = np.array([
#             [ cos(vehicle_yaw),  sin(vehicle_yaw), -(cos(vehicle_yaw)*vehicle_x + sin(vehicle_yaw)*vehicle_y)],
#             [-sin(vehicle_yaw),  cos(vehicle_yaw),  (sin(vehicle_yaw)*vehicle_x - cos(vehicle_yaw)*vehicle_y)],
#             [ 0, 0, 1 ]
#         ])

#         last_local_point = None  # 마지막 점 저장

#         for i, pose_stamped in enumerate(traj_msg.poses):
#             path_point = pose_stamped.pose.position
#             global_point = np.array([path_point.x, path_point.y, 1])
#             local_point = det_t.dot(global_point)

#             # 항상 마지막 점 기억
#             last_local_point = local_point  

#             if local_point[0] > 0:
#                 dis = np.hypot(local_point[0], local_point[1])
#                 if dis >= self.lfd:
#                     self.forward_point = local_point
#                     self.is_look_forward_point = True
#                     break

#         # ✅ 못 찾으면 마지막 점 사용 (fallback)
#         if not self.is_look_forward_point and last_local_point is not None:
#             self.forward_point = last_local_point
#             self.is_look_forward_point = True
#             # optional: log
#             rospy.logwarn("[PurePursuit] No forward point found, using last point instead.")

#         return self.forward_point, self.is_look_forward_point

#     def steer_control(self, state, traj_msg):
#         forward_point, found = self.find_forward_point(state, traj_msg)
#         if not found:
#             return 0.0

#         local_x, local_y = forward_point[0], forward_point[1]
#         theta = np.arctan2(local_y, local_x)
#         delta = np.arctan2(2 * self.wheel_base * np.sin(theta), self.lfd)
#         delta = np.clip(delta, -self.max_steer, self.max_steer)
#         return delta
