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
        delta = np.arctan2(2 * self.wheel_base * np.sin(alpha), np.hypot(dx, dy))
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        gain = np.clip(state[3] / 5.0, 0.3, 1.0)
        delta *= gain
        return delta
