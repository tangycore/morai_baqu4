from enum import Enum
import numpy as np

class DrivingMode(Enum):
    VELOCITY_KEEPING = 1
    STOPPING = 2
    EMERGENCY = 3
    FOLLOWING = 4

# common config
GEN_T_STEP = 0.05

# lateral final state position config
DT_0_MIN = -3.5
DT_0_MAX = 3.5
DT_0_STEP = 3.5

# longitunial final state position config
GAP = 4
STOP_POS = 400

# longitunial final state speed config
ST_1_MIN = -6
ST_1_MAX = 6
ST_1_STEP = 2

# terminal time config
V_KEEP_TT_MIN = 3
V_KEEP_TT_MAX = 6
STOP_TT_MIN = 6
STOP_TT_MAX = 10
FOLLOWING_TT_MIN = 3
FOLLOWING_TT_MAX = 6
TT_STEP = 0.5

# plot config
SHOW_LATERAL_PLOT = True
SHOW_OPT_LATERAL_PLOT = True
SHOW_LONGITUDINAL_PLOT = True
SHOW_OPT_LONGITUDINAL_PLOT = True
SHOW_ALL_FRENET_PATH = True
SHOW_OPT_PATH = True
SHOW_VALID_PATH = True

# cost config
K_J   = 0.1 # jerk config
K_T   = 100 # time cost config (얼마나 빠르게 도달 할건지)
K_D   = 150.0 # lateral cost config
K_S   = 1.0 # longitunial cost config

K_LAT = 1.0 # lateral cost config when combine lateral and longitunial traj
K_LON = 1.0  # longitunial cost config when combine lateral and longitunial traj


DESIRED_LAT_POS = 0
FINAL_DESIRED_SPEED = 20 # 현재 차량이 최고 도달 할 수 있는 속도

# 속도, 가속도, 곡률 최대값
V_MAX = 10
ACC_MAX = 7
K_MAX = 0.3

# Car config
# The unit is in meters.
OVERALL_LENGTH = 4.635
FRONT_OVERHANG = 0.845
REAR_OVERHANG = 0.79
OVERALL_WIDTH = 1.890
TRACK_WIDTH  = 1.647
WHEEL_BASE = 3.0
TR = 0.24
TW = 0.48

MAX_STEER = np.deg2rad(40)  # rad
SPEED = 1.0

BUBBLE_R = (WHEEL_BASE) / 2 # for collision check

# Local reference line segmentation (meters)
LOCAL_REF_WINDOW_M = 320.0
LOCAL_REF_BACKWARD_M = 30.0
