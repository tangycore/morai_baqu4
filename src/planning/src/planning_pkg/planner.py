import bisect
import rospy
from planning_pkg.config import *
from planning_pkg.frenet import *
from planning_pkg.frenet_path import FrenetPath
from planning_pkg.polynomial import Quartic, Quintic

def generate_lateral_movement(di_0, di_1, di_2, dt_1, dt_2, tt, tt_max, dt_0_candidates):
    trajectories = []
    for dt_0 in dt_0_candidates:
        lat_traj = Quintic(di_0, di_1, di_2, dt_0, dt_1, dt_2, tt)

        t_list = [t for t in np.arange(0.0, tt, GEN_T_STEP)]
        d0_list = [lat_traj.get_position(t) for t in t_list]
        d1_list = [lat_traj.get_velocity(t) for t in t_list]
        d2_list = [lat_traj.get_acceleration(t) for t in t_list]
        dj_list = [lat_traj.get_jerk(t) for t in t_list]

        for t in np.arange(tt, tt_max + GEN_T_STEP, GEN_T_STEP):
            t_list.append(t)
            d0_list.append(d0_list[-1])
            d1_list.append(d1_list[-1])
            d2_list.append(d2_list[-1])
            dj_list.append(dj_list[-1])
        
        generated_traj = {
            't': t_list,
            'd0': d0_list,
            'd1': d1_list,
            'd2': d2_list,
            'jerk': dj_list

        }
        trajectories.append(generated_traj)
    return trajectories


def generate_longitudinal_movement_using_quartic(si_0, si_1, si_2, st_2, tt, tt_max, st_1_candidates):

    trajectories = []
    for st_1 in st_1_candidates:
        long_traj = Quartic(si_0, si_1, si_2, st_1, st_2, tt)

        t_list = [t for t in np.arange(0.0, tt, GEN_T_STEP)]
        s0_list = [long_traj.get_position(t) for t in t_list]
        s1_list = [long_traj.get_velocity(t) for t in t_list]
        s2_list = [long_traj.get_acceleration(t) for t in t_list]
        sj_list = [long_traj.get_jerk(t) for t in t_list]

        for t in np.arange(tt, tt_max + GEN_T_STEP, GEN_T_STEP):
            t_list.append(t)
            _s = s0_list[-1] + s1_list[-1] * GEN_T_STEP
            s0_list.append(_s)
            s1_list.append(s1_list[-1])
            s2_list.append(s2_list[-1])
            sj_list.append(sj_list[-1])

        generated_traj = {
            't': t_list,
            's0': s0_list,
            's1': s1_list,
            's2': s2_list,
            'jerk': sj_list

        }
        trajectories.append(generated_traj)
    return trajectories

def generate_longitudinal_movement_using_quintic(si_0, si_1, si_2, st_1, st_2, tt, tt_max, st_0_candidates):

    trajectories = []

    for st_0 in st_0_candidates:
        long_traj = Quintic(si_0, si_1, si_2, st_0, st_1, st_2, tt)

        t_list = [t for t in np.arange(0.0, tt, GEN_T_STEP)]
        s0_list = [long_traj.get_position(t) for t in t_list]
        s1_list = [long_traj.get_velocity(t) for t in t_list]
        s2_list = [long_traj.get_acceleration(t) for t in t_list]
        sj_list = [long_traj.get_jerk(t) for t in t_list]

        for t in np.arange(tt, tt_max + GEN_T_STEP, GEN_T_STEP):
            t_list.append(t)
            s0_list.append(s0_list[-1])
            s1_list.append(s1_list[-1])
            s2_list.append(s2_list[-1])
            sj_list.append(sj_list[-1])

        generated_traj = {
            't': t_list,
            's0': np.round(s0_list, 3).tolist(),
            's1': np.round(s1_list, 3).tolist(),
            's2': np.round(s2_list, 3).tolist(),
            'jerk': np.round(sj_list, 3).tolist()

        }
        trajectories.append(generated_traj)
    return trajectories

def generate_velocity_keeping_trajectories_in_frenet(lat_state, lon_state, opt_d, desired_speed):
    frenet_paths = []

    v_keep_tt_max = V_KEEP_TT_MAX
    v_keep_tt_min = V_KEEP_TT_MIN

    if desired_speed == 0:
        desired_speed_list = sorted(set(np.arange( 5 * (lon_state[1] // 5), 0, -5)) | {desired_speed})
        if lon_state[1] < 3:
            v_keep_tt_min = 0.5
            v_keep_tt_max = 3.0 
    else:
        desired_speed_list = sorted(set(np.arange(5, desired_speed + 5, 5)) | {desired_speed})
    curr_desired_speed_idx = min(bisect.bisect_left(desired_speed_list, lon_state[1]), len(desired_speed_list) - 1)
    curr_desired_speed = desired_speed_list[curr_desired_speed_idx]
    dt_0_candidates = [-2, 0]
    st_1_candidates = np.arange(curr_desired_speed + ST_1_MIN, curr_desired_speed + ST_1_MAX + ST_1_STEP, ST_1_STEP)
    for tt in np.arange(v_keep_tt_min, v_keep_tt_max + TT_STEP, TT_STEP):
        lat_traj_list = generate_lateral_movement(*lat_state, tt, v_keep_tt_max, dt_0_candidates)
        lon_traj_list = generate_longitudinal_movement_using_quartic(*lon_state, tt, v_keep_tt_max, st_1_candidates)

        for lat_traj in lat_traj_list:
            for lon_traj in lon_traj_list:
                fp = FrenetPath()

                fp.t = lat_traj['t']
                fp.d0 = lat_traj['d0']
                fp.d1 = lat_traj['d1']
                fp.d2 = lat_traj['d2']
                fp.dj = lat_traj['jerk']

                fp.s0 = lon_traj['s0']
                fp.s1 = lon_traj['s1']
                fp.s2 = lon_traj['s2']
                fp.sj = lon_traj['jerk']

                d_diff = (lat_traj['d0'][-1] - opt_d) ** 2
                lat_cost = K_J * sum(np.power(lat_traj['jerk'], 2)) + K_T * tt + K_D * d_diff
                fp.lat_cost = lat_cost

                v_diff = (lon_traj['s1'][-1] - desired_speed) ** 2
                lon_cost = K_J * sum(np.power(lon_traj['jerk'], 2)) + K_T * tt + K_S * v_diff

                fp.lon_cost = lon_cost

                fp.tot_cost = K_LAT * fp.lat_cost + K_LON * fp.lon_cost
                frenet_paths.append(fp)

    return frenet_paths

def generate_stopping_trajectories_in_frenet(lat_state, lon_state, opt_d):
    frenet_paths = []

    dt_0_candidates = np.arange(DT_0_MIN, DT_0_MAX + DT_0_STEP, DT_0_STEP)
    st_0_candidates = [STOP_POS - GAP]

    remaining_s = (STOP_POS - GAP) - lon_state[0]
    if remaining_s < 0:
        return frenet_paths

    dynamic_tt_max = STOP_TT_MAX
    dynamic_tt_min = STOP_TT_MIN
    if remaining_s <= 5:
        dynamic_tt_max = 3.0
        dynamic_tt_min = 0.5
    elif remaining_s <= 15:
        dynamic_tt_max = 6.0
        dynamic_tt_min = 3.0
    elif remaining_s <= 25:
        dynamic_tt_max = 8.0
        dynamic_tt_min = 5.0
    elif lon_state[1] >= 12:
        dynamic_tt_max = 14.0
        dynamic_tt_min = 10.0

    for tt in np.arange(dynamic_tt_min, dynamic_tt_max + TT_STEP, TT_STEP):
        lat_traj_list = generate_lateral_movement(*lat_state, tt, dynamic_tt_max, dt_0_candidates)
        lon_traj_list = generate_longitudinal_movement_using_quintic(*lon_state, tt, dynamic_tt_max, st_0_candidates)

        for lat_traj in lat_traj_list:
            for lon_traj in lon_traj_list:
                fp = FrenetPath()

                fp.t = lat_traj['t']
                fp.d0 = lat_traj['d0']
                fp.d1 = lat_traj['d1']
                fp.d2 = lat_traj['d2']
                fp.dj = lat_traj['jerk']

                fp.s0 = lon_traj['s0']
                fp.s1 = lon_traj['s1']
                fp.s2 = lon_traj['s2']
                fp.sj = lon_traj['jerk']

                d_diff = (lat_traj['d0'][-1] - opt_d)**2
                lat_cost = K_J * sum(np.power(lat_traj['jerk'], 2)) + K_T * tt + K_D * d_diff
                fp.lat_cost = lat_cost

                s_diff = (lon_traj['s0'][-1] - (STOP_POS - GAP))**2
                lon_cost = K_J * sum(np.power(lon_traj['jerk'], 2)) + K_T * tt + 50 * K_S * s_diff

                fp.lon_cost = lon_cost

                fp.tot_cost = K_LAT * fp.lat_cost + K_LON * fp.lon_cost
                frenet_paths.append(fp)

    return frenet_paths

def generate_following_trajectories_in_frenet(lat_state, lon_state, lv, opt_d):
    frenet_paths = []

    dt_0_candidates = [DT_0_MIN, 0, DT_0_MAX]
    safe_d = 5
    st_0 = max(lv.s - (safe_d + 1.5 * lv.idm.v), lv.s - 6)
    st_0_candidates = [st_0 - 5, st_0, st_0 + 5]

    for tt in np.arange(FOLLOWING_TT_MIN, FOLLOWING_TT_MAX + TT_STEP, TT_STEP):
        lat_traj_list = generate_lateral_movement(*lat_state, tt, FOLLOWING_TT_MAX, dt_0_candidates)
        lon_traj_list = generate_longitudinal_movement_using_quintic(*lon_state, tt, FOLLOWING_TT_MAX, st_0_candidates)

        for lat_traj in lat_traj_list:
            for lon_traj in lon_traj_list:
                fp = FrenetPath()

                fp.t = np.array(lat_traj['t'])
                fp.d0 = np.array(lat_traj['d0'])
                fp.d1 = np.array(lat_traj['d1'])
                fp.d2 = np.array(lat_traj['d2'])
                fp.dj = np.array(lat_traj['jerk'])

                fp.s0 = np.array(lon_traj['s0'])
                fp.s1 = np.array(lon_traj['s1'])
                fp.s2 = np.array(lon_traj['s2'])
                fp.sj = np.array(lon_traj['jerk'])

                d_diff = (lat_traj['d0'][-1] - opt_d)**2
                lat_cost = K_J * sum(np.power(lat_traj['jerk'], 2)) + K_T * tt + K_D * d_diff
                fp.lat_cost = lat_cost

                s_diff = (lon_traj['s0'][-1] - st_0)**2
                lon_cost = K_J * sum(np.power(lon_traj['jerk'], 2)) + K_T * tt + K_S * s_diff

                fp.lon_cost = lon_cost

                fp.tot_cost = K_LAT * fp.lat_cost + K_LON * fp.lon_cost
                frenet_paths.append(fp)

    return frenet_paths

def frenet_paths_to_world(frenet_paths, center_line_xlist, center_line_ylist, center_line_slist):
    for fp in frenet_paths:
        if len(center_line_xlist) == 0 or len(center_line_slist) == 0:
            continue
        center_headings = []
        for s, d in zip(fp.s0, fp.d0):
            # 세그먼트 끝단을 벗어나는 경우를 막기 위해 s 범위를 제한
            s_clamped = np.clip(s, center_line_slist[0], center_line_slist[-1])
            x, y, heading = frenet2world(s_clamped, d, center_line_xlist, center_line_ylist, center_line_slist)
            fp.xlist.append(x)
            fp.ylist.append(y)
            center_headings.append(heading)

        num_points = len(fp.xlist)
        if num_points == 0:
            continue

        fp.yawlist = []
        fp.ds = [0.0]
        cumulative_s = [0.0]
        last_heading = center_headings[-1] if center_headings else 0.0

        if num_points == 1:
            fp.yawlist.append(last_heading)
            fp.kappa = np.zeros(1, dtype=float)
            continue

        total_s = 0.0
        for i in range(num_points - 1):
            dx = fp.xlist[i + 1] - fp.xlist[i]
            dy = fp.ylist[i + 1] - fp.ylist[i]
            raw_step = np.hypot(dx, dy)
            step_for_curvature = max(raw_step, 1e-4)

            if raw_step < 1e-4:
                # fall back to the center-line heading when the step is nearly zero to avoid yaw spikes at stop
                fallback_idx = min(i + 1, len(center_headings) - 1)
                heading = center_headings[fallback_idx] if center_headings else last_heading
            else:
                heading = np.arctan2(dy, dx)

            fp.yawlist.append(heading)
            fp.ds.append(raw_step)

            total_s += step_for_curvature
            cumulative_s.append(total_s)
            last_heading = heading

        fp.yawlist.append(fp.yawlist[-1])

        s_axis = np.asarray(cumulative_s, dtype=float)
        x_array = np.asarray(fp.xlist, dtype=float)
        y_array = np.asarray(fp.ylist, dtype=float)

        if len(x_array) < 3 or s_axis[-1] < 1e-6:
            fp.kappa = np.zeros(len(x_array), dtype=float)
            continue

        dx_ds = np.gradient(x_array, s_axis, edge_order=2)
        dy_ds = np.gradient(y_array, s_axis, edge_order=2)
        d2x_ds2 = np.gradient(dx_ds, s_axis, edge_order=2)
        d2y_ds2 = np.gradient(dy_ds, s_axis, edge_order=2)

        denom = (dx_ds**2 + dy_ds**2)**1.5
        fp.kappa = np.divide(
            dx_ds * d2y_ds2 - dy_ds * d2x_ds2,
            denom,
            out=np.zeros_like(dx_ds),
            where=denom > 1e-6
        )


    return frenet_paths

def check_collision(path, obstacles):
    """Two-circle collision check - fast and simple!"""
    if not path.xlist or not path.yawlist or not obstacles:
        return False
    
    # NumPy 배열 변환
    px_array = np.array(path.xlist)
    py_array = np.array(path.ylist)
    yaw_array = np.array(path.yawlist)
    
    # 삼각함수 계산 (한 번만!)
    cos_yaw = np.cos(yaw_array)
    sin_yaw = np.sin(yaw_array)
    
    # 차량 파라미터
    wheelbase_half = WHEEL_BASE * 0.5  # 1.5m
    vehicle_radius = OVERALL_WIDTH * 0.5 + 0.3  # 0.946 + 0.3 = 1.246m
    
    # 전방/후방 원 중심
    front_x = px_array + wheelbase_half * cos_yaw
    front_y = py_array + wheelbase_half * sin_yaw
    rear_x = px_array - wheelbase_half * cos_yaw
    rear_y = py_array - wheelbase_half * sin_yaw
    
    # Early rejection을 위한 경로 시작점
    start_x = px_array[0]
    start_y = py_array[0]
    
    for obstacle in obstacles:
        obj = obstacle.get('object')
        if obj is None:
            continue
        
        ox = float(getattr(obj, 'x', 0.0))
        oy = float(getattr(obj, 'y', 0.0))
        
        # ✅ Early rejection: 너무 멀면 스킵
        dist_to_start = np.hypot(ox - start_x, oy - start_y)
        if dist_to_start > 40.0:
            continue
        
        # 장애물 반경
        obs_w = float(getattr(obj, 'width', 0.0))
        obs_h = float(getattr(obj, 'height', 0.0))
        obs_radius = max(0.5 * np.hypot(obs_w, obs_h), 0.4)
        
        # 안전 거리
        safety_dist = vehicle_radius + obs_radius
        
        # 전방/후방 원 충돌 체크
        front_dist = np.hypot(front_x - ox, front_y - oy)
        rear_dist = np.hypot(rear_x - ox, rear_y - oy)
        
        if np.any(front_dist < safety_dist) or np.any(rear_dist < safety_dist):
            return True
    
    return False

def is_forward_motion(path, eps=1e-4):

    s = np.array(path.s0, dtype=float)

    if len(s) <= 1:
        return True
    
    v = np.array(path.s1, dtype=float)
    if np.all(np.abs(v) < 0.05):   # 전체가 거의 0 속도
        return True

    if np.any(np.diff(s) < -eps):
        return False

    v = np.array(path.s1, dtype=float)

    v[(v >= 0) & (v < 0.1)] = 0.0

    if np.any(v < -eps):
        return False

    return True

def is_in_road(path, boundaries, center_line_xlist, center_line_ylist):
    frenet_boundaries = [world2frenet(0, boundery, center_line_xlist, center_line_ylist)[1] for boundery in [5.25, -5.25]]
    road_d_min, road_d_max = np.min(frenet_boundaries), np.max(frenet_boundaries)
    for d in path.d0:
        if d <= road_d_min or road_d_max <= d:
            return False
    return True

def check_valid_path(paths, obs):
    valid_paths = []
    cv, ca, ck, cb, co = 0, 0, 0, 0, 0
    for path in paths:
        acc_squared = [(a_s**2 + a_d**2) for (a_s, a_d) in zip(path.s2, path.d2)]
        if any([v > V_MAX for v in path.s1]):
            cv += 1
            continue
        elif any([acc > ACC_MAX**2 for acc in acc_squared]):
            ca += 1
            continue
        elif any([abs(kappa) > K_MAX for kappa in path.kappa]):
            ck += 1
            # rospy.logwarn(f"max kappa : {max(path.kappa)}")
            continue
        elif not is_forward_motion(path):
            cb += 1
            continue
        elif obs and check_collision(path, obs):
            co += 1
            continue
        valid_paths.append(path)
    rospy.logwarn(f"check_valid_path total: {len(paths)} \n constraint => velo: {cv}, accel: {ca}, kappa: {ck}, back: {cb}, colli: {co}")
    return valid_paths

def generate_opt_path(valid_paths):
    opt_path = min(valid_paths, key=lambda p: p.tot_cost)
    return opt_path
