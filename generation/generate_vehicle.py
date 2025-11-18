import numpy as np
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import InitialState, CustomState
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from typing import List, Optional
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
from commonroad_route_planner.reference_path import ReferencePath


def generate_random_position_and_orientation(scenario):
    """
    在场景中生成随机位置和朝向
    
    Args:
        scenario: CommonRoad场景对象
        
    Returns:
        tuple: (position, orientation) 如果成功生成，否则返回 (None, None)
    """
    lanelets = list(scenario.lanelet_network.lanelets)
    
    # 过滤出有有效距离信息的车道
    valid_lanelets = []
    for lanelet in lanelets:
        if hasattr(lanelet, 'distance') and len(lanelet.distance) > 0 and lanelet.distance[-1] > 0:
            valid_lanelets.append(lanelet)
    
    if not valid_lanelets:
        return None, None
        
    # 随机选择一个有效车道
    lanelet = valid_lanelets[np.random.randint(len(valid_lanelets))]
    
    # 生成随机距离(0 到车道总长度之间)
    random_distance = np.random.uniform(0, lanelet.distance[-1])
    
    # 获取该距离处的位置
    center_pos, right_pos, left_pos, segment_id = lanelet.interpolate_position(random_distance)
    
    # center_pos 就是车道中心线上的随机位置
    pos = center_pos
    
    # 计算朝向 - 使用车道在该点的方向
    if segment_id < len(lanelet.center_vertices) - 1:
        direction = lanelet.center_vertices[segment_id + 1] - lanelet.center_vertices[segment_id]
        orientation = np.arctan2(direction[1], direction[0])
    else:
        # 如果是最后一个段，使用前一个段的方向
        direction = lanelet.center_vertices[segment_id] - lanelet.center_vertices[segment_id - 1]
        orientation = np.arctan2(direction[1], direction[0])
        
    return pos, orientation

class scenario_with_vehicles:
    def __init__(self, scenario, random_seed: int = None, vehicles_config: dict = {"CAR": 1}):
        self.scenario = scenario
        if random_seed is not None:
            np.random.seed(random_seed)
        self.vehicles_config = vehicles_config


    def vehicle_initial_state(self):
        """
        生成车辆初始状态
        """
        pos, orientation = generate_random_position_and_orientation(self.scenario)
        velocity = np.random.uniform(1.0, 10.0)
        acceleration = np.random.uniform(0.0, 1.0)
        yaw_rate = np.random.uniform(0.0, 1.0)
        return InitialState(position=pos, orientation=orientation, velocity=velocity, time_step=0, acceleration=acceleration, yaw_rate=yaw_rate)


    def generate_vehicle(self, reference_path: Optional[ReferencePath] = None, global_trajectory: Optional[GlobalTrajectory] = None): 
        '''
        根据self.vehicles_config生成车辆并添加到self.scenario中
        
        Args:
            reference_path: 可选的参考路径(ReferencePath)，包含.reference_path与.path_orientation
            global_trajectory: 可选的全局轨迹(GlobalTrajectory)，若提供则根据其生成车辆轨迹
        '''
        vehicles = []
        
        def _build_states(path_points, orientations, velocities):
            """原始：按索引作为时间步 i -> time_step=i。"""
            generated_states: List[State] = []
            for i in range(len(path_points)):
                state = CustomState(
                    position=path_points[i],
                    orientation=orientations[i],
                    velocity=velocities[i],
                    time_step=i
                )
                generated_states.append(state)
            return generated_states

        def _build_states_time_parameterized(path_points, orientations, velocities, scenario_dt: float):
            """时间参数化：根据弧长与速度计算累计时间并映射到离散 time_step。"""
            generated_states: List[State] = []

            # 确保为 numpy 数组
            pts = np.asarray(path_points)
            v = np.asarray(velocities)

            # 计算相邻点弧长增量 ds（首点为 0）
            if len(pts) >= 2:
                diffs = np.diff(pts, axis=0)
                ds = np.hypot(diffs[:, 0], diffs[:, 1])
                ds = np.insert(ds, 0, 0.0)
            else:
                ds = np.array([0.0])

            # 段速度使用前后均值更稳健，首段回退到自身
            if len(v) >= 2:
                v_prev = np.insert(v[:-1], 0, v[0])
                v_avg = (v + v_prev) / 2.0
            else:
                v_avg = v.copy()

            # 防止除零：最低速度阈值
            v_avg = np.clip(v_avg, 0.1, None)

            # 段时间与累计时间轴（首点为 0）
            dt_seg = ds / v_avg
            t_abs = np.cumsum(dt_seg)

            # 确保时间严格单调递增用于插值（去除重复时间点）
            if len(t_abs) > 1:
                keep_mask = np.concatenate(([True], np.diff(t_abs) > 1e-9))
                t_inc = t_abs[keep_mask]
                pts_inc = pts[keep_mask]
                v_inc = v[keep_mask]
                ori_inc = np.asarray(orientations)[keep_mask]
            else:
                t_inc = t_abs
                pts_inc = pts
                v_inc = v
                ori_inc = np.asarray(orientations)

            if len(t_inc) == 0:
                return generated_states

            total_time = float(t_inc[-1])
            if total_time <= 0.0:
                # 退化情况：全部挤在同一时间，退回原始首点
                state = CustomState(
                    position=pts_inc[0],
                    orientation=ori_inc[0],
                    velocity=v_inc[0],
                    time_step=0
                )
                generated_states.append(state)
                return generated_states

            # 构造均匀时间网格并线性插值到该网格
            K = int(np.floor(total_time / float(scenario_dt)))
            t_grid = np.arange(K + 1, dtype=float) * float(scenario_dt)

            # 位置插值（分别对 x, y）
            x_interp = np.interp(t_grid, t_inc, pts_inc[:, 0])
            y_interp = np.interp(t_grid, t_inc, pts_inc[:, 1])

            # 速度插值
            v_interp = np.interp(t_grid, t_inc, v_inc)

            # 朝向插值：先解包角度再插值，最后回包
            ori_unwrapped = np.unwrap(ori_inc)
            ori_interp = np.interp(t_grid, t_inc, ori_unwrapped)

            for k in range(len(t_grid)):
                state = CustomState(
                    position=np.array([x_interp[k], y_interp[k]]),
                    orientation=float(ori_interp[k]),
                    velocity=float(v_interp[k]),
                    time_step=int(k)
                )
                generated_states.append(state)
            return generated_states

        def _attach_vehicle(generated_states: List[State], initial_state: InitialState):
            trajectory = Trajectory(initial_time_step=initial_state.time_step, state_list=generated_states)
            prediction = TrajectoryPrediction(trajectory, Rectangle(4.3, 1.8))
            vehicle = DynamicObstacle(
                obstacle_id=1000,
                obstacle_type=ObstacleType.CAR,
                obstacle_shape=Rectangle(4.3, 1.8),
                initial_state=initial_state,
                prediction=prediction
            )
            vehicles.append(vehicle)
            self.scenario.add_objects(vehicles)
            return self.scenario
        
        # 如果提供了global_trajectory，根据其生成一个轨迹车辆
        if global_trajectory is not None:
            ref_path = global_trajectory.reference_path  # (n,2)
            print("len(ref_path): ", len(ref_path))
            orientations = global_trajectory.path_orientation  # (n,)
            velocities = global_trajectory.velocity_profile  # (n,)
            generated_states = _build_states_time_parameterized(ref_path, orientations, velocities, self.scenario.dt)
            # generated_states = _build_states(ref_path, orientations, velocities)
            print("len(generated_states): ", len(generated_states))
            # 初始状态：与第一帧状态严格对齐，避免初始状态与预测轨迹不一致导致静态显示
            initial_state = generated_states[0].convert_state_to_state(InitialState())
            return _attach_vehicle(generated_states, initial_state)

        # 如果提供了reference_path，根据其生成一个轨迹车辆（使用恒定速度）
        if reference_path is not None:
            path_points = reference_path.reference_path  # (n,2)
            # print("path_points: ", path_points)
            orientations = reference_path.path_orientation  # (n,)
            constant_velocity = 10.0
            velocities = np.ones(len(path_points)) * constant_velocity
            generated_states = _build_states(path_points, orientations, velocities)
            # 初始状态：优先使用reference_path自带的initial_state，否则由第一个状态转换
            if hasattr(reference_path, 'initial_state') and reference_path.initial_state is not None:
                initial_state = reference_path.initial_state
            else:
                initial_state = generated_states[0].convert_state_to_state(InitialState())
            return _attach_vehicle(generated_states, initial_state)
        
        # 原逻辑：根据配置生成随机车辆
        # 定义所有车辆类型基础属性
        type_map = {
            "CAR":       {"type": ObstacleType.CAR,        "length": 4.3,  "width": 1.8},
            "TRUCK":     {"type": ObstacleType.TRUCK,      "length": 12.0, "width": 2.5},
            "PEDESTRIAN":{"type": ObstacleType.PEDESTRIAN, "length": 0.6,  "width": 0.6},
            "BICYCLE":   {"type": ObstacleType.BICYCLE,    "length": 1.8,  "width": 0.5}
        }
        
        config = self.vehicles_config
        vehicle_id = 1000
        
        # 根据配置生成车辆
        for vehicle_name, count in config.items():
            if count > 0 and vehicle_name in type_map:
                vehicle_info = type_map[vehicle_name]
                # 生成count个该类型的车辆
                for _ in range(count):
                    initial_state = self.vehicle_initial_state()
                    if initial_state is not None:  # 检查是否成功生成初始状态
                        vehicle = DynamicObstacle(
                            obstacle_id=vehicle_id,
                            obstacle_type=vehicle_info["type"],
                            obstacle_shape=Rectangle(vehicle_info["length"], vehicle_info["width"]),
                            initial_state=initial_state,
                            prediction=None
                        )
                        vehicles.append(vehicle)
                        vehicle_id += 1
        
        self.scenario.add_objects(vehicles)
        return self.scenario

        