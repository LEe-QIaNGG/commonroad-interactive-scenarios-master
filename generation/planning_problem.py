from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.state import InitialState, CustomState
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.common.util import Interval, AngleInterval
from commonroad.planning.goal import GoalRegion
import numpy as np
from generate_vehicle import generate_random_position_and_orientation

def generate_random_planning_problems(scenario, num_problems=5,random_seed=None):
    """
    为OSM导入的场景生成随机Planning Problem Set
    """
    planning_problem_set = PlanningProblemSet()
    if random_seed is not None:
        np.random.seed(random_seed)
    # 获取所有可用的车道
    available_lanelets = scenario.lanelet_network.lanelets
    if len(available_lanelets) < 2:
        print("警告：可用车道数量不足，无法生成规划问题")
        return planning_problem_set
    
    for i in range(num_problems):
        # 使用独立函数生成起始位置与朝向
        start_pos, start_orientation = generate_random_position_and_orientation(scenario)
        if start_pos is None or start_orientation is None:
            continue
        
        # 使用独立函数生成目标位置与朝向
        goal_pos, goal_orientation = generate_random_position_and_orientation(scenario)
        if goal_pos is None or goal_orientation is None:
            continue
        print("start_pos: ", start_pos, "goal_pos: ", goal_pos)
        # 创建初始状态
        initial_state = InitialState(
            position=start_pos,
            orientation=start_orientation,
            velocity=5.0,
            acceleration=0.0,
            time_step=0,
            yaw_rate=0.0,
            slip_angle=0.0
        )
        
        # 创建目标状态（使用Shape类型的位置）
        goal_state = CustomState(
            position=Circle(5.0, goal_pos),  # 使用圆形区域作为目标位置
            orientation=AngleInterval(0.0, 0.0),   # 目标方向范围
            velocity=Interval(0.0, 0.0),     # 目标速度范围
            time_step=Interval(100, 100)     # 目标时间步范围
        )
        
        # 创建目标区域
        goal_region = GoalRegion([goal_state])
        
        # 创建规划问题
        planning_problem = PlanningProblem(
            planning_problem_id=i,
            initial_state=initial_state,
            goal_region=goal_region
        )
        
        planning_problem_set.add_planning_problem(planning_problem)
    
    return planning_problem_set