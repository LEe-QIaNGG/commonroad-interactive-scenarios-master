from commonroad.planning.planning_problem import PlanningProblem


from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory

# own code base
from commonroad_global_planner.global_planner import GlobalPlanner
from commonroad_global_planner.utils.visualization import visualize_global_trajectory
from commonroad_velocity_planner.velocity_planner_interface import ImplementedPlanners
from planning_problem import generate_random_planning_problems
from commonroad.visualization.mp_renderer import MPRenderer
import matplotlib.pyplot as plt
import os
from generate_vehicle import scenario_with_vehicles
from scene import create_vehicle_animation, find_max_time_step, calculate_total_time

 


random_seed = 22 #feasible seed: 3,9,15,20，21
path_to_xml = os.path.join(os.getcwd(), 'XML_Data/CHN_TST-2_1_T-1.xml')
# ========== Load the CommonRoad Scenario Object=========== #
scenario, planning_problem_set = CommonRoadFileReader(
    f"{path_to_xml}"
).open()
# retrieve the first planning problem in the problem set
planning_problem_set = generate_random_planning_problems(scenario, num_problems=5,random_seed=random_seed)

# 获取有效的规划问题并规划
planning_problem = None
problems = list(planning_problem_set.planning_problem_dict.values())
global_trajectory = None
for idx, prob in enumerate(problems):
    try:
        global_planner: GlobalPlanner = GlobalPlanner(
            scenario=scenario,
            planning_problem=prob
        )
        candidate_trajectory: GlobalTrajectory = global_planner.plan_global_trajectory(
            use_regulatory_stop_elements=True,
            regulatory_elements_time_step=0,
            retrieve_shortest=True, 
            consider_least_lance_changes=True,
            velocity_planner=ImplementedPlanners.BangBangSTPlanner
        )
        # 检查轨迹是否有效（如长度，属性齐全）
        if candidate_trajectory is not None and hasattr(candidate_trajectory, 'reference_path') \
            and len(candidate_trajectory.reference_path) > 2:
            planning_problem = prob
            global_trajectory = candidate_trajectory
            print(f"选择到第{idx+1}个problem，轨迹长度: {len(candidate_trajectory.reference_path)}")
            rnd = MPRenderer()
            scenario.draw(rnd)
            prob.draw(rnd)
            rnd.render(show=True)
            plt.show(block=True)

            break
        else:
            print(f"第{idx+1}个problem规划失败（轨迹无效）")
    except Exception as e:
        print(f"第{idx+1}个planning_problem异常: {e}")
if planning_problem is None or global_trajectory is None:
    raise RuntimeError("所有planning_problem都规划失败")



# visualize_global_trajectory(
#     scenario=scenario,
#     planning_problem=planning_problem,
#     global_trajectory=global_trajectory,
#     save_img=True,
#     save_path="visualization/global_planner/seed_"+str(random_seed)+".png",
# )
TST= scenario_with_vehicles(scenario,random_seed=random_seed)
TST_scenario = TST.generate_vehicle(global_trajectory=global_trajectory)
max_time_step = find_max_time_step(TST_scenario)
create_vehicle_animation(TST_scenario,time_step=max_time_step,delta_time_steps=2)
print("total_time: ", calculate_total_time(global_trajectory))