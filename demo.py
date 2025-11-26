import os
import sys
import copy
import numpy as np
from pathlib import Path
from math import cos, sin

sys.path.append(os.path.join(os.getcwd(), "../"))

# Import commonroad_sumo instead of sumocr
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import CustomState
from commonroad_sumo.simulation.interactive_simulation import (
    InteractiveSumoSimulationWithMotionPlanner,
    InteractiveSumoSimulationWithMotionPlannerConfig
)
from commonroad_sumo.sumolib.sumo_project import SumoProject

path = os.path.abspath("")

# specify required arguments
name_scenario = "CHN_TST-4_1_T-1"

# replace with local folder path (in this case we cloned the whole repository from https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/tree/2020a_scenarios):
folder_scenarios = os.path.join(path, "./interactive_scenarios/")
path_scenario = os.path.join(folder_scenarios, name_scenario)
# path where solutions are stored
path_solutions = os.path.join(path, f"./outputs/solutions/{name_scenario}")

# path to store output video
path_video = os.path.join(path, f"./outputs/videos/{name_scenario}")
os.makedirs(path_video, exist_ok=True)

# path to store simulated scenarios
path_scenarios_simulated = os.path.join(path, f"./outputs/simulated_scenarios/{name_scenario}")
os.makedirs(path_scenarios_simulated, exist_ok=True)


class SimpleMotionPlanner(TrajectoryPlannerInterface):
    """
    A simple motion planner that decelerates the ego vehicle to full stop.
    This is a demo planner for testing purposes.
    """
    
    def plan(self, scenario: Scenario, planning_problem: PlanningProblem) -> Trajectory:
        """
        Plan a trajectory for the given planning problem.
        
        :param scenario: The current CommonRoad scenario
        :param planning_problem: The planning problem to solve
        :return: A trajectory for the ego vehicle
        """
        initial_state = planning_problem.initial_state
        state_list = [copy.deepcopy(initial_state)]
        
        # Simple planner: decelerate to full stop
        current_state = copy.deepcopy(initial_state)
        a = -4.0  # deceleration
        dt = scenario.dt
        
        for i in range(50):  # Plan for 50 time steps
            if current_state.velocity <= 0:
                break
                
            # Update position
            v = current_state.velocity
            x, y = current_state.position
            o = current_state.orientation
            
            next_state = CustomState(
                time_step=current_state.time_step + 1,
                position=np.array([x + v * cos(o) * dt, y + v * sin(o) * dt]),
                orientation=o,
                velocity=max(0.0, v + a * dt),
                acceleration=a,
                steering_angle=0.0,
            )
            
            state_list.append(next_state)
            current_state = next_state
        
        return Trajectory(initial_time_step=initial_state.time_step, state_list=state_list)


# Load scenario and planning problem set
scenario_file = os.path.join(path_scenario, f"{name_scenario}.cr.xml")
scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

# Create SUMO project from the interactive scenario folder
# The interactive scenario folder should contain the .sumo.cfg file
sumo_project = SumoProject.from_folder(Path(path_scenario))

# Create simulation config
simulation_config = InteractiveSumoSimulationWithMotionPlannerConfig()
simulation_config.ego_veh_width = 1.6
simulation_config.ego_veh_length = 4.3

# Create the interactive simulation
simulation = InteractiveSumoSimulationWithMotionPlanner(
    scenario=scenario,
    planning_problem_set=planning_problem_set,
    sumo_project=sumo_project,
    simulation_config=simulation_config
)

# Create motion planner
motion_planner = SimpleMotionPlanner()

# Run the simulation
print(f"Starting simulation with commonroad_sumo...")
result = simulation.run(
    motion_planner=motion_planner,
    reevaluation_interval=1,
    max_simulation_steps=500
)

print(f"Simulation completed!")

# Get the final scenario with ego vehicles
scenario_with_planner = result.get_scenario_with_ego_vehicles_as_dynamic_obstacles()

# Create video
print(f"Creating video...")
video_path = result.create_video(
    output_folder=path_video,
    follow_ego=False,
    video_file_type="mp4"
)
print(f"Video saved to: {video_path}")

# # Optionally save the scenario
# from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
# fw = CommonRoadFileWriter(scenario_with_planner, planning_problem_set)
# fw.write_to_file(
#     os.path.join(path_scenarios_simulated, name_scenario + "_planner.xml"),
#     OverwriteExistingFile.ALWAYS
# )
# print(f"Scenario saved to: {os.path.join(path_scenarios_simulated, name_scenario + '_planner.xml')}")
