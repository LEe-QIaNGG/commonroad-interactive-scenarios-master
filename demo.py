import os, sys
sys.path.append(os.path.join(os.getcwd(), "../"))

from simulation.simulations import simulate_without_ego, simulate_with_solution, simulate_with_planner
from simulation.utility import visualize_scenario_with_trajectory, save_solution
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import CommonRoadSolutionReader, VehicleType, VehicleModel, CostFunction
from commonroad.scenario.scenario import Tag

path = os.path.abspath("")

# specify required arguments
name_scenario = "CHN_TST-2_1_T-300"

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


vehicle_type = VehicleType.FORD_ESCORT
vehicle_model = VehicleModel.KS
cost_function = CostFunction.TR1

# run simulation, a video animation of the simulation is stored in the end
# scenario_without_ego, pps = simulate_without_ego(interactive_scenario_path=path_scenario,
#                                                  output_folder_path=path_video,
#                                                  create_video=True)
# # write simulated scenario to CommonRoad xml file
# fw = CommonRoadFileWriter(scenario_without_ego, pps, author, affiliation, source, tags)
# fw.write_to_file(os.path.join(path_scenarios_simulated, name_scenario + "_no_ego.xml"), OverwriteExistingFile.ALWAYS)
# visualize_scenario_with_trajectory(scenario_without_ego, pps)


scenario_with_planner, pps, ego_vehicles_planner = simulate_with_planner(interactive_scenario_path=path_scenario,
                                                                         output_folder_path=path_video,
                                                                         create_video=True)

# # write simulated scenario to CommonRoad xml file
# if scenario_with_planner:
#     # write simulated scenario to file
#     fw = CommonRoadFileWriter(scenario_with_planner, pps, author, affiliation, source, tags)
#     fw.write_to_file(os.path.join(path_scenarios_simulated, name_scenario + "_planner.xml"), OverwriteExistingFile.ALWAYS)
    
#     # save the planned trajectory to solution file
#     save_solution(scenario_with_planner, pps, ego_vehicles_planner, vehicle_type, vehicle_model, cost_function,
#                   path_solutions, overwrite=True)         


# name_solution = "solution_KS1:TR1:DEU_Cologne-63_5_I-1:2020a"
# solution = CommonRoadSolutionReader.open(os.path.join(path_solutions, name_solution + ".xml"))
# # run simulation, a video of the simulation is stored in the end
# scenario_with_solution, pps, ego_vehicles_solution = simulate_with_solution(interactive_scenario_path=path_scenario,
#                                                                             output_folder_path=path_video,
#                                                                             solution=solution,
#                                                                             create_video=True)
# # write simulated scenario to CommonRoad xml file
# if scenario_with_solution:
#     # write simulated scenario to file
#     fw = CommonRoadFileWriter(scenario_with_solution, pps, author, affiliation, source, tags)
#     fw.write_to_file(os.path.join(path_scenarios_simulated, name_scenario + "_solution.xml"), OverwriteExistingFile.ALWAYS)

# visualize_scenario_with_trajectory(scenario_without_ego, pps)
# visualize_scenario_with_trajectory(scenario_with_solution, pps, ego_vehicles_solution)                  