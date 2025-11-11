
import os, sys
sys.path.append(os.path.join(os.getcwd(), "../"))

from simulation.simulations import simulate_without_ego, simulate_with_solution, simulate_with_planner
from simulation.utility import visualize_scenario_with_trajectory, save_solution
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import CommonRoadSolutionReader, VehicleType, VehicleModel, CostFunction
from commonroad.scenario.scenario import Tag

path = os.path.abspath("")

# specify required arguments
name_scenario = "DEU_Frankfurt-34_11_I-1"
# replace with local folder path (in this case we cloned the whole repository from https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/tree/2020a_scenarios):
folder_scenarios = os.path.join(path, "interactive_scenarios")
path_scenario = os.path.join(folder_scenarios, name_scenario)
# path where solutions are stored
path_solutions = os.path.join(path, "../outputs/solutions")

# path to store output video
path_video = os.path.join(path, "../outputs/videos")

# path to store simulated scenarios
path_scenarios_simulated = os.path.join(path, "../outputs/simulated_scenarios")

# demo attributes for saving the simulated scenarios
author = 'Max Mustermann'
affiliation = 'Technical University of Munich, Germany'
source = ''
tags = {Tag.URBAN}

vehicle_type = VehicleType.FORD_ESCORT
vehicle_model = VehicleModel.KS
cost_function = CostFunction.TR1

# run simulation, a video animation of the simulation is stored in the end
scenario_without_ego, pps = simulate_without_ego(interactive_scenario_path=path_scenario,
                                                 output_folder_path=path_video,
                                                 create_video=True)
# write simulated scenario to CommonRoad xml file
fw = CommonRoadFileWriter(scenario_without_ego, pps, author, affiliation, source, tags)
fw.write_to_file(os.path.join(path_scenarios_simulated, name_scenario + "_no_ego.xml"), OverwriteExistingFile.ALWAYS)
visualize_scenario_with_trajectory(scenario_without_ego, pps)