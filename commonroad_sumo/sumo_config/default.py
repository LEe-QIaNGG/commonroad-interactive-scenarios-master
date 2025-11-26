"""Default configuration class for SUMO simulations"""

from abc import ABCMeta


class DefaultConfig(metaclass=ABCMeta):
    """
    Default configuration class for SUMO simulations.
    This is a compatibility class for sumocr's DefaultConfig.
    """
    
    # logging level for logging module
    logging_level = "INFO"  # select DEBUG, INFO, WARNING, ERROR, CRITICAL

    # default path under which the scenario folder with name DefaultConfig.scenario_name are located
    scenarios_path = None

    # scenario name and also folder name under which all scenario files are stored
    scenario_name = "<scenario_name>"

    ##
    # simulation
    ##
    dt = 0.1  # length of simulation step of the interface
    step_length = 0.1  # step length for SUMO simulation (alias for dt)
    delta_steps = 1  # number of sub-steps simulated in SUMO during every dt
    presimulation_steps = 30  # number of time steps before simulation with ego vehicle starts
    simulation_steps = 100  # number of simulated (and synchronized) time steps
    with_sumo_gui = False  # currently not supported, since we use libsumo
    # lateral resolution > 0 enables SUMO'S sublane model
    lateral_resolution = 1.0
    # re-compute orientation when fetching vehicles from SUMO.
    # Avoids lateral "sliding" at lane changes at computational costs
    compute_orientation = True
    # assign lanelet ids to dynamic obstacles. Activate only when required, due to significant computation time.
    add_lanelets_to_dyn_obstacles = False

    # ego vehicle
    ego_start_time: int = 0
    ego_veh_width = 1.6
    ego_veh_length = 4.3
    ego_ids = []
    n_ego_vehicles = 0

    ##
    # Plotting
    ##
    video_start = 1
    video_end = simulation_steps

    # autoscale plot limits; plot_x1,plot_x2, plot_y1, plot_y2 only works if plot_auto is False
    plot_auto = True

    # axis limits of plot
    plot_x1 = 450
    plot_x2 = 550
    plot_y1 = 65
    plot_y2 = 1100
    figsize_x = 15
    figsize_y = 15
    window_width = 150
    window_height = 200

    ##
    # Adjust Speed Limits
    ##
    # [m/s] if not None: use this speed limit instead of speed limit from CommonRoad files
    overwrite_speed_limit = 130 / 3.6
    # [m/s] default max. speed for SUMO for unrestricted sped limits
    unrestricted_max_speed_default = 120 / 3.6
    # [m] shifted waiting position at junction (equivalent to SUMO's contPos parameter)
    wait_pos_internal_junctions = -4.0
    # [m/s] default speed limit when no speed_limit is given
    unrestricted_speed_limit_default = 130 / 3.6

    ##
    # ego vehicle sync parameters
    ##
    # Distance around the ego vehicle in which other vehicles are synchronized via the commonroad interface
    field_of_view = 500
    # Time window to detect the lanelet change in seconds
    lanelet_check_time_window = int(2 / dt)
    # The absolute margin allowed between the planner position and ego position in SUMO
    protection_margin = 2.0
    # Variable can be used  to force the consistency to certain number of steps
    consistency_window = 4
    # Used to limit the sync mechanism only to move xy
    lane_change_sync = False
    # tolerance for detecting start of lane change
    lane_change_tol = 0.00

    ##
    # TRAFFIC GENERATION
    ##
    # probability that vehicles will start at the fringe of the network (edges without
    # predecessor), and end at the fringe of the network (edges without successor).
    fringe_factor: int = 1000000000
    # number of vehicle departures per second
    veh_per_second = 50
    # Interval of departure times for vehicles
    departure_interval_vehicles = None  # Will be set if needed
    # max. number of vehicles in route file
    n_vehicles_max: int = 30
    # max. number of vehicles per km
    max_veh_per_km: int = 70
    # random seed for deterministic sumo traffic generation (applies if not set to None)
    random_seed: int = 1234

    @classmethod
    def from_scenario_name(cls, scenario_name: str):
        """Initialize the config with a scenario name"""
        obj = cls()
        obj.scenario_name = scenario_name
        return obj

    @classmethod
    def from_dict(cls, param_dict: dict):
        """Initialize config from dictionary"""
        obj = cls()
        for param, value in param_dict.items():
            if hasattr(obj, param):
                setattr(obj, param, value)
        return obj

