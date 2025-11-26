import logging
from dataclasses import dataclass, field

from commonroad.common.util import Interval
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from typing_extensions import override

from commonroad_sumo.cr2sumo.map_converter.mapping import VEHICLE_TYPE_CR2SUMO
from commonroad_sumo.cr2sumo.traffic_generator.traffic_generator import (
    AbstractTrafficGenerator,
)
from commonroad_sumo.errors import SumoTrafficGenerationError
from commonroad_sumo.helpers import SumoTool, execute_sumo_tool
from commonroad_sumo.interface.driving_model_parameters_provider import (
    DrivingModelParametersProvider,
    StaticDrivingModelParametersProvider,
)
from commonroad_sumo.sumolib.net import SumoVehicleType, SumoVehicleTypeDistribution
from commonroad_sumo.sumolib.sumo_project import SumoFileType, SumoProject

_LOGGER = logging.getLogger(__name__)


@dataclass
class RandomTripsTrafficGeneratorConfig:
    random_seed: int = 1234

    max_veh_per_km: int = 25
    """maximum number of vehicles per km / sec."""

    fringe_factor: int = 2147483646
    """probability that vehicles will start at the fringe of the network (edges without
     predecessor), and end at the fringe of the network (edges without successor).
    """

    veh_distribution: dict[ObstacleType, float] = field(
        default_factory=lambda: {
            ObstacleType.CAR: 4,
            ObstacleType.TRUCK: 0.8,
            ObstacleType.BUS: 0.3,
            ObstacleType.BICYCLE: 0.2,
            ObstacleType.PEDESTRIAN: 0,
        }
    )

    veh_params_provider: DrivingModelParametersProvider = field(
        default_factory=StaticDrivingModelParametersProvider
    )

    departure_interval_vehicles: Interval = Interval(0, 1000)
    """The time frame for which vehiicle trips are generated."""


class RandomTripsTrafficGenerator(AbstractTrafficGenerator):
    def __init__(self, config: RandomTripsTrafficGeneratorConfig | None = None) -> None:
        super().__init__()
        self._config = config or RandomTripsTrafficGeneratorConfig()

    @override
    def generate_traffic(self, scenario: Scenario, sumo_project: SumoProject) -> bool:
        net_file_path = sumo_project.get_file_path(SumoFileType.NET)

        total_lane_length = _get_total_lanelet_length(scenario)

        # calculate period based on traffic frequency depending on map size
        period = 1 / (
            self._config.max_veh_per_km * (total_lane_length / 1000) * scenario.dt
        )
        _LOGGER.debug(
            f"Traffic frequency of {period} was determined based on the total lane length of the road network"
        )

        vehicle_trips_file_path = sumo_project.get_file_path(SumoFileType.VEHICLE_TRIPS)
        vehicle_routes_file = sumo_project.create_file(SumoFileType.VEHICLE_ROUTES)

        veh_params_provider = self._config.veh_params_provider

        vehicle_type_nodes = []
        for obstacle_type, probability in self._config.veh_distribution.items():
            if probability <= 0:
                continue

            vehicle_type_id = VEHICLE_TYPE_CR2SUMO[obstacle_type].value
            vehicle_type_node = SumoVehicleType(
                vehicle_type_id=vehicle_type_id,
                gui_shape=vehicle_type_id,
                vehicle_class=vehicle_type_id,
                probability=probability,
                length=veh_params_provider.get_shape(obstacle_type).length,
                width=veh_params_provider.get_shape(obstacle_type).width,
                acceleration=veh_params_provider.get_accel(obstacle_type),
                decceleration=veh_params_provider.get_decel(obstacle_type),
                max_speed=veh_params_provider.get_max_speed(obstacle_type),
                lc_strategic=veh_params_provider.get_generic_parameter(
                    obstacle_type, "lc_strategic"
                ),
                lc_cooperative=veh_params_provider.get_generic_parameter(
                    obstacle_type, "lc_cooperative"
                ),
                lc_speed_gain=veh_params_provider.get_generic_parameter(
                    obstacle_type, "lc_speed_gain"
                ),
                lc_impatience=veh_params_provider.get_generic_parameter(
                    obstacle_type, "lc_impatience"
                ),
                lc_sigma=veh_params_provider.get_generic_parameter(
                    obstacle_type, "lc_sigma"
                ),
                # Driver behavior parameters
                sigma=veh_params_provider.get_generic_parameter(obstacle_type, "sigma"),
                speed_dev=veh_params_provider.get_generic_parameter(
                    obstacle_type, "speed_dev"
                ),
                speed_factor=veh_params_provider.get_generic_parameter(
                    obstacle_type, "speed_factor"
                ),
                impatience=veh_params_provider.get_generic_parameter(
                    obstacle_type, "impatience"
                ),
            )
            vehicle_type_nodes.append(vehicle_type_node)

        vehicle_type_distribution_node = SumoVehicleTypeDistribution(
            id_="DEFAULT_VEHTYPE", v_types=vehicle_type_nodes
        )
        vehicle_routes_file.add_node(vehicle_type_distribution_node)
        vehicle_routes_file_path = sumo_project.get_file_path(
            SumoFileType.VEHICLE_ROUTES
        )
        sumo_project.write()

        # TODO: check result
        random_trips_vehicles_result = execute_sumo_tool(
            SumoTool.RANDOM_TRIPS,
            [
                "-n",
                str(net_file_path),
                "-o",
                str(vehicle_trips_file_path),
                "-r",
                str(vehicle_routes_file_path),
                "-b",
                str(self._config.departure_interval_vehicles.start),
                "-e",
                str(self._config.departure_interval_vehicles.end),
                "-p",
                str(period),
                "--fringe-factor",
                str(self._config.fringe_factor),
                "--seed",
                str(self._config.random_seed),
                "--validate",
                '--trip-attributes=departLane="best" departSpeed="max" departPos="random_free"',
                "--allow-fringe",
            ],
        )
        if random_trips_vehicles_result is None:
            raise SumoTrafficGenerationError(
                f"Failed to generate random trips for vehicles in scenario {scenario.scenario_id}: randomTrips.py failed. See debug logs for more information."
            )

        generate_trips_for_pedestrians = False
        if generate_trips_for_pedestrians:
            pedestrian_trips_file_path = sumo_project.get_file_path(
                SumoFileType.PEDESTRIAN_TRIPS
            )
            pedestrian_routes_file_path = sumo_project.get_file_path(
                SumoFileType.PEDESTRIAN_ROUTES
            )
            # TODO: check result
            random_trips_pedestrians_result = execute_sumo_tool(
                SumoTool.RANDOM_TRIPS,
                [
                    "-n",
                    str(net_file_path),
                    "-o",
                    str(pedestrian_trips_file_path),
                    "-r",
                    str(pedestrian_routes_file_path),
                    "-b",
                    str(self._config.departure_interval_vehicles.start),
                    "-e",
                    str(self._config.departure_interval_vehicles.end),
                    "-p",
                    str(1 - self._config.veh_distribution[ObstacleType.PEDESTRIAN]),
                    "--allow-fringe",
                    "--fringe-factor",
                    str(self._config.fringe_factor),
                    "--persontrips",
                    "--seed",
                    str(self._config.random_seed),
                    '--trip-attributes= modes="public car" departPos="base"',
                    "--allow-fringe",
                ],
            )
            if random_trips_pedestrians_result is None:
                raise SumoTrafficGenerationError(
                    f"Failed to generate random trips for pedestrians in scenario {scenario.scenario_id}: randomTrips.py failed. See debug logs for more information."
                )

        return True


def _get_total_lanelet_length(scenario: Scenario) -> int:
    total_lane_length = 0
    for lanelet in scenario.lanelet_network.lanelets:
        print(lanelet.distance)
        total_lane_length += lanelet.distance[-1]

    return total_lane_length
