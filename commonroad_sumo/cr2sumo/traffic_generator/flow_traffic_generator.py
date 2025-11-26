import logging

from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario

import sumolib
from commonroad_sumo.cr2sumo.map_converter.mapping import VEHICLE_TYPE_CR2SUMO
from commonroad_sumo.cr2sumo.traffic_generator.demand_calculator import (
    AbstractDemandCalculator,
    InfrastructureDemandCalculator,
    RandomDemandCalculator,
    ReferenceDemandCalculator,
)
from commonroad_sumo.cr2sumo.traffic_generator.traffic_generator import (
    AbstractTrafficGenerator,
)
from commonroad_sumo.errors import SumoTrafficGenerationError
from commonroad_sumo.sumolib.net import (
    SumoFlow,
    SumoVehicleType,
    SumoVehicleTypeDistribution,
)
from commonroad_sumo.sumolib.sumo_project import SumoFileType, SumoProject

_LOGGER = logging.getLogger(__name__)


def _create_flows_from_demand_calculator(
    scenario: Scenario,
    sumo_project: SumoProject,
    map_matching_delta: float,
    demand_calculator: AbstractDemandCalculator,
) -> bool:
    routes_definitions_file = sumo_project.create_file(SumoFileType.VEHICLE_ROUTES)

    obstacle_types = [
        ObstacleType.CAR,
        ObstacleType.TRUCK,
        ObstacleType.BUS,
        ObstacleType.MOTORCYCLE,
    ]
    for obstacle_type in obstacle_types:
        vehicle_type = VEHICLE_TYPE_CR2SUMO[obstacle_type]
        routes_definitions_file.add_node(
            SumoVehicleType.from_vehicle_type(vehicle_type)
        )

    sumo_net: sumolib.net.Net = sumolib.net.readNet(
        sumo_project.get_file_path(SumoFileType.NET)
    )

    reachability_cache = {}
    flow_idx = 0
    for (
        start_lanelet_id,
        end_lanelet_id,
    ) in demand_calculator.get_origin_destination_pairs():
        vehicle_spawn_rate = demand_calculator.get_origin_destination_spawn_rate(
            start_lanelet_id, end_lanelet_id
        )
        _LOGGER.debug(
            "Determined vehicle spawn rate of %s for O/D pair %s/%s",
            vehicle_spawn_rate,
            start_lanelet_id,
            end_lanelet_id,
        )
        if vehicle_spawn_rate == 0.0:
            continue
        start_lanelet = scenario.lanelet_network.find_lanelet_by_id(start_lanelet_id)
        if start_lanelet is None:
            raise SumoTrafficGenerationError(
                f"The demand calculator {demand_calculator} included the origin lanelet {start_lanelet_id} in its O/D matrix, but the lanelet is not part of the lanelet network. This is a bug."
            )
        start_pos_x = start_lanelet.center_vertices[0][0]
        start_pos_y = start_lanelet.center_vertices[0][1]

        possible_start_lanes = sumo_net.getNeighboringLanes(
            start_pos_x, start_pos_y, r=map_matching_delta
        )
        if len(possible_start_lanes) == 0:
            raise SumoTrafficGenerationError(
                f"Cannot convert demand: start lanelet {start_lanelet_id} could not be matched to its corresponding SUMO lane!"
            )
        possible_start_lanes = list(sorted(possible_start_lanes, key=lambda p: p[1]))

        end_lanelet = scenario.lanelet_network.find_lanelet_by_id(end_lanelet_id)
        end_pos_x = end_lanelet.center_vertices[-1][0]
        end_pos_y = end_lanelet.center_vertices[-1][1]
        possible_end_lanes = sumo_net.getNeighboringLanes(
            end_pos_x, end_pos_y, r=map_matching_delta
        )
        if len(possible_end_lanes) == 0:
            raise SumoTrafficGenerationError(
                f"Cannot convert demand: end lanelet {start_lanelet_id} could not be matched to its corresponding SUMO lane!"
            )
        possible_end_lanes = list(sorted(possible_end_lanes, key=lambda p: p[1]))

        end_lane = None
        start_lane = None
        for possible_start_lane, _ in possible_start_lanes:
            possible_start_edge = possible_start_lane.getEdge()
            if possible_start_edge not in reachability_cache:
                reachability_cache[possible_start_edge] = sumo_net.getReachable(
                    possible_start_edge
                )
            for possible_end_lane, _ in possible_end_lanes:
                possible_end_edge = possible_end_lane.getEdge()
                if possible_end_edge in reachability_cache[possible_start_edge]:
                    start_lane = possible_start_lane
                    end_lane = possible_end_lane
                    break

        if start_lane is None or end_lane is None:
            continue

        v_type_distribution_id = f"vTypeDist_Flow_{flow_idx}"
        v_type_probabilities = []
        for obstacle_type in obstacle_types:
            obstacle_type_probability = demand_calculator.get_obstacle_type_probability(
                start_lanelet_id, end_lanelet_id, obstacle_type
            )
            v_type_probabilities.append(obstacle_type_probability)

        if sum(v_type_probabilities) <= 0.0:
            # NOTE: Added for scenarios with pedestrians, because pedestrians are currently not considered!
            _LOGGER.debug(
                "Skipped O/D pair %s/%s because the obstacle type probabilities are all zero",
                start_lanelet_id,
                end_lanelet_id,
            )
            continue

        v_types = [
            VEHICLE_TYPE_CR2SUMO[obstacle_type].value
            for obstacle_type in obstacle_types
        ]
        v_type_distribution = SumoVehicleTypeDistribution(
            v_type_distribution_id,
            v_types,
            v_type_probabilities,
        )
        routes_definitions_file.add_node(v_type_distribution)

        flow = SumoFlow(
            id_=str(flow_idx),
            start_edge=start_lane.getEdge().getID(),
            end_edge=end_lane.getEdge().getID(),
            start_lane=start_lane.getIndex(),
            end_lane=end_lane.getIndex(),
            period=vehicle_spawn_rate,
            vehicle_type=v_type_distribution_id,
            depart_speed="max",
        )

        routes_definitions_file.add_node(flow)
        flow_idx += 1

    sumo_project.write()
    return True


class _FlowTrafficGenerator(AbstractTrafficGenerator):
    def __init__(self, map_matching_delta: int = 10) -> None:
        super().__init__()
        self._map_matching_delta = map_matching_delta

    def _create_flows_from_demand_calculator(
        self,
        scenario: Scenario,
        sumo_project: SumoProject,
        demand_calculator: AbstractDemandCalculator,
    ) -> bool:
        return _create_flows_from_demand_calculator(
            scenario, sumo_project, self._map_matching_delta, demand_calculator
        )


class DemandTrafficGenerator(_FlowTrafficGenerator):
    def generate_traffic(self, scenario: Scenario, sumo_project: SumoProject) -> bool:
        demand_calcualtor = ReferenceDemandCalculator(scenario)
        return self._create_flows_from_demand_calculator(
            scenario, sumo_project, demand_calcualtor
        )


class InfrastructureTrafficGenerator(_FlowTrafficGenerator):
    def generate_traffic(self, scenario: Scenario, sumo_project: SumoProject) -> bool:
        demand_calcualtor = InfrastructureDemandCalculator(scenario)
        return self._create_flows_from_demand_calculator(
            scenario, sumo_project, demand_calcualtor
        )


class RandomTrafficGenerator(_FlowTrafficGenerator):
    def __init__(self, map_matching_delta: int = 10, seed: int = 1234) -> None:
        super().__init__(map_matching_delta)
        self._seed = seed

    def generate_traffic(self, scenario: Scenario, sumo_project: SumoProject) -> bool:
        demand_calcualtor = RandomDemandCalculator(scenario, self._seed)
        return self._create_flows_from_demand_calculator(
            scenario, sumo_project, demand_calcualtor
        )
