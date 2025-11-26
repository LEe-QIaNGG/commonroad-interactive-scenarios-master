import random
from abc import ABC, abstractmethod
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import DefaultDict, Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LaneletType
from commonroad.scenario.obstacle import (
    DynamicObstacle,
    ObstacleType,
    TrajectoryPrediction,
)
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import TraceState

from commonroad_sumo.cr2sumo.map_converter.util import get_scenario_length_in_seconds


@dataclass
class Flow:
    start_lanelet_id: int
    end_lanelet_id: int
    dynamic_obstacle: DynamicObstacle


def _find_all_reachable_end_lanelets(
    lanelet_network: LaneletNetwork, lanelet: Lanelet
) -> Set[int]:
    """
    Recursivley search for all reachable lanelets without a successor, starting from `lanelet`.

    :param lanelet_network: The `LaneletNetwork` in which to search for the reachable end lanelets.
    :param lanelet: The origin lanelet from which a DFS will be performed.
    :returns: A set of lanelet IDs. The associated lanelets are reachable from the origin `lanelet` and are at the perimeter of the lanelet network (aka. they do not have any successors).
    """
    frontier = [lanelet.lanelet_id]
    res = set()
    visited = set()

    while len(frontier) > 0:
        current = frontier.pop(0)
        if current in visited:
            continue

        current_lanelet = lanelet_network.find_lanelet_by_id(current)
        if len(current_lanelet.successor) == 0:
            res.add(current)

        frontier.extend(current_lanelet.successor)

        if current_lanelet.adj_left_same_direction:
            frontier.append(current_lanelet.adj_left)
        if current_lanelet.adj_right_same_direction:
            frontier.append(current_lanelet.adj_right)

        visited.add(current)

    return res


def _find_outgoing_lanelet_for_state(
    state: TraceState, lanelet_network: LaneletNetwork
) -> Optional[int]:
    possible_lanelet_ids = lanelet_network.find_lanelet_by_position([state.position])[0]
    if len(possible_lanelet_ids) == 0:
        return None
    lanelet_id = lanelet_network.find_most_likely_lanelet_by_state([state])[0]
    lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
    path = _find_all_reachable_end_lanelets(lanelet_network, lanelet)

    if len(path) == 0:
        return lanelet_id

    return path.pop()


def _find_most_likely_lanelet_by_state(
    lanelet_network: LaneletNetwork, state: TraceState
) -> Optional[int]:
    # First we need to check that there is at least one lanelet which matches the state,
    # otherwise the invocation of `find_most_likely_lanelet_by_state` will fail.
    possible_lanelet_ids = lanelet_network.find_lanelet_by_position([state.position])[0]
    if len(possible_lanelet_ids) == 0:
        return None

    lanelet_id = lanelet_network.find_most_likely_lanelet_by_state([state])[0]

    return lanelet_id


def _find_incoming_lanelet_for_state(
    state: TraceState, lanelet_network: LaneletNetwork
) -> Optional[int]:
    possible_lanelet_ids = lanelet_network.find_lanelet_by_position([state.position])[0]
    if len(possible_lanelet_ids) == 0:
        return None
    lanelet_id = lanelet_network.find_most_likely_lanelet_by_state([state])[0]

    return lanelet_id


def _get_flow_of_dynamic_obstacle(
    dynamic_obstacle: DynamicObstacle, scenario: Scenario
) -> Optional[Flow]:
    if dynamic_obstacle.initial_state.time_step == 0:
        return None

    if dynamic_obstacle.prediction is None:
        return None

    if not isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
        return None

    incoming_lanelet_id = _find_incoming_lanelet_for_state(
        dynamic_obstacle.initial_state, scenario.lanelet_network
    )
    if incoming_lanelet_id is None:
        return None

    outgoing_lanelet_id = _find_outgoing_lanelet_for_state(
        dynamic_obstacle.prediction.trajectory.final_state, scenario.lanelet_network
    )
    if outgoing_lanelet_id is None:
        return None

    return Flow(
        start_lanelet_id=incoming_lanelet_id,
        end_lanelet_id=outgoing_lanelet_id,
        dynamic_obstacle=dynamic_obstacle,
    )


def extract_flows_from_scenario(scenario: Scenario) -> List[Flow]:
    flows: List[Flow] = list()
    for dynamic_obstacle in scenario.dynamic_obstacles:
        flow = _get_flow_of_dynamic_obstacle(dynamic_obstacle, scenario)
        if flow is not None:
            flows.append(flow)
    return flows


def _compute_obstacle_type_distribution(
    dynamic_obstacles: Iterable[DynamicObstacle],
) -> Dict[ObstacleType, float]:
    obstacle_type_counts = Counter(
        obstacle.obstacle_type for obstacle in dynamic_obstacles
    )
    obstacle_type_distribution = {
        obstacle_type: round(n / obstacle_type_counts.total(), 2)
        for obstacle_type, n in obstacle_type_counts.items()
    }
    return obstacle_type_distribution


def _get_default_speed_limit_for_lanelet_type(lanelet_types: Set[LaneletType]) -> float:
    """
    Determine the velocity limit based on the lanelet type.

    Parameters
    ----------
    lanelet_types: Set[LaneletType]
        Set of lanelet types.

    Returns
    -------
    velocity_limit: float
        Velocity limit in m/s.
    """
    velocity_limit = float("inf")
    for lt_type in lanelet_types:
        if lt_type in (
            LaneletType.BICYCLE_LANE,
            LaneletType.SIDEWALK,
            LaneletType.CROSSWALK,
            LaneletType.PARKING,
            LaneletType.BUS_STOP,
            LaneletType.DRIVE_WAY,
        ):
            velocity_limit = min(velocity_limit, 30 / 3.6)  # 8.33
        elif lt_type in (LaneletType.INTERSECTION, LaneletType.URBAN):
            velocity_limit = min(velocity_limit, 50 / 3.6)  # 13.89
        elif lt_type in (LaneletType.COUNTRY, LaneletType.BUS_LANE):
            velocity_limit = min(velocity_limit, 100 / 3.6)  # 27.78
        elif lt_type in (LaneletType.ACCESS_RAMP, LaneletType.EXIT_RAMP):
            velocity_limit = min(velocity_limit, 140 / 3.6)  # 38.89
        elif lt_type in (
            LaneletType.HIGHWAY,
            LaneletType.INTERSTATE,
            LaneletType.MAIN_CARRIAGE_WAY,
        ):
            velocity_limit = min(velocity_limit, 160 / 3.6)  # 44.44
        else:
            # we assume a default of 50km/h if the lanelet type is unknown
            velocity_limit = min(velocity_limit, 50 / 3.6)  # 13.89

    return velocity_limit


def _get_speed_limit_on_lanelet(
    lanelet: Lanelet, lanelet_network: LaneletNetwork
) -> float:
    speed_limit_signs = dict()
    # TODO: copied from cr-ots-interface. What is happening here? Can we remove this?
    for sign in lanelet_network.traffic_signs:
        if sign.traffic_sign_elements[0].traffic_sign_element_id.value in [
            "274",
            "274.1",
            "R2-1",
        ]:
            speed_limit_signs.update(
                {
                    sign.traffic_sign_id: float(
                        sign.traffic_sign_elements[0].additional_values[0]
                    )
                }
            )

    speed_limit = _get_default_speed_limit_for_lanelet_type(lanelet.lanelet_type)
    for sign_id in lanelet.traffic_signs:
        if sign_id in speed_limit_signs:
            speed_limit = speed_limit_signs[sign_id]

    return speed_limit


def _get_obstacle_length(dynamic_obstacle: DynamicObstacle) -> float:
    if isinstance(dynamic_obstacle.obstacle_shape, Rectangle):
        return dynamic_obstacle.obstacle_shape.length
    elif isinstance(dynamic_obstacle.obstacle_shape, Circle):
        return dynamic_obstacle.obstacle_shape.radius * 2
    else:
        raise NotImplementedError(
            f"Shape of obstacle is {type(dynamic_obstacle.obstacle_shape)}, but currently only 'Rectangle' and 'Circle' can be handled"
        )


def _compute_lanelet_capacities_from_flows(flows: Sequence[Flow], scenario: Scenario):
    total_number_of_dyanmic_obstacle_in_scenario = len(flows)
    partitioned_by_origin: Dict[int, List[DynamicObstacle]] = defaultdict(list)
    for flow in flows:
        partitioned_by_origin[flow.start_lanelet_id].append(flow.dynamic_obstacle)

    lanelet_capacities: Dict[int, float] = {}

    for start_lanelet_id, dynamic_obstacles in partitioned_by_origin.items():
        start_lanelet = scenario.lanelet_network.find_lanelet_by_id(start_lanelet_id)
        speed_limit = _get_speed_limit_on_lanelet(
            start_lanelet, scenario.lanelet_network
        )
        # TODO: handle shapes that are not a rectangle
        total_length_of_obstacles = sum(
            [
                _get_obstacle_length(dynamic_obstacle)
                for dynamic_obstacle in dynamic_obstacles
            ]
        ) / len(dynamic_obstacles)
        s_0 = 2.0  # m -- minimum distance between vehicles
        T = 1.45  # s -- desired time headway
        start_lane_capacity = (
            60 * speed_limit / (total_length_of_obstacles + (s_0 + speed_limit * T))
        )

        lanelet_capacities[start_lanelet_id] = start_lane_capacity

    lanelet_network_load_factor = total_number_of_dyanmic_obstacle_in_scenario / sum(
        lanelet_capacities.values()
    )

    load_factor_adjused_lanelet_capacities: Dict[int, float] = {}
    for lanelet_id, lanelet_capacity in lanelet_capacities.items():
        load_factor_adjused_lanelet_capacities[lanelet_id] = (
            lanelet_capacity * lanelet_network_load_factor
        )

    return load_factor_adjused_lanelet_capacities


class AbstractDemandCalculator(ABC):
    def __init__(self, scenario: Scenario) -> None:
        self._scenario = scenario

    @abstractmethod
    def get_origin_destination_spawn_rate(
        self, start_lanelet_id: int, end_lanelet_id: int
    ) -> float:
        """
        Get the number of vehicles that should be spawned per second on `lanelet_id` if they target `end_lanelet_id`.
        """
        ...

    @abstractmethod
    def get_obstacle_type_probability(
        self, start_lanelet_id: int, end_lanelet_id: int, obstacle_type: ObstacleType
    ) -> float: ...

    @abstractmethod
    def get_origin_destination_pairs(self) -> List[Tuple[int, int]]: ...


class ReferenceDemandCalculator(AbstractDemandCalculator):
    def __init__(self, scenario: Scenario) -> None:
        super().__init__(scenario)
        self._scenario_length = get_scenario_length_in_seconds(self._scenario)

        self._flows = extract_flows_from_scenario(self._scenario)
        self._origin_destination_matrix = (
            self._create_origin_destination_matrix_from_flows()
        )

    def _create_origin_destination_matrix_from_flows(
        self,
    ) -> Dict[int, Dict[int, List[DynamicObstacle]]]:
        od_matrix: DefaultDict[int, DefaultDict[int, List[DynamicObstacle]]] = (
            defaultdict(lambda: defaultdict(list))
        )
        for flow in self._flows:
            od_matrix[flow.start_lanelet_id][flow.end_lanelet_id].append(
                flow.dynamic_obstacle
            )

        return dict(od_matrix)

    def get_origin_spawn_rate(self, lanelet_id: int) -> float:
        if lanelet_id not in self._origin_destination_matrix:
            return 0.0

        total_obstacles_in_routes_from_start_id = sum(
            len(obstacles)
            for obstacles in self._origin_destination_matrix[lanelet_id].values()
        )

        return round(total_obstacles_in_routes_from_start_id / self._scenario_length, 2)

    def get_origin_destination_spawn_rate(
        self, start_lanelet_id: int, end_lanelet_id: int
    ) -> float:
        if start_lanelet_id not in self._origin_destination_matrix:
            return 0.0

        if end_lanelet_id not in self._origin_destination_matrix[start_lanelet_id]:
            return 0.0

        return round(
            (
                len(self._origin_destination_matrix[start_lanelet_id][end_lanelet_id])
                / self._scenario_length
            ),
            2,
        )

    def get_origin_destination_probability(
        self, start_lanelet_id: int, end_lanelet_id: int
    ) -> float:
        if start_lanelet_id not in self._origin_destination_matrix:
            return 0.0

        total_obstacles_in_routes_from_start_id = len(
            self._origin_destination_matrix[start_lanelet_id].values()
        )

        end_lanelet_counts = {}
        for end_lanelet_id, dynamic_obstacles in self._origin_destination_matrix[
            start_lanelet_id
        ].items():
            end_lanelet_counts[end_lanelet_id] = len(dynamic_obstacles)

        return round(
            end_lanelet_counts[end_lanelet_id]
            / total_obstacles_in_routes_from_start_id,
            2,
        )

    def get_origin_destination_pairs(self) -> List[Tuple[int, int]]:
        od_pairs = []
        for origin in self._origin_destination_matrix.keys():
            for destination in self._origin_destination_matrix[origin].keys():
                od_pairs.append((origin, destination))
        return od_pairs

    def get_obstacle_type_probability(
        self, start_lanelet_id: int, end_lanelet_id: int, obstacle_type: ObstacleType
    ) -> float:
        if start_lanelet_id not in self._origin_destination_matrix:
            return 0.0

        if end_lanelet_id not in self._origin_destination_matrix[start_lanelet_id]:
            return 0.0

        obstacle_type_distribution = _compute_obstacle_type_distribution(
            self._origin_destination_matrix[start_lanelet_id][end_lanelet_id]
        )
        if obstacle_type not in obstacle_type_distribution:
            return 0.0

        return obstacle_type_distribution[obstacle_type]


class InfrastructureDemandCalculator(ReferenceDemandCalculator):
    def __init__(self, scenario: Scenario) -> None:
        super().__init__(scenario)

        self._lanelet_capacities = _compute_lanelet_capacities_from_flows(
            self._flows, self._scenario
        )

    def get_origin_destination_spawn_rate(
        self, start_lanelet_id: int, end_lanelet_id: int
    ) -> float:
        if start_lanelet_id not in self._origin_destination_matrix:
            return 0.0

        if end_lanelet_id not in self._origin_destination_matrix[start_lanelet_id]:
            return 0.0

        if start_lanelet_id not in self._lanelet_capacities:
            return 0.0

        # Lanelet capacities are defined only for the start lanelet.
        # Since the spawn rate for this O/D pair was requested, the start lanelet capacity
        # must be adjusted so that, across all O/D pairs with the same start lanelet,
        # the total matches the start lanelet capacity.
        # This prevents overloading the lanelet network.
        total_obstacles_in_routes_from_start_id = sum(
            len(obstacles)
            for obstacles in self._origin_destination_matrix[start_lanelet_id].values()
        )
        obstacles_in_route_to_end_id = len(
            self._origin_destination_matrix[start_lanelet_id][end_lanelet_id]
        )

        origin_destination_frac = (
            obstacles_in_route_to_end_id / total_obstacles_in_routes_from_start_id
        )

        # TODO: Divide by 60 seems a bit superflous because the lanelet_capacities are all multiplacted with 60 during their computation. Maybe this can be streamlined?
        return round(
            (self._lanelet_capacities[start_lanelet_id] * origin_destination_frac) / 60,
            2,
        )


class RandomDemandCalculator(AbstractDemandCalculator):
    def __init__(self, scenario: Scenario, seed: int) -> None:
        super().__init__(scenario)
        self._random = random.Random(seed)

        self._od_spawn_probability_matrix: Dict[Tuple[int, int], float] = {}
        self._od_obstacle_probability_matrix: Dict[
            Tuple[int, int, ObstacleType], float
        ] = {}
        self._create_random_od_matrix_for_scenario(scenario)

    def _create_random_od_matrix_for_scenario(self, scenario: Scenario) -> None:
        """
        Initialize the O/D matrix based on the `LaneletNetwork` of `scenario`.

        :returns: Nothing.
        """
        for origin_lanelet in scenario.lanelet_network.lanelets:
            if len(origin_lanelet.predecessor) > 0:
                # A lanelet is considered an origin lanelet, if it does not have any predecessing lanelets.
                continue

            load_factor = abs(self._random.normalvariate(mu=0.3, sigma=0.2))

            # Choose all destination lanelets as
            destination_lanelet_ids = _find_all_reachable_end_lanelets(
                scenario.lanelet_network, origin_lanelet
            )

            distribution = np.random.uniform(0, 1, len(destination_lanelet_ids))
            for i, destination_lanelet_id in enumerate(destination_lanelet_ids):
                # TODO: Hardcoded values
                freq_car = max(0.0, self._random.normalvariate(0.86478, 0.03313))
                freq_bus = max(0.0, self._random.normalvariate(0.00511, 0.00297))
                freq_truck = max(0.0, self._random.normalvariate(0.11999, 0.02962))
                freq_motorcycle = max(0.0, self._random.normalvariate(0.00912, 0.00630))

                sum_freq = freq_car + freq_bus + freq_truck + freq_motorcycle
                frequencies = [
                    freq_car / sum_freq,
                    freq_bus / sum_freq,
                    freq_truck / sum_freq,
                    freq_motorcycle / sum_freq,
                ]

                obstacle_types = [
                    ObstacleType.CAR,
                    ObstacleType.BUS,
                    ObstacleType.TRUCK,
                    ObstacleType.MOTORCYCLE,
                ]

                for freq, obstacle_type in zip(frequencies, obstacle_types):
                    self._od_obstacle_probability_matrix[
                        (
                            origin_lanelet.lanelet_id,
                            destination_lanelet_id,
                            obstacle_type,
                        )
                    ] = freq

                speed_limit = _get_speed_limit_on_lanelet(
                    origin_lanelet, scenario.lanelet_network
                )
                lengths = (
                    frequencies[0] * 4.19
                    + frequencies[1] * 12
                    + frequencies[2] * 12
                    + frequencies[3] * 2.1
                )  # we use the default OTS length values
                s_0 = 2.0  # m -- minimum distance between vehicles
                T = 1.45  # s -- desired time headway
                demand = speed_limit / (lengths + (s_0 + speed_limit * T))
                od_pair_choosen_probability = distribution[i] / sum(distribution)
                self._od_spawn_probability_matrix[
                    (origin_lanelet.lanelet_id, destination_lanelet_id)
                ] = demand * load_factor * od_pair_choosen_probability

    def get_origin_destination_spawn_rate(
        self, start_lanelet_id: int, end_lanelet_id: int
    ) -> float:
        if (start_lanelet_id, end_lanelet_id) not in self._od_spawn_probability_matrix:
            return 0.0

        return round(
            self._od_spawn_probability_matrix[(start_lanelet_id, end_lanelet_id)], 2
        )

    def get_obstacle_type_probability(
        self, start_lanelet_id: int, end_lanelet_id: int, obstacle_type: ObstacleType
    ) -> float:
        if (
            start_lanelet_id,
            end_lanelet_id,
            obstacle_type,
        ) not in self._od_obstacle_probability_matrix:
            return 0.0

        return round(
            self._od_obstacle_probability_matrix[
                (start_lanelet_id, end_lanelet_id, obstacle_type)
            ],
            2,
        )

    def get_origin_destination_pairs(self) -> List[Tuple[int, int]]:
        return list(self._od_spawn_probability_matrix.keys())
