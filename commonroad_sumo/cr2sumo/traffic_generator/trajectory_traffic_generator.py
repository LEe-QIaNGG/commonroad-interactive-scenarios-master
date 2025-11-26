import logging
import math
from dataclasses import dataclass

from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from typing_extensions import override

import sumolib
from commonroad_sumo.cr2sumo.map_converter.mapping import VEHICLE_TYPE_CR2SUMO
from commonroad_sumo.cr2sumo.map_converter.util import (
    get_state_list_of_dynamic_obstacle,
)
from commonroad_sumo.cr2sumo.traffic_generator.traffic_generator import (
    AbstractTrafficGenerator,
)
from commonroad_sumo.errors import SumoTrafficGenerationError
from commonroad_sumo.helpers import SumoApplication, execute_sumo_application
from commonroad_sumo.sumolib.net import SumoVehicle, SumoVehicleType
from commonroad_sumo.sumolib.sumo_project import SumoFileType, SumoProject
from commonroad_sumo.sumolib.xml import SumoXmlFile

from .sumo_map_matcher import SumoMapMatcher

_LOGGER = logging.getLogger(__name__)


@dataclass
class ResimulationTrafficGeneratorConfig:
    map_matching_delta: float = 5.0
    """Control the distance in which edges are considered for a route. Inncreasing this might help to reduce route mapping issues, but might produce unrealistic routes."""

    extend_routes_upstream_junction: bool = False
    """For vehicles which start inside junctions, extend the route to an edge before the junction.
     By enabling this option, vehicles which start inside a junction can be simulated, but their route will diverege from the CommonRoad route.
     If this options is disabled such vehicles cannot be simulated and are excluded from the simulation."""

    extend_routes_downstream_junction: bool = False
    """For vehicles which start inside junctions, extend the route to an edge before the junction.
     By enabling this option, vehicles which start inside a junction can be simulated, but their route will diverege from the CommonRoad route.
     If this options is disabled such vehicles cannot be simulated and are excluded from the simulation."""

    extend_routes: bool = False
    """By default, the trajectory conversion tries to create routes which match the given trajectories as close as possible.
    This means that vehicles in the simulation drive only to the position of their last state, and not to the end of edges.
    When general route extension is enabled, the vehicles try to drive to at least the end of each edge."""


class _AbstractResimulationTrafficGenerator(AbstractTrafficGenerator):
    """
    Base traffic generator for re-simulation.

    Handles the traffic generation for re-simulation by converting the original trajectories to SUMO routes.
    It tries to keep the generated routes as close as possible to the original. By adjusting
    the config parameters, this behavior can be modified to be able to simulate more scenarios.

    :param safe: Controls whether insertion checks are enabled (True), or disable for the generated SUMO vehicles.
    :param config: Provide additional configuration, to control the trajectory conversion.
    """

    def __init__(
        self,
        safe: bool = True,
        config: ResimulationTrafficGeneratorConfig | None = None,
    ) -> None:
        super().__init__()
        self._safe = safe

        self._config = config or ResimulationTrafficGeneratorConfig()

    @override
    def generate_traffic(self, scenario: Scenario, sumo_project: SumoProject) -> bool:
        # Put the vehicle definitions *before* the routes. This is important since SUMO loads
        # the route definitions file lazily. If the vehicle type definitions come after the
        # routes, SUMO would fail.
        route_definition_file = _create_route_file_with_vehicle_type_definitions(
            scenario, sumo_project
        )

        sumo_net: sumolib.net.Net = sumolib.net.readNet(
            sumo_project.get_file_path(SumoFileType.NET)
        )
        for dynamic_obstacle in scenario.dynamic_obstacles:
            if dynamic_obstacle.prediction is None:
                continue

            if not isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
                raise SumoTrafficGenerationError(
                    f"Failed to convert trajectories for scenario {scenario.scenario_id}: CommonRoad dynamic obstacle {dynamic_obstacle.obstacle_id} trajectory is of type {type(dynamic_obstacle.prediction)}, but must be {type(TrajectoryPrediction)}"
                )

            # Try to convert the dynamic obstacle to a SUMO vehicle.
            sumo_vehicle = self._create_sumo_vehicle_from_commonroad_dynamic_obstacle(
                dynamic_obstacle, scenario, sumo_net
            )
            if sumo_vehicle is not None:
                route_definition_file.add_node(sumo_vehicle)

        sumo_project.write()

        # Use `duarouter` to repair and reroute the vehicles.
        # This improves the reliability of the simulation since it ensures that all routes are valid.
        duarouter_result = execute_sumo_application(
            SumoApplication.DUAROUTER,
            [
                "-n",
                str(sumo_project.get_file_path(SumoFileType.NET)),
                "-r",
                str(sumo_project.get_file_path(SumoFileType.VEHICLE_ROUTES)),
                "--ignore-errors",
                "--repair",
                "true",
                "-o",
                str(sumo_project.get_file_path(SumoFileType.VEHICLE_ROUTES)),
            ],
        )
        if duarouter_result is None:
            raise SumoTrafficGenerationError(
                f"Failed to convert trajectories for scenario {scenario.scenario_id}: duarouter failed to repair SUMO routes."
            )

        return True

    def _create_sumo_vehicle_from_commonroad_dynamic_obstacle(
        self,
        dynamic_obstacle: DynamicObstacle,
        scenario: Scenario,
        sumo_net: sumolib.net.Net,
    ) -> SumoVehicle | None:
        """
        Create a vehicle in SUMO from a CommonRoad dynamic obstacle.

        :param dynamic_obstacle: The dynamic obstacle that should be converted.
        :param scenario: The scenario in which the obstacle is contained.
        :param sumo_net: SUMO network matching the scenario.

        :returns: A new SumoVehicle with a route that matches the original obstacle, or None if the route is invalid.
        """
        edges = self._get_route_for_dynamic_obstacle(dynamic_obstacle, sumo_net)
        if len(edges) == 0:
            _LOGGER.warning(
                "The trajectory of the vehicle %s could not be mapped to any edges, because it either happens outside of the lanelet network or happens soley inside a SUMO junction. It will be skipped.",
                dynamic_obstacle.obstacle_id,
            )
            return None

        depart_lane_idx, depart_lane_pos = self._get_depart_location(
            tuple(dynamic_obstacle.initial_state.position), edges[0]
        )
        arrival_lane_idx, arrival_lane_pos = self._get_arrival_location(
            tuple(dynamic_obstacle.prediction.trajectory.final_state.position),
            edges[-1],
        )
        vehicle = SumoVehicle(
            vehicle_id=str(dynamic_obstacle.obstacle_id),
            depart_time=dynamic_obstacle.initial_state.time_step * scenario.dt,
            depart_speed=dynamic_obstacle.initial_state.velocity,
            depart_lane_id=depart_lane_idx,
            depart_pos=depart_lane_pos,
            arrival_lane_id=arrival_lane_idx,
            arrival_pos=arrival_lane_pos,
            vehicle_type=VEHICLE_TYPE_CR2SUMO[dynamic_obstacle.obstacle_type].value,
            edge_ids=[edge.getID() for edge in edges],
            insertion_checks=self._safe,
        )
        return vehicle

    def _get_route_for_dynamic_obstacle(
        self,
        dynamic_obstacle: DynamicObstacle,
        sumo_net: sumolib.net.Net,
    ) -> list[sumolib.net.edge.Edge]:
        """
        Map the state list of the CommonRoad dynamic obstacle onto the SUMO network to create a SUMO route.

        :param dynamic_obstacle: CommonRoad dynamic obstacle which should be mapped.
        :param sumo_net: The SUMO network for the scenario of the dynamic obstacle.

        :returns: The list of SUMO edges in the route of the vehicle. Route might be empty if the vehicle cannot be mapped on any valid SUMO edges.
        """
        state_list = get_state_list_of_dynamic_obstacle(dynamic_obstacle)
        position_trace = [tuple(state.position) for state in state_list]

        map_matcher = SumoMapMatcher(sumo_net, self._config.map_matching_delta)
        edges = map_matcher.map_trace(position_trace)

        if self._config.extend_routes_upstream_junction:
            # If the vehicle starts inside a junction, its route is modified to start before the junction, so that the vehicle can be simulated.
            start_pos = position_trace[0]
            start_orientation = state_list[0].orientation
            next_edge = edges[0] if len(edges) > 0 else None
            junction_at_start_poition = map_matcher.junction_at_position(
                start_pos, outgoing_edge=next_edge
            )
            # Check if start position is in a junction
            if junction_at_start_poition is not None:
                # Try to add upstream edges
                upstream_edge = _get_likely_upstream_edge_from_junction(
                    junction_at_start_poition,
                    start_pos,
                    start_orientation,
                    next_edge,
                )
                if upstream_edge is None:
                    _LOGGER.debug(
                        f"Failed to determine route before junction for obstacle {dynamic_obstacle.obstacle_id}"
                    )
                # Only insert the edge if it is not a duplicate, to ensure the routes stay valid.
                elif len(edges) == 0 or edges[0].getID() != upstream_edge.getID():
                    _LOGGER.debug(
                        f"Selected {upstream_edge.getID()} as most likely uptstream edge for dynamic obstacle {dynamic_obstacle.obstacle_id} at time step {state_list[0].time_step}"
                    )
                    edges.insert(0, upstream_edge)

        if self._config.extend_routes_downstream_junction:
            # If the vehicle starts inside a junction, its route is modified to end after the junction, so that the vehicle can be simulated.
            end_pos = position_trace[-1]
            end_orientation = state_list[-1].orientation
            prev_edge = edges[-1] if len(edges) > 0 else None
            junction_at_end_poition = map_matcher.junction_at_position(
                end_pos, incoming_edge=prev_edge
            )
            # Check if start position is in a junction
            if junction_at_end_poition is not None:
                # Try to add downstream edges
                downstream_edge = _get_likely_downstream_edge_from_junction(
                    junction_at_end_poition,
                    end_pos,
                    end_orientation,
                    next_edge,
                )
                if downstream_edge is None:
                    _LOGGER.debug(
                        f"Failed to determine route after junction for obstacle {dynamic_obstacle.obstacle_id}"
                    )
                # Only insert the edge if it is not a duplicate, to ensure the routes stay valid.
                elif len(edges) == 0 or edges[-1].getID() != downstream_edge.getID():
                    _LOGGER.debug(
                        f"Selected {downstream_edge.getID()} as most likely downstream edge for dynamic obstacle {dynamic_obstacle.obstacle_id} at time step {state_list[-1].time_step}"
                    )
                    edges.append(downstream_edge)

        return edges

    def _get_depart_location(
        self, position: tuple[float, float], depart_edge: sumolib.net.edge.Edge
    ) -> tuple[int, float]:
        """Determine the corresponding depart location inside the edge.

        Since SUMO does not use cartesian coordinates for departure information,
        the cartesian departure position must be mapped to a position relative to the edge and lane.
        """
        depart_lane_idx, depart_lane_pos, _ = depart_edge.getClosestLanePosDist(
            position
        )
        depart_lane_idx = depart_lane_idx or 0
        depart_lane_pos = depart_lane_pos or 0.0
        return (depart_lane_idx, depart_lane_pos)

    def _get_arrival_location(
        self, position: tuple[float, float], arrival_edge: sumolib.net.edge.Edge
    ) -> tuple[int, float]:
        """Determine the corresponding arrival location inside the edge.

        Since SUMO does not use cartesian coordinates for arrival information,
        the cartesian arrival position must be mapped to a position relative to the edge and lane.
        """
        arrival_lane_idx, arrival_lane_pos, _ = arrival_edge.getClosestLanePosDist(
            position
        )
        arrival_lane_idx = arrival_lane_idx or 0
        arrival_lane_pos = arrival_lane_pos or arrival_edge.getLength() - 1e-3
        if self._config.extend_routes:
            return (arrival_lane_idx, arrival_edge.getLength() - 1e-3)
        else:
            return (arrival_lane_idx, arrival_lane_pos)


class SafeResimulationTrafficGenerator(_AbstractResimulationTrafficGenerator):
    def __init__(
        self, config: ResimulationTrafficGeneratorConfig | None = None
    ) -> None:
        super().__init__(safe=True, config=config)


class UnsafeResimulationTrafficGenerator(_AbstractResimulationTrafficGenerator):
    def __init__(
        self, config: ResimulationTrafficGeneratorConfig | None = None
    ) -> None:
        super().__init__(safe=False, config=config)


def _create_route_file_with_vehicle_type_definitions(
    scenario: Scenario, sumo_project: SumoProject
) -> SumoXmlFile:
    route_definition_file = sumo_project.create_file(SumoFileType.VEHICLE_ROUTES)

    for obstacle_type in _get_set_of_obstacle_types_in_scenario(scenario):
        vehicle_type = VEHICLE_TYPE_CR2SUMO[obstacle_type]
        route_definition_file.add_node(SumoVehicleType.from_vehicle_type(vehicle_type))

    return route_definition_file


def _get_set_of_obstacle_types_in_scenario(scenario: Scenario) -> set[ObstacleType]:
    obstacle_types = set()
    for obstacle in scenario.dynamic_obstacles:
        obstacle_types.add(obstacle.obstacle_type)

    return obstacle_types


def _get_likely_upstream_edge_from_junction(
    junction: sumolib.net.node.Node,
    position: tuple[float, float],
    orientation: float,
    next_edge: sumolib.net.edge.Edge | None = None,
) -> sumolib.net.edge.Edge | None:
    """
    Find an incoming edge of the junction which can be used as a starting point for vehicles starting inside junctions.

    :param junction: Junction for which the incoming edge should be found.
    :param position: Position inside the junction. Is used to find the closest edge.
    :param orientation: Orientation of vehicle. Is used to search the most likely edge.
    :param next_edge: Hint to the search, which edge is targeted after the junction. Helps to filter out incoming edges which do not connect to the target edge.

    :returns: A likely upstream edge, or None if no valid upstream edge could be found.
    """

    incoming_edges: list[sumolib.net.edge.Edge] = junction.getIncoming()

    # Get the maximum distance to all edges, so that the distance can later be normalized for the scoring.
    max_distance = max(
        sumolib.geomhelper.distancePointToPolygon(position, incoming_edge.getShape())
        for incoming_edge in incoming_edges
    )

    current_best_edge = None
    current_best_score = math.inf

    for incoming_edge in incoming_edges:
        if incoming_edge.isSpecial():  # Skip internal edges
            continue

        # If we got a hint for the next edge, we can use this to select only upstream edges
        # which are also connected to this edge.
        if next_edge is not None:
            connections_from_incoming_to_next = junction.getConnections(
                incoming_edge, next_edge
            )
            if len(connections_from_incoming_to_next) == 0:
                continue

        distance_to_incoming = sumolib.geomhelper.distancePointToPolygon(
            position, incoming_edge.getShape()
        )
        normalized_distance = distance_to_incoming / max_distance

        # Get initial orientation of incoming edge.
        # For incoming edges we need to consider the rotation at the end of the edge.
        rotation = sumolib.geomhelper.rotationAtShapeOffset(
            incoming_edge.getShape(), incoming_edge.getLength() - 1e-3
        )
        if rotation is not None:
            angle_diff = sumolib.geomhelper.minAngleDegreeDiff(
                math.degrees(rotation),
                math.degrees(orientation),
            )
            normalized_angle_diff = angle_diff / 180.0
        else:
            # Fallback when rotation of edge could not be determined.
            normalized_angle_diff = 0.5

        # Determine a score between 0.0 and 2.0. Smaller score = better match.
        new_score = normalized_angle_diff + normalized_distance

        if new_score < current_best_score:
            current_best_score = new_score
            current_best_edge = incoming_edge

    return current_best_edge


def _get_likely_downstream_edge_from_junction(
    junction: sumolib.net.node.Node,
    position: tuple[float, float],
    orientation: float,
    prev_edge: sumolib.net.edge.Edge | None = None,
) -> sumolib.net.edge.Edge | None:
    """
    Find an outgoing edge of the junction which is likely the next edge for a vehicle at position.

    Uses a simple scoring system to weigh orientation and distance to outgoing edges, to select the most likely downstream edge.

    :param junction: Junction for which the outgoing edge should be found.
    :param position: Position inside the junction. Is used to search the most likely edge.
    :param orientation: Orientation of vehicle. Is used to search the most likely edge.
    :param prev_edge: Hint to the search, which edge is targeted before the junction. Helps to filter out outgoing edges which do not connect to the target edge.

    :returns: A likely downstream edge, or None if no valid downstream edge could be found.
    """
    outgoing_edges: list[sumolib.net.edge.Edge] = junction.getOutgoing()

    # Get the maximum distance to all edges, so that the distance can later be normalized for the scoring.
    max_distance = max(
        sumolib.geomhelper.distancePointToPolygon(position, outgoing_edge.getShape())
        for outgoing_edge in outgoing_edges
    )

    current_best_edge = None
    current_best_score = math.inf

    for outgoing_edge in outgoing_edges:
        if outgoing_edge.isSpecial():  # Skip internal edges
            continue

        # If we got a hint for the previous edge, we can use this to select only downstream edges
        # which are also connected to this edge.
        if prev_edge is not None:
            connections_from_prev_to_outgoing = junction.getConnections(
                prev_edge, outgoing_edge
            )
            # Skip downstream edges which are not connected.
            if len(connections_from_prev_to_outgoing) == 0:
                continue

        distance_to_outgoing = sumolib.geomhelper.distancePointToPolygon(
            position, outgoing_edge.getShape()
        )
        normalized_distance = distance_to_outgoing / max_distance

        # Get initial orientation of outgoing edge.
        # For outgoing edges we need to consider the rotation at the start of the edge.
        rotation = sumolib.geomhelper.rotationAtShapeOffset(
            outgoing_edge.getShape(), 1e-3
        )
        if rotation is not None:
            angle_diff = sumolib.geomhelper.minAngleDegreeDiff(
                math.degrees(rotation), math.degrees(orientation)
            )
            normalized_angle_diff = angle_diff / 180.0
        else:
            # Fallback when rotation of edge could not be determined.
            normalized_angle_diff = 0.5

        # Determine a score between 0.0 and 2.0. Smaller score = better match.
        new_score = normalized_distance + normalized_angle_diff

        if new_score < current_best_score:
            current_best_score = new_score
            current_best_edge = outgoing_edge

    return current_best_edge
