import itertools
import logging
import subprocess
import warnings
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
from functools import reduce
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple

import matplotlib.colors as mcolors
import networkx as nx
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.lanelet import (
    Lanelet,
    LaneletNetwork,
    LaneletType,
    LineMarking,
)
from commonroad.scenario.scenario import Scenario, Tag
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
    TrafficLightDirection,
)
from commonroad.scenario.traffic_sign import SupportedTrafficSignCountry, TrafficSign
from commonroad.scenario.traffic_sign_interpreter import TrafficSignInterpreter
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt
from shapely.geometry import LineString, Point
from typing_extensions import Self

from commonroad_sumo.cr2sumo.map_converter import (
    partition_lanelet_network_into_edges_and_lanes,
)
from commonroad_sumo.cr2sumo.map_converter.intersections import (
    determine_end_node_type_for_lanelets,
    determine_start_node_type_for_lanelets,
    is_highway_intersection,
    lanelets_are_highway,
)
from commonroad_sumo.cr2sumo.map_converter.util import (
    _find_intersecting_edges,
    max_lanelet_network_id,
    merge_lanelets,
    min_cluster,
)
from commonroad_sumo.helpers import (
    SumoApplication,
    execute_sumo_application,
)
from commonroad_sumo.sumolib.net import (
    TLS,
    Connection,
    Crossing,
    Edge,
    Junction,
    Lane,
    Net,
    Node,
    NodeType,
    RightOfWay,
    Roundabout,
    SpreadType,
    VehicleType,
    sumo_net_from_xml,
)
from commonroad_sumo.sumolib.sumo_project import (
    SumoFileType,
    SumoIntermediateFileType,
    SumoIntermediateProject,
    SumoProject,
)

from .mapping import (
    ClusterInstruction,
    directions_SUMO2CR,
    get_edge_types_from_template,
    get_sumo_edge_type,
    traffic_light_states_SUMO2CR,
)
from .traffic_light import TrafficLightEncoder
from .traffic_sign import TrafficSignEncoder

_LOGGER = logging.getLogger(__name__)


def _compute_node_coords(lanelets, index: int):
    vertices = np.array([la.center_vertices[index] for la in lanelets])
    return np.mean(vertices, axis=0)


class PartionedLaneletNetworkGraph:
    def __init__(self, edges2lanes, graph):
        self._graph = graph
        self._edges2lanes = edges2lanes
        self._lane2edge = {}
        for edge_id, lane_ids in edges2lanes.items():
            for lane_id in lane_ids:
                self._lane2edge[lane_id] = edge_id

    def get_lane_ids_for_edge_id(self, edge_id: int) -> List[int]:
        return self._edges2lanes[edge_id]

    def find_edge_id_for_lane_id(self, lane_id: int) -> int:
        return self._lane2edge[lane_id]


def convert_lanelet_network_to_edge_graph(
    lanelet_network: LaneletNetwork,
) -> PartionedLaneletNetworkGraph:
    cr_lane_ids_by_cr_edge_ids = partition_lanelet_network_into_edges_and_lanes(
        lanelet_network
    )

    lane_id2edge_id = {}
    for edge_id, lane_ids in cr_lane_ids_by_cr_edge_ids.items():
        for lane_id in lane_ids:
            lane_id2edge_id[lane_id] = edge_id

    node_idx = 0
    start_nodes = {}
    end_nodes = {}
    for edge_id, lane_ids in cr_lane_ids_by_cr_edge_ids.items():
        successors = set()
        predecessors = set()
        lanelets = []
        for lane_id in lane_ids:
            lanelet = lanelet_network.find_lanelet_by_id(lane_id)
            if lanelet is None:
                continue
            lanelets.append(lanelet)

            for successor_edge in lanelet.successor:
                successors.add(lane_id2edge_id[successor_edge])

            for predecessor_edge in lanelet.predecessor:
                predecessors.add(lane_id2edge_id[predecessor_edge])

        start_node_id = None
        for predecessor in predecessors:
            if predecessor in end_nodes:
                start_node_id = end_nodes[predecessor]
                break

        if start_node_id is None:
            start_node_id = node_idx
            node_idx += 1

        end_node_id = None
        for successor in successors:
            if successor in start_nodes:
                end_node_id = start_nodes[successor]
                break

        if end_node_id is None:
            end_node_id = node_idx
            node_idx += 1

        start_nodes[edge_id] = start_node_id
        end_nodes[edge_id] = end_node_id

    graph = []
    for edge_id, lane_ids in cr_lane_ids_by_cr_edge_ids.items():
        start_node_id = start_nodes[edge_id]
        end_node_id = end_nodes[edge_id]

        graph.append((start_node_id, end_node_id, edge_id))

    return PartionedLaneletNetworkGraph(cr_lane_ids_by_cr_edge_ids, graph)


@dataclass
class SumoNetwork:
    nodes: List[Node]
    edges: List[Edge]
    lanes: List[Lane]
    connections: List[Connection]


def calculate_lanelet_width_from_cr(lanelet: Lanelet) -> float:
    """
    Calculate the average width of a lanelet.
    :param lanelet: the lane whose width is to be calculated
    :return: average_width
    """
    distances = np.linalg.norm(
        lanelet.right_vertices[:, np.newaxis, :]
        - lanelet.left_vertices[np.newaxis, :, :],
        axis=2,
    )

    # Compute average distance
    avg_distance = np.mean(np.min(distances, axis=1)) + np.mean(
        np.min(distances, axis=0)
    )
    avg_distance /= 2  # Average between both directions
    return avg_distance


def convert_lanelet_network_graph_to_sumo_network(
    graph: PartionedLaneletNetworkGraph, lanelet_network: LaneletNetwork
) -> SumoNetwork:
    sumo_nodes = {}
    sumo_edges = {}
    sumo_lanes = {}
    sumo_connections = []
    for start_node_id, end_node_id, edge_id in graph._graph:
        lanelets = [
            lanelet_network.find_lanelet_by_id(lane_id)
            for lane_id in graph.get_lane_ids_for_edge_id(edge_id)
        ]
        if start_node_id not in sumo_nodes:
            coords = _compute_node_coords(lanelets, index=0)
            sumo_node = Node(start_node_id, NodeType.UNREGULATED, coords)
            sumo_nodes[start_node_id] = sumo_node

        if end_node_id not in sumo_nodes:
            coords = _compute_node_coords(lanelets, index=-1)
            sumo_node = Node(end_node_id, NodeType.UNREGULATED, coords)
            sumo_nodes[end_node_id] = sumo_node

        edge = Edge(
            edge_id,
            from_node=sumo_nodes[start_node_id],
            to_node=sumo_nodes[end_node_id],
            spread_type=SpreadType.CENTER,
        )
        sumo_edges[edge_id] = edge

        for lanelet in lanelets:
            width = calculate_lanelet_width_from_cr(lanelet)
            lane = Lane(
                edge,
                speed=16,
                length=lanelet.distance[-1],
                width=width,
                shape=lanelet.center_vertices,
            )
            sumo_lanes[lanelet.lanelet_id] = lane

    for lane_id in sumo_lanes.keys():
        lanelet = lanelet_network.find_lanelet_by_id(lane_id)

        for successor in lanelet.successor:
            from_edge_id = graph.find_edge_id_for_lane_id(lane_id)
            from_edge = sumo_edges[from_edge_id]
            to_edge_id = graph.find_edge_id_for_lane_id(successor)
            to_edge = sumo_edges[to_edge_id]
            to_lane = sumo_lanes[successor]
            from_lane = sumo_lanes[lane_id]

            connection = Connection(from_edge, to_edge, from_lane, to_lane)
            sumo_connections.append(connection)

    return SumoNetwork(
        list(sumo_nodes.values()),
        list(sumo_edges.values()),
        list(sumo_lanes.values()),
        sumo_connections,
    )


def create_intermediate_sumo_project_from_sumo_network(
    sumo_network: SumoNetwork, output_folder: Path
) -> SumoIntermediateProject:
    sumo_project = SumoIntermediateProject.from_folder(output_folder)

    nodes_file = sumo_project.create_file(SumoIntermediateFileType.NODES)
    for node in sumo_network.nodes:
        nodes_file.add_node(node)

    connections_file = sumo_project.create_file(SumoIntermediateFileType.CONNECTIONS)
    for connection in sumo_network.connections:
        connections_file.add_node(connection)

    edges_file = sumo_project.create_file(SumoIntermediateFileType.EDGES)
    for edge in sumo_network.edges:
        edges_file.add_node(edge)

    sumo_project.write()

    return sumo_project


def convert_intermediate_sumo_project_with_netconvert(
    sumo_intermediate_project: SumoIntermediateProject, cleanup: bool
) -> Optional[SumoProject]:
    """
    Function that merges the edges and nodes files into one using netconvert
    :param output_path
    :param cleanup: deletes temporary input files after creating net file (only deactivate for debugging)
    :param connections_path:
    :param nodes_path:
    :param edges_path:
    :param traffic_path:
    :param type_path:
    :param output_path: the relative path of the output
    :return: bool: returns False if conversion fails
    """

    sumo_project = SumoProject.from_intermediate_sumo_project(sumo_intermediate_project)

    netconvert_result = execute_sumo_application(
        SumoApplication.NETCONVERT,
        [
            "--no-turnarounds=true",
            "--junctions.join=true",
            "--junctions.join-dist=20",
            "--junctions.join-same=true",
            "--junctions.join-turns=true",
            "--junctions.scurve-stretch=5.0",
            "--junctions.internal-link-detail=20",
            "--junctions.corner-detail=20",
            "--junctions.endpoint-shape=true",
            "--edges.join=true",
            "--ramps.guess=true",
            "--plain.extend-edge-shape=true",
            "--geometry.avoid-overlap=true",
            "--geometry.remove.min-length=5.0",  # Allow merging of edges
            "--fringe.guess=true",
            "--tls.guess-signals=true",
            "--tls.group-signals=true",
            f"--tls.green.time={50}",
            f"--tls.red.time={50}",
            f"--tls.yellow.time={10}",
            f"--tls.allred.time={50}",
            f"--tls.left-green.time={50}",
            f"--tls.crossing-min.time={50}",
            f"--tls.crossing-clearance.time={50}",
            "--offset.disable-normalization=true",
            f"--node-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.NODES)}",
            f"--edge-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.EDGES)}",
            f"--connection-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.CONNECTIONS)}",
            # f"--tllogic-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.TLLOGICS)}",
            # f"--type-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.TYPES)}",
            f"--output-file={sumo_project.get_file_path(SumoFileType.NET)}",
        ],
    )

    if netconvert_result is None:
        _LOGGER.error(
            "Failed to merge intermediate files: netconvert failed with an unknown error!"
        )
        return None

    # All warnings produced by netconvert are considered debug messages,
    # because they are usually rather informative
    # and do not affect the functionality of the simulation
    for line in netconvert_result.splitlines():
        if line.startswith("Warning"):
            warning_message = line.lstrip("Warning: ")
            _LOGGER.debug(
                f"netconvert produced a warning while converting network: {warning_message}"
            )
        else:
            _LOGGER.debug("netconvert output: %s", line)

    if cleanup:
        sumo_intermediate_project.cleanup()

    return sumo_project


@dataclass
class CR2SumoMapConverterConfig:
    highway_mode: bool = True
    """If enabled the map conversion uses less aggressive clustering, and creates zipper junctions for ramps."""

    country_id: SupportedTrafficSignCountry = SupportedTrafficSignCountry.ZAMUNDA
    """The country of the scenarios for which the convert is employed. This is used to interpret
     traffic signs and also determines the default speed limits and available edge types."""

    # TODO: If this parameter is set to None, it breaks the tests.
    overwrite_speed_limit: float | None = 130 / 3.6
    """If set, the value will be used as speed limit for all edges instead of the speed limits
     from the CommonRoad scenario."""

    wait_pos_internal_junctions: float = -4.0
    """Shifted waiting position at junction in meters (equivalent to SUMO's contPos parameter)."""

    random_seed: int = 1234

    @classmethod
    def from_scenario(cls, scenario: Scenario) -> Self:
        """
        Create the configuration for the cr2sumo map converter based on a given scenario.

        This method will try to auto-detect highway mode and country ID.
        """
        # Automatically detect whether highway mode should be disabled.
        highway_mode = True
        if scenario.tags is not None:
            if Tag.INTERSECTION in scenario.tags:
                highway_mode = False

        _LOGGER.debug(
            f"Automatically determined highway mode {'on' if highway_mode else 'off'} for scenario {scenario.scenario_id}"
        )

        # Automatically determine the country ID.
        raw_country_id = scenario.scenario_id.country_id
        try:
            country_id = SupportedTrafficSignCountry(raw_country_id)
        except ValueError:
            country_id = SupportedTrafficSignCountry.ZAMUNDA

        _LOGGER.debug(
            f"Automatically determined country ID {country_id.value} for scenario {scenario.scenario_id}"
        )

        return cls(highway_mode, country_id)


class CR2SumoMapConverter:
    """Converts CommonRoad map to sumo map .net.xml"""

    def __init__(
        self, scenario: Scenario, conf: CR2SumoMapConverterConfig | None = None
    ):
        """
        :param scenario: CommonRoad Scenario to be converted
        :param conf: configuration file for additional map conversion parameters
        """
        self._scenario: Scenario = scenario
        self._lanelet_network = scenario.lanelet_network
        self._conf = conf or CR2SumoMapConverterConfig.from_scenario(scenario)

        # all the nodes of the map, key is the node ID
        self.nodes: Dict[int, Node] = {}
        # all the edges of the map, key is the edge ID
        self.edges: Dict[int, Edge] = {}
        # dictionary for the shape of the edges
        self._points_dict: Dict[int, np.ndarray] = {}
        # all the connections of the map
        # lane_id -> list(lane_id)
        self._connections: Dict[str, List[str]] = defaultdict(list)
        self._new_connections: Set[Connection] = set()
        # collect which lanelet ids are prohibited by a lanelet
        self.prohibits = defaultdict(list)
        # dict of merged_node_id and Crossing
        self._crossings: Dict[int, Set[Lanelet]] = dict()
        # key is the ID of the edges and value the ID of the lanelets that compose it
        self.lanes_dict: Dict[int, List[int]] = {}
        # lane ID -> Lane
        self.lanes: Dict[str, Lane] = {}
        # edge_id -> length (float)
        self.edge_lengths: Dict[str, float] = {}

        # traffic signs
        self._traffic_sign_interpreter: TrafficSignInterpreter = TrafficSignInterpreter(
            self._conf.country_id, self._lanelet_network
        )
        # Read Edge Types from template
        self.edge_types = get_edge_types_from_template(self._conf.country_id)

        self.lane_id2lanelet_id: Dict[str, int] = {}
        self.lanelet_id2lane_id: Dict[int, str] = {}
        self.lanelet_id2edge_id: Dict[int, int] = {}
        self.lanelet_id2edge_lane_id: Dict[int, int] = {}
        self.roundabouts: List[Roundabout] = []
        # generated junctions by NETCONVERT
        self.lanelet_id2junction: Dict[int, Junction] = {}

        # for an edge_id gives id's of start and end nodes
        self._start_nodes: Dict[int, int] = {}
        self._end_nodes: Dict[int, int] = {}

        self.traffic_light_signals = TLS()

        # NETCONVERT files
        self._traffic_file = ""
        self._additional_file = ""
        self._output_file = ""

    @classmethod
    def from_file(cls, file_path_cr, conf: CR2SumoMapConverterConfig | None = None):
        scenario, _ = CommonRoadFileReader(file_path_cr).open()
        return cls(scenario, conf)

    def _convert_map(self):
        self._find_lanes()
        self._init_nodes()
        self._create_sumo_edges_and_lanes()
        self._init_connections()
        self.new_nodes, self.merged_dictionary, replaced_nodes = (
            self._merge_junctions_intersecting_lanelets()
        )
        _LOGGER.debug(f"Merged: {self.merged_dictionary}")
        self.new_edges = self._filter_edges(self.merged_dictionary, replaced_nodes)
        self._create_lane_based_connections()
        # self._set_prohibited_connections()
        self.roundabouts = self._create_roundabouts()
        # self._create_crossings()
        self._encode_traffic_signs()
        self._create_traffic_lights()

    def _find_lanes(self):
        """
        Convert a CommonRoad net into a SUMO net
        sumo_net contains the converted net
        """

        self._points_dict = {
            lanelet.lanelet_id: lanelet.center_vertices
            for lanelet in self._lanelet_network.lanelets
        }
        lanelet_ids_by_edges_ids = partition_lanelet_network_into_edges_and_lanes(
            self._lanelet_network
        )
        for edge_id, lanelet_ids in lanelet_ids_by_edges_ids.items():
            self.lanes_dict[edge_id] = lanelet_ids
            lanelets = [
                self._lanelet_network.find_lanelet_by_id(lanelet_id)
                for lanelet_id in lanelet_ids
            ]
            self.edge_lengths[edge_id] = min(
                [lanelet.distance[-1] for lanelet in lanelets]
            )

            for i_lane, l_id in enumerate(lanelet_ids):
                self.lanelet_id2edge_id[l_id] = edge_id
                self.lanelet_id2edge_lane_id[l_id] = i_lane

    def _compute_node_coords(self, lanelets, index: int):
        vertices = np.array([la.center_vertices[index] for la in lanelets])
        return np.mean(vertices, axis=0)

    def _init_nodes(self):
        # creation of the start and end nodes
        # start node
        self.node_id_next = 1
        self._start_nodes = {}  # contains start nodes of each edge{edge_id: node_id}
        self._end_nodes = {}  # contains end nodes of each edge{edge_id: node_id}

        for edge_id, lanelet_ids in self.lanes_dict.items():
            self._create_node(edge_id, lanelet_ids, "from")
            self._create_node(edge_id, lanelet_ids, "to")

    def _create_node(self, edge_id: int, lanelet_ids: List[int], node_role: str):
        """
        Creates new node for an edge or assigns it to an existing node.
        :param edge_id: edge ID
        :param lanelet_ids: list of lanelet ids
        :param node_role: 'from' or 'to'
        :return:
        """
        assert node_role == "from" or node_role == "to"

        if node_role == "from":
            index = 0
            if edge_id in self._start_nodes:
                # already assigned to a node, see @REFERENCE_1
                return
        else:
            index = -1
            if edge_id in self._end_nodes:
                return

        conn_edges = set()
        lanelets = []
        for l_id in lanelet_ids:
            lanelet_tmp = self._lanelet_network.find_lanelet_by_id(l_id)
            lanelets.append(lanelet_tmp)
            if lanelet_tmp is not None:
                if node_role == "to":
                    conn_lanelet = lanelet_tmp.successor
                else:
                    conn_lanelet = lanelet_tmp.predecessor

                if conn_lanelet is not None:
                    try:
                        [
                            conn_edges.add(self.lanelet_id2edge_id[succ])
                            for succ in conn_lanelet
                        ]
                    except KeyError as exp:
                        raise RuntimeError(
                            f"The lanelet network is inconsistent in scenario {self._scenario.scenario_id}, "
                            f"there is a problem with adjacency of the lanelet {exp}"
                        )

        if node_role == "from":
            node_type = determine_start_node_type_for_lanelets(
                self._lanelet_network, lanelets
            )
        else:
            node_type = determine_end_node_type_for_lanelets(
                self._lanelet_network, lanelets
            )

        if len(conn_edges) > 0:
            node_candidates = []
            if node_role == "from":
                node_list_other = self._end_nodes
            else:
                node_list_other = self._start_nodes

            # check if connected edges already have a start/end node
            for to_edg in conn_edges:
                if to_edg in node_list_other:
                    node_candidates.append(node_list_other[to_edg])

            # check: connected edges should already use the same node
            assert len(set(node_candidates)) <= 1, "Unexpected error, please report!"
            if node_candidates:
                # assign existing node
                if node_role == "from":
                    self._start_nodes[edge_id] = node_candidates[0]
                else:
                    self._end_nodes[edge_id] = node_candidates[0]
            else:
                # create new node
                coords = self._compute_node_coords(lanelets, index=index)
                self.nodes[self.node_id_next] = Node(
                    self.node_id_next,
                    node_type,
                    coords,
                    right_of_way=RightOfWay.DEFAULT,
                )
                # @REFERENCE_1
                if node_role == "from":
                    self._start_nodes[edge_id] = self.node_id_next
                    for conn_edg in conn_edges:
                        self._end_nodes[conn_edg] = self.node_id_next
                else:
                    self._end_nodes[edge_id] = self.node_id_next
                    for conn_edg in conn_edges:
                        self._start_nodes[conn_edg] = self.node_id_next

                self.node_id_next += 1
        else:
            # dead end
            coords = self._compute_node_coords(lanelets, index=index)
            self.nodes[self.node_id_next] = Node(
                self.node_id_next,
                node_type,
                coords,
                right_of_way=RightOfWay.DEFAULT,
            )
            if node_role == "from":
                self._start_nodes[edge_id] = self.node_id_next
            else:
                self._end_nodes[edge_id] = self.node_id_next

            self.node_id_next += 1

    def _stop_line_end_offset(
        self, lanelets: Iterable[Lanelet], end_node: Node
    ) -> Optional[float]:
        """
        Computes the end_offset parameter, modelling the stop line for some lanelets
        :param lanelets: Lanelets composing an edge, to compute the end_offset from
        :param end_node: end node of the edge composed by lanelets.
        If the stop_lines LineMarking is SOLID or BORAD_SOLID, this node's type is set
        to ALLWAY_STOP
        :return: Optional end_offset if the given lanelets define a least one stop_line
        """
        # compute edge end_offset from composing lanelets
        projections: List[float] = []
        lengths: List[float] = []
        min_lengths: List[float] = []
        for lanelet in lanelets:
            # if no stop sign defined, or the stop sign has no lane marking, ignore it
            if not lanelet.stop_line or (
                lanelet.stop_line
                and lanelet.stop_line.line_marking == LineMarking.NO_MARKING
            ):
                continue
            if lanelet.stop_line.line_marking in {
                LineMarking.SOLID,
                LineMarking.BROAD_SOLID,
            }:
                end_node.type = NodeType.ALLWAY_STOP
            center_line = LineString(lanelet.center_vertices)
            min_distances = lanelet.inner_distance
            distances = lanelet.distance
            if lanelet.stop_line.start is None or lanelet.stop_line.end is None:
                projections.append(center_line.length)
                min_lengths.append(min_distances[-1])
                lengths.append(distances[-1])
                continue
            centroid = (lanelet.stop_line.start + lanelet.stop_line.end) / 2
            proj = center_line.project(Point(centroid))
            assert 0 <= proj <= center_line.length, (
                f"Stop Line for lanelet {lanelet.lanelet_id} has to be within"
                f"it's geometry. Remove stop line for lanelet "
                f"{lanelet.lanelet_id}"
                f"or change it's start and end position to fix this."
            )
            projections.append(proj)
            lengths.append(distances[-1])
            min_lengths.append(min_distances[-1])
        if lengths and projections:
            # end offset is the mean difference to the composing lanelet's lengths
            return min(
                0.0, max(min(min_lengths), np.mean(lengths) - np.mean(projections))
            )
        return None

    def _create_sumo_edges_and_lanes(self):
        """
        Creates edges for net file with previously collected edges and nodes.
        :return:
        """

        def calculate_lanelet_width_from_cr(lanelet: Lanelet) -> float:
            """
            Calculate the average width of a lanelet.
            :param lanelet: the lane whose width is to be calculated
            :return: average_width
            """
            distances = np.sqrt(
                np.sum((lanelet.left_vertices - lanelet.right_vertices) ** 2, axis=1)
            )
            return np.min(distances)

        for edge_id, lanelet_ids in self.lanes_dict.items():
            # Creation of Edge, using id as name
            start_node = self.nodes[self._start_nodes[edge_id]]
            end_node = self.nodes[self._end_nodes[edge_id]]
            lanelets = [
                self._lanelet_network.find_lanelet_by_id(lanelet_id)
                for lanelet_id in lanelet_ids
            ]

            # get edge type
            lanelet_types = [
                lanelet_type
                for lanelet in lanelets
                for lanelet_type in lanelet.lanelet_type
            ]
            edge_type = get_sumo_edge_type(
                self.edge_types, self._conf.country_id, *lanelet_types
            )

            edge = Edge(
                id=edge_id,
                from_node=start_node,
                to_node=end_node,
                type_id=edge_type.id,
                spread_type=SpreadType.CENTER,
                end_offset=self._stop_line_end_offset(lanelets, end_node),
            )

            speed_limit = 0
            self.edges[edge_id] = edge
            if self._conf.overwrite_speed_limit:
                speed_limit = self._conf.overwrite_speed_limit
            # else:
            #     speed_limit = self._traffic_sign_interpreter.speed_limit(frozenset([lanelet.lanelet_id]))
            #     if speed_limit is None or np.isinf(speed_limit):
            #         speed_limit = self.conf.unrestricted_speed_limit_default

            for lanelet_id in lanelet_ids:
                shape = self._points_dict[lanelet_id]
                lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
                lanelet_width = calculate_lanelet_width_from_cr(lanelet)

                lane = Lane(
                    edge,
                    speed=speed_limit,
                    length=self.edge_lengths[edge_id],
                    width=lanelet_width,
                    shape=shape,
                )
                self.lanes[lane.id] = lane
                self.lane_id2lanelet_id[lane.id] = lanelet_id
                self.lanelet_id2lane_id[lanelet_id] = lane.id

        # set oncoming lanes
        for edge_id, lanelet_ids in self.lanes_dict.items():
            leftmost_lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_ids[-1])
            if leftmost_lanelet.adj_left is not None:
                self.lanes[
                    self.lanelet_id2lane_id[lanelet_ids[-1]]
                ].setAdjacentOpposite(
                    self.lanelet_id2lane_id[leftmost_lanelet.adj_left]
                )

        for edge in self.edges.values():
            for e in edge.to_node.outgoing:
                edge.add_outgoing(e)
            for e in edge.from_node.incoming:
                edge.add_incoming(e)

    def _init_connections(self):
        """
        Init connections, doesn't consider junctions yet.
        :return:
        """
        for la in self._lanelet_network.lanelets:
            if la.successor:
                self._connections[self.lanelet_id2lane_id[la.lanelet_id]] += [
                    self.lanelet_id2lane_id[succ] for succ in la.successor
                ]

    def _encode_traffic_signs(self):
        """
        Encodes all traffic signs and writes the according changes to relevant nodes / edges
        :return:
        """
        encoder = TrafficSignEncoder(self.edge_types)
        traffic_signs: Dict[int, TrafficSign] = {
            t.traffic_sign_id: t for t in self._lanelet_network.traffic_signs
        }
        for lanelet in self._lanelet_network.lanelets:
            if not lanelet.traffic_signs:
                continue
            edge_id = self.lanelet_id2edge_id[lanelet.lanelet_id]
            if edge_id not in self.new_edges:
                _LOGGER.warning(
                    f"Merged Edge {edge_id} with traffic signs {lanelet.traffic_signs}. "
                    f"These Traffic signs will not be converted."
                )
                continue
            edge = self.new_edges[edge_id]
            for traffic_sign_id in lanelet.traffic_signs:
                traffic_sign = traffic_signs[traffic_sign_id]
                encoder.apply(traffic_sign, edge)
        encoder.encode()

    def _merge_junctions_intersecting_lanelets(self):
        """
        Merge nodes when their connecting edges intersect.
        :return:
        """
        # new dictionary for the merged nodes
        new_nodes: Dict[int, Node] = self.nodes.copy()
        # key is the merged node, value is a list of the nodes that form the merged node
        merged_dictionary: Dict[int, Set[Node]] = {}
        replaced_nodes: Dict[int, List[int]] = defaultdict(list)

        # compute dict with all intersecting lanelets of each lanelet based on their shapes
        intersecting_edges: Dict[int, Set[int]] = defaultdict(set)
        for pair in _find_intersecting_edges(self.lanes_dict, self._lanelet_network):
            intersecting_edges[pair[0]].add(pair[1])
            intersecting_edges[pair[1]].add(pair[0])

        # INTERSECTION BASED CLUSTERING
        # create clusters of nodes belonging to intersection elements fomr lanelet network
        def cluster_lanelets_from_intersection(
            lanelet_network, intersecting_edges
        ) -> Tuple[
            Dict[int, Set[Node]], Dict[int, Set[Lanelet]], Dict[int, Set[NodeType]]
        ]:
            clusters: Dict[int, Set[Node]] = defaultdict(set)
            cluster_types: Dict[int, Set[NodeType]] = defaultdict(set)
            next_cluster_id = 0
            # crossings are additional info for a cluster
            clusters_crossing: Dict[int, Set[Lanelet]] = defaultdict(set)
            # collect intersections that are deleted afterwards
            delete_intersections = []
            for intersection in lanelet_network.intersections:
                if len(intersection.incomings) < 2:
                    # some maps model road forks using intersection elements,
                    # however sumo doesn't need a junction for this
                    continue
                cluster_instruction = self.get_cluster_instruction(
                    intersection, lanelet_network, intersecting_edges
                )
                if cluster_instruction == ClusterInstruction.NO_CLUSTERING:
                    continue

                _LOGGER.debug(
                    "Clustering intersection %s according to %s",
                    intersection.intersection_id,
                    cluster_instruction,
                )
                intersect_any = False
                for incoming in intersection.incomings:
                    intersecting_lanelets = {
                        lanelet_id
                        for inc_tmp in intersection.incomings
                        for lanelet_id in inc_tmp.successors_right
                        | inc_tmp.successors_left
                        | inc_tmp.successors_straight
                        if incoming.incoming_id != inc_tmp.incoming_id
                    }
                    intersecting_lanelets -= incoming.incoming_lanelets
                    intersection_edges_others: List[Edge] = list(
                        self.edges[self.lanelet_id2edge_id[step]]
                        for step in intersecting_lanelets
                    )
                    out_lanelets_self = {
                        lanelet_id
                        for lanelet_id in incoming.successors_right
                        | incoming.successors_left
                        | incoming.successors_straight
                    }
                    out_edges_self: List[Edge] = list(
                        self.edges[self.lanelet_id2edge_id[step]]
                        for step in out_lanelets_self
                    )
                    intersection_edges_others = list(
                        set(intersection_edges_others) - set(out_edges_self)
                    )
                    # check whether any lanelets of the intersection actually intersect,
                    # else remove the whole cluster
                    intersect = False
                    for e1 in out_edges_self:
                        for e2 in intersection_edges_others:
                            if e2.id in intersecting_edges[e1.id]:
                                intersect = intersect_any = True
                                # delete_intersections.append(intersection.intersection_id)
                                break
                        if intersect:
                            break
                    if intersect is False:
                        for inc2 in intersection.incomings:
                            if inc2.left_of == incoming.incoming_id:
                                inc2.left_of = incoming.left_of
                        del intersection._incomings[
                            intersection._incomings.index(incoming)
                        ]

                if intersect_any is False:
                    continue

                if len(intersection.incomings) <= 1:
                    delete_intersections.append(intersection.intersection_id)

                # get all edges of intersection
                intersecting_lanelets = {
                    lanelet_id
                    for inc_tmp in intersection.incomings
                    for lanelet_id in inc_tmp.successors_right
                    | inc_tmp.successors_left
                    | inc_tmp.successors_straight
                }
                incoming_lanelets = {
                    lanelet_id
                    for incoming in intersection.incomings
                    for lanelet_id in incoming.incoming_lanelets
                }
                intersecting_lanelets -= incoming_lanelets
                intersection_edges: List[Edge] = list(
                    self.edges[self.lanelet_id2edge_id[step]]
                    for step in intersecting_lanelets
                )

                clusters[next_cluster_id] = {
                    node
                    for e in intersection_edges
                    for node in [e.from_node, e.to_node]
                }
                if cluster_instruction == ClusterInstruction.ZIPPER:
                    cluster_types[next_cluster_id].add(NodeType.ZIPPER)
                else:
                    cluster_types[next_cluster_id].add(NodeType.PRIORITY)

                # generate partial Crossings
                clusters_crossing[next_cluster_id] = {
                    self._lanelet_network.find_lanelet_by_id(lanelet_id)
                    for lanelet_id in intersection.crossings
                }
                next_cluster_id += 1

            return clusters, clusters_crossing, cluster_types, next_cluster_id

        clusters, clusters_crossing, cluster_types, next_cluster_id = (
            cluster_lanelets_from_intersection(
                self._lanelet_network, intersecting_edges
            )
        )

        # merge overlapping clusters
        while True:
            try:
                a_id, b_id = next(
                    (a_id, b_id)
                    for a_id, a in clusters.items()
                    for b_id, b in clusters.items()
                    if a_id < b_id and a & b
                )
                clusters[a_id] |= clusters[b_id]
                del clusters[b_id]
                clusters_crossing[a_id] |= clusters_crossing[b_id]
                del clusters_crossing[b_id]
            except StopIteration:
                break

        # collect lanelet ids that ´prohibit´ other lanelets in terms of SUMO's definition
        # (i.e. that have higher priority to pass an intersection)
        # for intersection in self.lanelet_network.intersections:
        #     inc_id2incoming_element = {inc.incoming_id: inc for inc in intersection.incomings}
        #     for inc in intersection.incomings:
        #         if inc.left_of is not None:
        #             if inc.left_of in inc_id2incoming_element:
        #                 for left_of_id in inc_id2incoming_element[inc.left_of].incoming_lanelets:
        #                     self.prohibits[self.lanelet_id2lane_id[left_of_id]].extend(
        #                         [self.lanelet_id2lane_id[l]for l in inc.incoming_lanelets])
        #             else:
        #                 warnings.warn(f"ID {inc.left_of} of left_of not among incomings"
        #                               f"{list(inc_id2incoming_element.keys())} of intersection"
        #                               f"{intersection.intersection_id}"
        #                               f"-> bug in lanelet_network of CommonRoad xml file!")

        # TODO: apply based on intersections
        if not lanelets_are_highway(self._lanelet_network.lanelets):
            # Expand merged clusters by all lanelets intersecting each other.
            # merging based on Lanelets intersecting
            explored_nodes = set()
            for current_node in self.nodes.values():
                clusters_flat = {n for cluster in clusters.values() for n in cluster}
                if current_node in explored_nodes | clusters_flat:
                    continue
                queue = [current_node]
                try:
                    current_cluster_id = next(
                        cluster_id
                        for cluster_id, cluster in clusters.items()
                        if current_node in cluster
                    )
                except StopIteration:
                    current_cluster_id = next_cluster_id
                    next_cluster_id += 1

                current_cluster = clusters[current_cluster_id]
                # delete current_cluster from dict
                if current_cluster:
                    clusters[current_cluster_id] = set()
                    queue = list(current_cluster)

                while queue:
                    expanded_node = queue.pop()
                    if expanded_node in explored_nodes:
                        continue
                    explored_nodes.add(expanded_node)

                    incomings = {e.id for e in expanded_node.incoming}
                    outgoings = {e.id for e in expanded_node.outgoing}
                    neighbor_nodes = {
                        node
                        for edge_id in outgoings | incomings
                        for intersecting in intersecting_edges[edge_id]
                        for node in [
                            self.edges[intersecting].from_node,
                            self.edges[intersecting].to_node,
                        ]
                    }
                    neighbor_nodes -= clusters_flat
                    queue += list(neighbor_nodes)
                    current_cluster |= neighbor_nodes

                clusters[current_cluster_id] = current_cluster

            # filter clusters with 0 nodes
            clusters = {
                cluster_id: cluster
                for cluster_id, cluster in clusters.items()
                if len(cluster) > 1
            }

        # MERGE COMPUTED CLUSTERS
        for cluster_id, cluster in clusters.items():
            cluster_type = cluster_types[cluster_id]
            _LOGGER.debug(f"Merging nodes: {[n.id for n in cluster]}")

            # create new merged node
            def merge_cluster(
                cluster: Set[Node], cluster_type: Set[NodeType] = {NodeType.PRIORITY}
            ) -> Node:
                cluster_ids = {n.id for n in cluster}
                merged_node = Node(
                    id=self.node_id_next,
                    node_type=NodeType.ZIPPER
                    if NodeType.ZIPPER in cluster_type
                    else NodeType.PRIORITY,
                    coord=np.mean([node.coord for node in cluster], axis=0),
                    right_of_way=RightOfWay.EDGE_PRIORITY,
                )
                self.node_id_next += 1
                new_nodes[merged_node.id] = merged_node
                for old_node_id in cluster_ids:
                    assert (
                        old_node_id not in replaced_nodes
                    ), f"{old_node_id} in {list(replaced_nodes.keys())}"
                    replaced_nodes[old_node_id].append(merged_node.id)
                merged_dictionary[merged_node.id] = cluster_ids
                return merged_node

            # clustered nodes at the border of a network need to be merged
            # separately, so junctions at network boundaries are converted correctly
            no_outgoing = {node for node in cluster if not node.outgoing}
            no_incoming = {node for node in cluster if not node.incoming}
            inner_cluster = cluster - no_outgoing - no_incoming
            if inner_cluster:
                merged_node = merge_cluster(inner_cluster, cluster_type)
                # Make crossing lanelets globally available
                if cluster_id in clusters_crossing:
                    self._crossings[merged_node.id] = clusters_crossing[cluster_id]
            if no_outgoing:
                merge_cluster(no_outgoing)
            if no_incoming:
                merge_cluster(no_incoming)

        replace_nodes_old = deepcopy(replaced_nodes)
        explored_nodes_all = set()
        for old_node, new_nodes_tmp in replace_nodes_old.items():
            if old_node in explored_nodes_all:
                continue

            if len(new_nodes_tmp) > 1:
                new_candidates = deepcopy(new_nodes_tmp)
                new_node = new_nodes_tmp[0]
                to_merge = set()
                explored_candidates = set()

                while new_candidates:
                    # merge with merged junction
                    new_node_tmp = new_candidates.pop()
                    if new_node_tmp in explored_candidates:
                        continue
                    explored_candidates.add(new_node_tmp)
                    to_merge |= merged_dictionary[new_node_tmp]
                    for merged_node in merged_dictionary[new_node_tmp]:
                        if len(replaced_nodes[merged_node]) > 1:
                            new_candidates = list(
                                set(new_candidates + replaced_nodes[merged_node])
                                - explored_candidates
                            )

                for node_id in explored_candidates:
                    del merged_dictionary[node_id]
                    if node_id != new_node:
                        del new_nodes[node_id]

                merged_dictionary[new_node] = to_merge
                explored_nodes_all |= to_merge
                for merged_node in to_merge:
                    replaced_nodes[merged_node] = [new_node]

        return new_nodes, merged_dictionary, replaced_nodes

    def _filter_edges(self, merged_dictionary, replaced_nodes):
        """
        Remove edges that lie inside a junction. Those will be replaced by internal edges
        :return: nothing
        """
        # new dictionary for the edges after deleting internal edges of junctions
        new_edges: Dict[int, Edge] = {}
        for edge in self.edges.values():
            if self._is_merged_edge(edge, merged_dictionary):
                continue

            edge_id = edge.id
            start_id = edge.from_node.id
            end_id = edge.to_node.id

            # update merged edges to from/to the merged node
            for new_node_id, merged_nodes in merged_dictionary.items():
                if start_id in merged_nodes:
                    edge.from_node = self.new_nodes[new_node_id]
                    break

            for new_node_id, merged_nodes in merged_dictionary.items():
                if end_id in merged_nodes:
                    edge.to_node = self.new_nodes[new_node_id]
                    break

            new_edges[edge_id] = edge

        return new_edges

    def _create_lane_based_connections(self):
        """
        Instantiate a new dictionary with only the connections that are meaningful after the simplification of the net
        :return: nothing
        """
        edge_ids = [edge.id for edge in self.new_edges.values()]
        for from_lane, connections in self._connections.copy().items():
            if int(from_lane.split("_")[0]) not in edge_ids:
                continue
            explored_lanes = set()
            queue = [[via] for via in connections]  # list with edge ids to toLane
            paths = []
            # explore paths until successor not inside junction anymore
            while queue:
                current_path = queue.pop()
                succ_lane = current_path[-1]
                explored_lanes.add(succ_lane)
                if int(succ_lane.split("_")[0]) not in edge_ids:
                    for next_lane in self._connections[succ_lane]:
                        if next_lane not in explored_lanes:
                            queue.append(current_path + [next_lane])
                else:
                    paths.append(current_path)

            for path in paths:
                if len(path) > 1:
                    shape = np.vstack(
                        [
                            self._points_dict[self.lane_id2lanelet_id[lane_id]]
                            for lane_id in path[:-1]
                        ]
                    )
                    via = path[0:-1]
                else:
                    shape = None
                    via = None

                if self._conf.highway_mode is True:
                    keep_clear = False
                else:
                    keep_clear = True

                no_connection = False
                not_keep_clear_types = {
                    LaneletType.ACCESS_RAMP,
                    LaneletType.INTERSTATE,
                    LaneletType.EXIT_RAMP,
                }
                keep_clear_types = {LaneletType.INTERSECTION}
                for lane_id in [from_lane] + path:
                    if (
                        len(
                            not_keep_clear_types
                            & self._lanelet_network.find_lanelet_by_id(
                                self.lane_id2lanelet_id[lane_id]
                            ).lanelet_type
                        )
                        > 0
                    ):
                        keep_clear = False
                    if (
                        len(
                            keep_clear_types
                            & self._lanelet_network.find_lanelet_by_id(
                                self.lane_id2lanelet_id[lane_id]
                            ).lanelet_type
                        )
                        > 0
                    ):
                        keep_clear = True

                # don't connect on-ramps to successor -> enforces lane change instead of driving straight
                lanelet_from = self._lanelet_network.find_lanelet_by_id(
                    self.lane_id2lanelet_id[from_lane]
                )
                if (
                    LaneletType.ACCESS_RAMP in lanelet_from.lanelet_type
                    and lanelet_from.adj_left is not None
                ):
                    no_connection = True

                # if no_connection is False:
                connection = Connection(
                    from_edge=self.new_edges[int(from_lane.split("_")[0])],
                    to_edge=self.new_edges[int(path[-1].split("_")[0])],
                    from_lane=self.lanes[from_lane],
                    to_lane=self.lanes[path[-1]],
                    via_lane_id=via,
                    shape=shape,
                    keep_clear=keep_clear,
                    cont_pos=self._conf.wait_pos_internal_junctions,
                    forbidden=no_connection,
                )

                # lanes changes to access ramps are forbidden
                # lanelet = self.lanelet_network.find_lanelet_by_id(self.lane_id2lanelet_id[from_lane])
                # if lanelet.adj_right is not None and LaneletType.ACCESS_RAMP \
                #         in self.lanelet_network.find_lanelet_by_id(lanelet.adj_right).lanelet_type:
                #     connection.change_right_allowed = {}
                self._new_connections.add(connection)

    def _create_roundabouts(self, driving_direction: str = "right"):
        if driving_direction == "left":
            inner_direction = "adj_right"
        elif driving_direction == "right":
            inner_direction = "adj_left"
        else:
            raise ValueError

        def find_inner_lanelet_cycles(
            lanelet_network: LaneletNetwork, max_length: float = 180.0
        ) -> List[int]:
            lanelets = lanelet_network._lanelets
            # candidiates are lanelets without adj_{left/right}
            length_half = max_length * 0.5
            queue = [
                la
                for la, lanelet in lanelets.items()
                if not getattr(lanelet, inner_direction)
                and lanelet.distance[-1] < length_half
            ]
            G = nx.DiGraph()
            for la in queue:
                [G.add_edge(la, succ) for succ in lanelets[la].successor]

            cycles = list(nx.simple_cycles(G))
            for cycle in reversed(cycles):
                length = 0
                for la in cycle:
                    length += lanelets[la].inner_distance[-1]
                    if length > max_length:
                        del cycles[cycles.index(cycle)]
                        break

            return cycles

        def lanelet_cycles_2_edge_cycles(
            lanelet_cycles: List[int], lanelet_id2edge_id
        ) -> List[Roundabout]:
            roundabouts = []
            for cycle in lanelet_cycles:
                edge_cycle = list(
                    dict.fromkeys([lanelet_id2edge_id[la] for la in cycle])
                )
                roundabouts.append(
                    Roundabout(
                        [self.new_edges[e] for e in edge_cycle if e in self.new_edges]
                    )
                )

            return roundabouts

        lanelet_cycles = find_inner_lanelet_cycles(self._lanelet_network)
        roundabouts = lanelet_cycles_2_edge_cycles(
            lanelet_cycles, self.lanelet_id2edge_id
        )
        self._set_roundabout_nodes_keep_clear(roundabouts, self.new_nodes)
        return roundabouts

    def _set_roundabout_nodes_keep_clear(
        self, roundabouts: List[Roundabout], nodes: Dict[int, Node]
    ):
        roundabout_nodes = set(
            itertools.chain.from_iterable(
                [e.to_node.id, e.from_node.id] for r in roundabouts for e in r.edges
            )
        )
        for node_id in roundabout_nodes:
            nodes[node_id].keep_clear = False

    def _set_prohibited_connections(self):
        """Add connections that are prohibited by a connection."""
        edges2connection = {c.from_lane.id: c for c in self._new_connections}
        edges2connection.update(
            {via: c for c in self._new_connections for via in c.via}
        )
        for connection in self._new_connections:
            prohibited_lanes = [
                edges2connection[p] for p in self.prohibits[connection.from_lane.id]
            ]
            prohibited_lanes.extend(
                [
                    edges2connection[p]
                    for via_lane in connection.via
                    for p in self.prohibits[via_lane]
                    if p in edges2connection
                ]
            )
            connection.prohibits = prohibited_lanes
        return True

    def _create_crossings(self):
        new_crossings = dict()
        for merged_node_id, crossing_lanelets in self._crossings.items():
            if not crossing_lanelets:
                continue
            merged_node = self.new_nodes[merged_node_id]
            adjacent_edges = {
                edge
                for edge in self.new_edges.values()
                if edge.from_node == merged_node or edge.to_node == merged_node
            }
            pedestrian_edges = {
                edge
                for edge in adjacent_edges
                if VehicleType.PEDESTRIAN in self.edge_types.types[edge.type_id].allow
            }
            non_pedestrian_edges = adjacent_edges - pedestrian_edges

            if not non_pedestrian_edges:
                continue

            # set any nodes referencing a crossing to PRIORITY_STOP
            # this forces lanes with low priority (e.g. the crossing)
            # to wait for lanes with priority (cars)
            merged_node.type = NodeType.PRIORITY_STOP

            clusters: List[Set[Edge]] = min_cluster(
                non_pedestrian_edges,
                lambda dist: dist < 4,
                lambda e1, e2: np.min(
                    [
                        np.linalg.norm(pt1 - pt2)
                        for lane1 in e1.lanes
                        for lane2 in e2.lanes
                        for pt1 in np.array(lane1.shape)
                        for pt2 in np.array(lane2.shape)
                    ]
                ),
            )
            crossing_lanelets = merge_lanelets(crossing_lanelets)

            crossings: List[Crossing] = []
            for edges in clusters:
                common_node: Node = reduce(
                    lambda a, b: a & b,
                    [{edge.from_node, edge.to_node} for edge in edges],
                ).pop()
                assert common_node, "Edges in one cluster have to share a common node"
                # find vertices closest to the common node
                pts = np.array(
                    [
                        vtx
                        for edge in edges
                        for lane in edge.lanes
                        for vtx in [
                            lane.shape[-1]
                            if edge.to_node == common_node
                            else lane.shape[0]
                        ]
                    ]
                )
                center = np.mean(pts, axis=0)

                lanelet = crossing_lanelets[
                    int(
                        np.argmin(
                            [
                                np.min(
                                    np.linalg.norm(
                                        lanelet.center_vertices - center, axis=1
                                    )
                                )
                                for lanelet in crossing_lanelets
                            ]
                        )
                    )
                ]
                shape = lanelet.center_vertices
                c = Crossing(
                    node=merged_node,
                    edges=edges,
                    shape=shape,
                    width=float(
                        np.median(
                            np.linalg.norm(
                                lanelet.left_vertices - lanelet.right_vertices, axis=1
                            )
                        )
                    ),
                )
                crossings.append(c)

            new_crossings[merged_node_id] = crossings
        self._crossings = new_crossings

    def _create_traffic_lights(self):
        cr_traffic_lights: Dict[int, TrafficLight] = (
            self._lanelet_network._traffic_lights
        )
        node_2_traffic_light: Dict[Node, Set[TrafficLight]] = defaultdict(set)
        node_2_connections: Dict[Node, Set[Connection]] = defaultdict(set)
        light_2_connections: Dict[TrafficLight, Set[Connection]] = defaultdict(set)
        incoming_lanelet_2_intersection = (
            self._lanelet_network.map_inc_lanelets_to_intersections
        )
        for lanelet in self._lanelet_network.lanelets:
            if not lanelet.traffic_lights:
                continue
            lights: Set[TrafficLight] = {
                cr_traffic_lights[tl]
                for tl in lanelet.traffic_lights
                if cr_traffic_lights[tl].active
            }
            edge_id = self.lanelet_id2edge_id[lanelet.lanelet_id]
            if edge_id not in self.new_edges:
                _LOGGER.warning(
                    f"Edge: {edge_id} has been removed in SUMO-NET but contained a traffic light"
                )
                continue
            edge = self.new_edges[edge_id]
            node: Node = edge.to_node
            node_2_traffic_light[node] |= lights

            # maps each succeeding lanelet to the angle (in radians, [-pi, pi]) it forms with the preceding one
            # successors: Dict[Connection, float] = dict()
            #
            intersection = (
                incoming_lanelet_2_intersection[lanelet.lanelet_id]
                if lanelet.lanelet_id in incoming_lanelet_2_intersection
                else None
            )

            def calc_direction_2_connections(edge: Edge):
                connections_init = {}
                if intersection is not None:
                    incoming_elem = intersection.map_incoming_lanelets[
                        lanelet.lanelet_id
                    ]
                    connections_init[TrafficLightDirection.STRAIGHT] = (
                        incoming_elem.successors_straight
                    )
                    connections_init[TrafficLightDirection.LEFT] = (
                        incoming_elem.successors_left
                    )
                    connections_init[TrafficLightDirection.RIGHT] = (
                        incoming_elem.successors_right
                    )
                elif len(lanelet.successor) == 1:
                    connections_init[TrafficLightDirection.STRAIGHT] = set(
                        lanelet.successor
                    )
                    connections_init[TrafficLightDirection.LEFT] = set(
                        lanelet.successor
                    )
                    connections_init[TrafficLightDirection.RIGHT] = set(
                        lanelet.successor
                    )
                else:
                    raise ValueError

                connections = defaultdict(set)
                for direction, init_queue in connections_init.items():
                    queue = [
                        self.edges[self.lanelet_id2edge_id[la]] for la in init_queue
                    ]
                    visited = set()
                    while queue:
                        current = queue.pop()
                        if current in visited:
                            continue
                        visited.add(current)
                        if current.id in self.new_edges:
                            connections[direction] |= set(
                                c
                                for c in self._new_connections
                                if c.from_edge == edge and c.to_edge == current
                            )
                            continue
                        queue += current.outgoing
                return dict(connections)

            # successor_edges = succeeding_new_edges(edge)
            direction_2_connections = calc_direction_2_connections(edge)

            for light in lights:
                # try:
                direction = light.direction
                if (
                    len(lanelet.successor) == 1
                    or not light.direction
                    or light.direction == TrafficLightDirection.ALL
                ):
                    connections = set(
                        itertools.chain.from_iterable(direction_2_connections.values())
                    )
                elif intersection is not None:
                    connections = set()
                    if direction in (
                        TrafficLightDirection.RIGHT,
                        TrafficLightDirection.LEFT_RIGHT,
                        TrafficLightDirection.STRAIGHT_RIGHT,
                    ):
                        connections |= direction_2_connections[
                            TrafficLightDirection.RIGHT
                        ]
                    if direction in (
                        TrafficLightDirection.LEFT,
                        TrafficLightDirection.LEFT_RIGHT,
                        TrafficLightDirection.LEFT_STRAIGHT,
                    ):
                        connections |= direction_2_connections[
                            TrafficLightDirection.LEFT
                        ]
                    if direction in (
                        TrafficLightDirection.STRAIGHT,
                        TrafficLightDirection.STRAIGHT_RIGHT,
                        TrafficLightDirection.LEFT_STRAIGHT,
                    ):
                        connections |= direction_2_connections[
                            TrafficLightDirection.STRAIGHT
                        ]
                elif len(lanelet.successor) == 1:
                    connections = set()
                else:
                    warnings.warn("Conenctions for traffic light cannot be computed")
                    node_2_traffic_light[node].remove(light)
                    continue
                node_2_connections[node] |= connections
                light_2_connections[light] |= connections
                # except KeyError:
                #     _LOGGER.exception(f"Unknown TrafficLightDirection: {light.direction}, "
                #                       f"could not add successors for lanelet {lanelet}")

        light_2_connections = dict(light_2_connections)
        # generate traffic lights in SUMO format
        encoder = TrafficLightEncoder(self._scenario.dt)
        for to_node, lights in node_2_traffic_light.items():
            try:
                program, connections = encoder.encode(
                    to_node, list(lights), light_2_connections
                )
                self.traffic_light_signals.add_program(program)
                for connection in connections:
                    self.traffic_light_signals.add_connection(connection)
            except (RuntimeError, ValueError, TypeError):
                continue

    def _is_merged_edge(self, edge: Edge, merged_dictionary):
        """
        returns True if the edge must be removed, False otherwise
        :param edge: the edge to consider
        :return: flag remove_edge
        """
        start_node_id = edge.from_node.id
        end_node_id = edge.to_node.id

        return any(
            start_node_id in merged_nodes and end_node_id in merged_nodes
            for merged_nodes in merged_dictionary.values()
        )

    def auto_generate_traffic_light_system(
        self,
        lanelet_id: int,
        green_time: int = 38,
        red_time: int = 12,
        yellow_time: int = 7,
        all_red_time: int = 0,
        left_green_time: int = 6,
        crossing_min_time: int = 4,
        crossing_clearance_time: int = 5,
        time_offset: int = 0,
    ) -> bool:
        """
        Automatically generate a Traffic Light System (TLS) for all lanelets
        in the same intersection as the given lanelet_id.
        The below has been partially adapted from: https://sumo.dlr.de/docs/netconvert.html#tls_building
        :param lanelet_id: ID of lanelet in intersection to generate traffic lights for
        :param green_time: Green phase duration. [s]
        :param yellow_time: Fixed time for yellow phase durations [s]
        :param red_time: Set INT as fixed time for red phase duration at traffic
        lights that do not have a conflicting flow [s]
        :param all_red_time: Fixed time for intermediate red phase after every switch [s].
        :param left_green_time: Green phase duration for left turns. Setting this value to 0
        disables additional left-turning phases [s].
        :param crossing_min_time: Minimum time duration for pedestrian crossings [s].
        :param crossing_clearance_time: Clearance time for pedestrian crossings [s].
        :param time_offset: Offset for start time of the generated traffic lights [s].
        :return: if the conversion was successful
        """
        assert green_time > 0
        assert yellow_time > 0
        assert all_red_time >= 0
        assert left_green_time > 0
        assert crossing_min_time > 0
        assert crossing_clearance_time > 0
        assert red_time >= 0

        if not self._output_file:
            _LOGGER.error("Need to call create_sumo_files first")
            return False

        # did the user select an incoming lanelet to the junction?
        if lanelet_id not in self.lanelet_id2junction:
            lanelet: Lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            if not lanelet:
                _LOGGER.warning(f"Unknown Lanelet: {lanelet_id}")
                return False
            # if the selected lanelet is not an incoming one, check the predecessors
            try:
                lanelet_id = next(
                    pred
                    for pred in lanelet.predecessor
                    if pred in self.lanelet_id2junction
                )
            except StopIteration:
                _LOGGER.warning(f"No junction found for lanelet {lanelet_id}")
                return False

        # does the lanelet already have a traffic light?
        # If so guess signals for them and copy the corresponding position
        lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
        guess_signals = bool(lanelet.traffic_lights)

        # auto generate the TLS with netconvert
        junction = self.lanelet_id2junction[lanelet_id]
        command = (
            f"netconvert"
            f" --sumo-net-file={self._output_file}"
            f" --output-file={self._output_file}"
            f" --tls.set={junction.id}"
            f" --tls.guess=true"
            f" --geometry.remove.keep-edges.explicit"
            f" --geometry.remove.min-length=0.0"
            f" --tls.guess-signals={'true' if guess_signals else 'false'}"
            f" --tls.group-signals=true"
            f" --tls.green.time={green_time}"
            f" --tls.red.time={red_time}"
            f" --tls.yellow.time={yellow_time}"
            f" --tls.allred.time={all_red_time}"
            f" --tls.left-green.time={left_green_time}"
            f" --tls.crossing-min.time={crossing_min_time}"
            f" --tls.crossing-clearance.time={crossing_clearance_time}"
        )
        try:
            out = subprocess.check_output(
                command.split(), timeout=5.0, stderr=subprocess.STDOUT
            )
            if "error" in str(out).lower():
                return False
        except Exception as e:
            _LOGGER.error("Encountered error while running netconvert %s", e)
            return False

        net = sumo_net_from_xml(self._output_file)
        self._update_junctions_from_net(net)
        junction = self.lanelet_id2junction[lanelet_id]

        # compute unused id value for the traffic lights
        next_cr_id = max_lanelet_network_id(self._lanelet_network) + 1

        # add generated Traffic Lights to the corresponding lanelets
        for connection in (
            conn
            for from_edges in net.connections.values()
            for from_lanes in from_edges.values()
            for to_edges in from_lanes.values()
            for conn in to_edges.values()
        ):
            # only add traffic lights the the connections at the current junction
            if connection.tls is None or connection.tls.id != str(junction.id):
                continue
            lanelet_id = self.lane_id2lanelet_id[connection.from_lane.id]
            lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            traffic_light = TrafficLight(
                traffic_light_id=next_cr_id,
                traffic_light_cycle=TrafficLightCycle(
                    [
                        TrafficLightCycleElement(
                            state=traffic_light_states_SUMO2CR[
                                phase.state[connection.tl_link]
                            ],
                            duration=round(phase.duration / self._scenario.dt),
                        )
                        for phase in connection.tls.phases
                    ],
                    time_offset=time_offset / self._scenario.dt,
                ),
                direction=directions_SUMO2CR[connection.direction],
                position=lanelet.right_vertices[-1],
            )
            next_cr_id += 1
            assert self._lanelet_network.add_traffic_light(
                traffic_light, {lanelet_id}
            ), f"Could not add traffic light to lanelet: {lanelet_id}"

        return True

    def write_intermediate_files(
        self, output_path: Optional[Path] = None
    ) -> SumoIntermediateProject:
        """
        Function for writing the edges and nodes files in xml format
        :param output_path: the relative path of the output
        :return: None
        """

        if output_path is None:
            sumo_intermediate_project = SumoIntermediateProject.create_temp(
                str(self._scenario.scenario_id)
            )
        else:
            sumo_intermediate_project = SumoIntermediateProject(
                str(self._scenario.scenario_id), output_path
            )

        self._add_nodes_file(sumo_intermediate_project)
        self._add_edges_file(sumo_intermediate_project)
        self._add_connections_file(sumo_intermediate_project)
        self._add_traffic_file(sumo_intermediate_project)
        self._add_edge_type_file(sumo_intermediate_project)

        sumo_intermediate_project.write()

        return sumo_intermediate_project

    def _add_nodes_file(
        self, sumo_intermediate_project: SumoIntermediateProject
    ) -> None:
        """
        Functio for writing the nodes file
        :param output_path: path for the file
        :return: nothing
        """
        sumo_nodes_file = sumo_intermediate_project.create_file(
            SumoIntermediateFileType.NODES
        )
        for node in self.new_nodes.values():
            sumo_nodes_file.add_node(node)

    def _add_edges_file(
        self, sumo_intermediate_project: SumoIntermediateProject
    ) -> None:
        """
        Function for writing the edges file
        :param output_path: path for the file
        :return: nothing
        """
        edges_file = sumo_intermediate_project.create_file(
            SumoIntermediateFileType.EDGES
        )
        for edge in self.new_edges.values():
            edges_file.add_node(edge)
        for roundabout in self.roundabouts:
            edges_file.add_node(roundabout)

    def _add_connections_file(
        self, sumo_intermediate_project: SumoIntermediateProject
    ) -> None:
        """
        Function for writing the connections file
        :param output_path: path for the file
        :return: nothing
        """
        connections_file = sumo_intermediate_project.create_file(
            SumoIntermediateFileType.CONNECTIONS
        )
        for connection in self._new_connections:
            connections_file.add_node(connection)

        for crossings in self._crossings.values():
            for crossing in crossings:
                connections_file.add_node(crossing)

    def _add_traffic_file(
        self, sumo_intermediate_project: SumoIntermediateProject
    ) -> None:
        """
        Writes the tll.net.xml file to disk
        :param output_path: path for the file
        """
        traffic_light_file = sumo_intermediate_project.create_file(
            SumoIntermediateFileType.TLLOGICS
        )
        traffic_light_file.add_node(self.traffic_light_signals)

    def _add_edge_type_file(
        self, sumo_intermediate_project: SumoIntermediateProject
    ) -> None:
        """
        Writes the tll.net.xml file to disk
        :param output_path: path for the file
        """

        edge_type_file = sumo_intermediate_project.create_file(
            SumoIntermediateFileType.TYPES
        )
        edge_type_file.add_node(self.edge_types)

    def merge_intermediate_files(
        self,
        sumo_intermediate_project: SumoIntermediateProject,
        cleanup: bool,
    ) -> Optional[SumoProject]:
        """
        Function that merges the edges and nodes files into one using netconvert
        :param output_path
        :param cleanup: deletes temporary input files after creating net file (only deactivate for debugging)
        :param connections_path:
        :param nodes_path:
        :param edges_path:
        :param traffic_path:
        :param type_path:
        :param output_path: the relative path of the output
        :return: bool: returns False if conversion fails
        """

        sumo_project = SumoProject.from_intermediate_sumo_project(
            sumo_intermediate_project
        )

        netconvert_result = execute_sumo_application(
            SumoApplication.NETCONVERT,
            [
                "--no-turnarounds=true",
                "--junctions.internal-link-detail=20",
                "--geometry.avoid-overlap=true",
                "--geometry.remove.keep-edges.explicit=true",
                "--geometry.remove.min-length=0.0",
                "--tls.guess-signals=true",
                "--tls.group-signals=true",
                f"--tls.green.time={50}",
                f"--tls.red.time={50}",
                f"--tls.yellow.time={10}",
                f"--tls.allred.time={50}",
                f"--tls.left-green.time={50}",
                f"--tls.crossing-min.time={50}",
                f"--tls.crossing-clearance.time={50}",
                "--offset.disable-normalization=true",
                f"--node-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.NODES)}",
                f"--edge-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.EDGES)}",
                f"--connection-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.CONNECTIONS)}",
                f"--tllogic-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.TLLOGICS)}",
                f"--type-files={sumo_intermediate_project.get_file_path(SumoIntermediateFileType.TYPES)}",
                f"--output-file={sumo_project.get_file_path(SumoFileType.NET)}",
                f"--seed={self._conf.random_seed}",
            ],
        )

        if netconvert_result is None:
            _LOGGER.error(
                "Failed to merge intermediate files: netconvert failed with an unknown error!"
            )
            return None

        # All warnings produced by netconvert are considered debug messages,
        # because they are usually rather informative
        # and do not affect the functionality of the simulation
        for line in netconvert_result.splitlines():
            if line.startswith("Warning"):
                warning_message = line.lstrip("Warning: ")
                _LOGGER.debug(
                    f"netconvert produced a warning while creating {self._output_file}: {warning_message}"
                )
            else:
                _LOGGER.debug("netconvert output: %s", line)

        if cleanup:
            sumo_intermediate_project.cleanup()

        return sumo_project

    def _update_junctions_from_net(self, net: Net):
        # parse junctions from .net.xml
        for junction in net.junctions.values():
            if junction.inc_lanes is None:
                continue
            for lane in junction.inc_lanes:
                # skip internal lanes
                if lane.id not in self.lane_id2lanelet_id:
                    continue
                self.lanelet_id2junction[self.lane_id2lanelet_id[lane.id]] = junction

    def create_sumo_files(
        self,
        output_folder: Optional[Path] = None,
        cleanup_tmp_files=True,
    ) -> Optional[SumoProject]:
        """
        Convert the CommonRoad scenario to a net.xml file, specified by the absolute path output_file and create
        all SUMO files required for the traffic simulation.
        :param output_folder of the returned SUMO files
        :param traffic_from_trajectories: if True, create route files based on trajectories from CommonRoad scenario;
            if False, create traffic randomly using SUMO's randomTrips script
        :param cleanup_tmp_files: clean up temporary files created during the .net conversion, useful for debugging
        :return returns whether conversion was successful
        """
        _LOGGER.debug(
            "Converting to CommonRoad lanelet network of scenario %s to SUMO in folder %s",
            self._scenario.scenario_id,
            output_folder,
        )
        self._convert_map()

        _LOGGER.debug("Merging Intermediate Files")

        sumo_intermediate_project = self.write_intermediate_files(output_folder)
        sumo_project = self.merge_intermediate_files(
            sumo_intermediate_project, cleanup=cleanup_tmp_files
        )

        if sumo_project is None:
            return None

        sumo_project.write()
        return sumo_project

    def get_cluster_instruction(
        self,
        intersection: Intersection,
        lanelet_network: LaneletNetwork,
        intersection_edges: Dict[int, List[int]],
    ):
        def get_all_adj(lanelet_id):
            adjacent_lanelets = set()
            if lanelet_network._lanelets[lanelet_id].adj_left is not None:
                adjacent_lanelets.add(lanelet_network._lanelets[lanelet_id].adj_left)
            if lanelet_network._lanelets[lanelet_id].adj_right is not None:
                adjacent_lanelets.add(lanelet_network._lanelets[lanelet_id].adj_right)

            return adjacent_lanelets

        def get_all_successors(lanelets: Set[int]) -> Set[int]:
            return set(
                itertools.chain.from_iterable(
                    set(lanelet_network.find_lanelet_by_id(la).successor)
                    for la in lanelets
                )
            )

        if is_highway_intersection(lanelet_network, intersection):
            zipper_return = ClusterInstruction.ZIPPER
        else:
            zipper_return = ClusterInstruction.CLUSTERING

        # check whether all successors are laterally adjacent at least successors
        # from two different incomings are adjacent-> choose zipper type
        successor_criterion = True
        s_types = ["successors_straight", "successors_left", "successors_right"]
        all_successors = set(
            itertools.chain.from_iterable(
                getattr(inc, s_typ)
                for inc in intersection.incomings
                for s_typ in s_types
            )
        )
        for s in all_successors:
            if not get_all_adj(s) & all_successors:
                successor_criterion = False
                break

        adj_other = False
        inc2successors = {
            inc.incoming_id: set(
                itertools.chain.from_iterable(getattr(inc, s_typ) for s_typ in s_types)
            )
            for inc in intersection.incomings
        }
        for inc_id, succ_tmp in inc2successors.items():
            for s in succ_tmp:
                adj = get_all_adj(s)
                for inc_id_other, succ_tmp2 in inc2successors.items():
                    if inc_id_other == inc_id:
                        continue
                    if adj & succ_tmp2:
                        adj_other = True
                        break

        if successor_criterion is True and adj_other is True:
            return zipper_return

        lanelet_2_inc = intersection.map_incoming_lanelets
        inc_lanelets = set(lanelet_2_inc.keys())
        inc_2_incoming_lanelets = defaultdict(list)
        for lanelet, incoming in lanelet_2_inc.items():
            inc_2_incoming_lanelets[incoming].append(lanelet)

        # check whether in all incoming at least one lanelet is adjacent to another incoming
        adjacency_criterion = (
            True  # is falsified if at least one incoming doesn't fulfil criterion
        )
        # check whether two incomings are merging
        zipper_criterion = False  # only needs to occur once
        for inc, inc_lanelets in inc_2_incoming_lanelets.items():
            is_adj_with_other_incoming = False
            for l_id in inc_lanelets:
                is_adj_with_other_incoming = False
                other_inc_lanelets = set(
                    itertools.chain.from_iterable(
                        [
                            incoming_lanelets
                            for inc_other, incoming_lanelets in inc_2_incoming_lanelets.items()
                            if inc != inc_other
                        ]
                    )
                )
                if set(get_all_adj(l_id)) & other_inc_lanelets:
                    is_adj_with_other_incoming = True
                if set(
                    lanelet_network.find_lanelet_by_id(l_id).successor
                ) & get_all_successors(other_inc_lanelets):
                    zipper_criterion = True

            if is_adj_with_other_incoming is False:
                adjacency_criterion = False

        # zipper with > 2 edges undefined
        if len(intersection.incomings) == 2:
            zipper_criterion = False

        if adjacency_criterion is True:
            if zipper_criterion is True:
                return zipper_return
            else:
                return ClusterInstruction.NO_CLUSTERING

        # check whether lanelets of intersection are even intersecting
        non_intersecting_criterion = True
        succ_edges = {self.lanelet_id2edge_id[s] for s in all_successors}
        for edge_id in succ_edges:
            if edge_id in intersection_edges:
                intersecting_edges_tmp = set(intersection_edges[edge_id]) & succ_edges
                for edge_tmp in intersecting_edges_tmp:
                    # intersections are not counted, if they occur between forking edges
                    if (
                        self.edges[edge_id].from_node.id
                        != self.edges[edge_tmp].from_node.id
                    ):
                        non_intersecting_criterion = False
                        break

        if non_intersecting_criterion is True:
            return ClusterInstruction.NO_CLUSTERING

        return ClusterInstruction.CLUSTERING

    def delete_traffic_light_if_no_intersection(self):
        """
        Delete traffic lights, if lanelet is not an incoming of an intersection.
        :return:
        """
        incoming_mapping = self._lanelet_network.map_inc_lanelets_to_intersections
        for l_id, lanelet in self._lanelet_network._lanelets.items():
            if (
                len(lanelet.traffic_lights) > 0
                and len(lanelet.successor) > 1
                and lanelet not in incoming_mapping
            ):
                lanelet._traffic_lights = set()

        self._lanelet_network.cleanup_traffic_lights()

    def draw_network(
        self, nodes: Dict[int, Node], edges: Dict[int, Edge], figsize=(20, 20)
    ):
        plt.figure(figsize=figsize)
        draw_params = {
            "lanelet": {"show_label": True},
            "intersection": {"draw_intersections": True, "show_label": True},
        }
        rnd = MPRenderer(draw_params=draw_params)
        self._lanelet_network.draw(rnd)
        rnd.render(show=False)
        G = nx.DiGraph()
        graph_nodes = list(nodes.keys())
        graph_nodes_pos = {node_id: node.coord for node_id, node in nodes.items()}
        for edge_id, edge in edges.items():
            G.add_edge(edge.from_node.id, edge.to_node.id, label=edge.id)
        G.add_nodes_from(graph_nodes)
        nodes = nx.draw_networkx_nodes(
            G, graph_nodes_pos, node_size=20, nodelist=graph_nodes
        )
        nx.draw_networkx_edges(G, graph_nodes_pos)
        nx.draw_networkx_labels(G, pos=graph_nodes_pos)
        nodes.set_zorder(800)
        # edges.set_zorder(800)

        nx.draw(G, graph_nodes_pos, with_labels=True)
        colors = itertools.cycle(set(mcolors.TABLEAU_COLORS) - {"tab:blue"})
        for cluster in self.merged_dictionary.values():
            coll = nx.draw_networkx_nodes(
                G, graph_nodes_pos, nodelist=cluster, node_color=next(colors)
            )
            labels = nx.draw_networkx_labels(G, pos=graph_nodes_pos)
            coll.set_zorder(900)
            # for pos

        for inter in self._lanelet_network.intersections:
            s = f"""\
            intersection {inter.intersection_id}:
            inc:{[inc.incoming_lanelets for inc in inter.incomings]}
            out:{
                list(
                    itertools.chain.from_iterable(
                        [
                            inc.successors_straight
                            | inc.successors_left
                            | inc.successors_right
                            for inc in inter.incomings
                        ]
                    )
                )
            }
            """
            pos = (
                self._lanelet_network.find_lanelet_by_id(
                    list(inter.incomings[0].incoming_lanelets)[0]
                )
                .center_vertices[-1]
                .flatten()
            )
            plt.text(x=pos[0], y=pos[1], s=s, zorder=1e4)
        labels = nx.get_edge_attributes(G, "label")
        nx.draw_networkx_edge_labels(G, pos=graph_nodes_pos, edge_labels=labels)
        plt.autoscale()
        plt.axis("equal")
        plt.show(block=True)
