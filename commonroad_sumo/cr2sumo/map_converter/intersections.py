import itertools
import logging
from typing import Dict, Iterable, List, Optional, Set

from commonroad.scenario.intersection import Intersection
from commonroad.scenario.lanelet import (
    Lanelet,
    LaneletNetwork,
    LaneletType,
    LineMarking,
)

from commonroad_sumo.cr2sumo.map_converter.mapping import ClusterInstruction
from commonroad_sumo.sumolib.net import NodeType

_LOGGER = logging.getLogger(__name__)


def get_cluster_instruction(
    intersection: Intersection,
    lanelet_network: LaneletNetwork,
    intersection_edges: Dict[int, List[int]],
) -> ClusterInstruction:
    successor_criterion = matches_successor_criterion(lanelet_network, intersection)
    adjacent_other_criterion = matches_adjacent_other_criterion(
        lanelet_network, intersection
    )
    if successor_criterion and adjacent_other_criterion:
        # cluster_instruction = get_cluster_instruction_for_zipper_intersection(
        # lanelet_network, intersection
        # )
        pass

    return ClusterInstruction.NO_CLUSTERING


def matches_successor_criterion(
    lanelet_network: LaneletNetwork, intersection: Intersection
) -> bool:
    all_successors_ids = _collect_successors_of_intersection(
        lanelet_network, intersection
    )

    successor_criterion = True
    for successor_id in all_successors_ids:
        adjacent_lanelets_in_successors_of_intersection = (
            len(
                _collect_all_adjacent_lanelet_ids(lanelet_network, successor_id)
                & all_successors_ids
            )
            > 0
        )
        if not adjacent_lanelets_in_successors_of_intersection:
            successor_criterion = False
            break

    return successor_criterion


def matches_adjacent_other_criterion(
    lanelet_network: LaneletNetwork, intersection: Intersection
) -> bool:
    # TODO: implementation?
    adj_other = False
    inc2successors = {
        inc.incoming_id: set(
            itertools.chain.from_iterable(getattr(inc, s_typ) for s_typ in [])  # type: ignore
        )
        for inc in intersection.incomings
    }
    for inc_id, succ_tmp in inc2successors.items():
        for s in succ_tmp:
            adj = _collect_all_adjacent_lanelet_ids(lanelet_network, s)
            for inc_id_other, succ_tmp2 in inc2successors.items():
                if inc_id_other == inc_id:
                    continue
                if adj & succ_tmp2:
                    adj_other = True
                    break

    return adj_other


def select_more_specific_node_type(
    node_type: NodeType, other_node_type: Optional[NodeType] = None
) -> NodeType:
    if other_node_type is None:
        return node_type

    node_type_priority_order = [
        NodeType.PRIORITY,
        NodeType.RIGHT_BEFORE_LEFT,
        NodeType.PRIORITY_STOP,
        NodeType.UNREGULATED,
        NodeType.ZIPPER,
        NodeType.ALLWAY_STOP,
        NodeType.TRAFFIC_LIGHT,
        NodeType.TRAFFIC_LIGHT_UNREGULATED,
        NodeType.TRAFFIC_LIGHT_RIGHT_ON_RED,
    ]
    priority = node_type_priority_order.index(node_type)
    other_priority = node_type_priority_order.index(other_node_type)

    if priority >= other_priority:
        return node_type
    else:
        return other_node_type


def determine_start_node_type_for_lanelets(
    lanelet_network: LaneletNetwork,
    lanelets: List[Lanelet],
    node_type_hint: Optional[NodeType] = None,
) -> NodeType:
    node_type = _determine_start_node_type_for_lanelets(lanelet_network, lanelets)
    return select_more_specific_node_type(node_type, node_type_hint)


def _determine_start_node_type_for_lanelets(
    lanelet_network: LaneletNetwork, lanelets: List[Lanelet]
) -> NodeType:
    return NodeType.PRIORITY


def determine_end_node_type_for_lanelets(
    lanelet_network: LaneletNetwork,
    lanelets: List[Lanelet],
    node_type_hint: Optional[NodeType] = None,
) -> NodeType:
    node_type = _determine_end_node_type_for_lanelets(lanelet_network, lanelets)
    return select_more_specific_node_type(node_type, node_type_hint)


def _determine_end_node_type_for_lanelets(
    lanelet_network: LaneletNetwork, lanelets: List[Lanelet]
) -> NodeType:
    lanelet_ids = [lanelet.lanelet_id for lanelet in lanelets]
    for lanelet in lanelets:
        if lanelet.stop_line and lanelet.stop_line.line_marking in {
            LineMarking.SOLID,
            LineMarking.BROAD_SOLID,
        }:
            _LOGGER.debug(
                "Choosing %s for %s, because lanelet %s has a solid stop line",
                NodeType.ALLWAY_STOP,
                lanelet_ids,
                lanelet.lanelet_id,
            )
            return NodeType.ALLWAY_STOP

        if len(lanelet.traffic_lights) > 0:
            return NodeType.TRAFFIC_LIGHT

        intersection = _find_intersection_for_lanelet(lanelet_network, lanelet)
        if intersection is None:
            continue

        if not is_highway_intersection(lanelet_network, intersection):
            return NodeType.RIGHT_BEFORE_LEFT

    all_predecessors = set(
        itertools.chain.from_iterable([lanelet.predecessor for lanelet in lanelets])
    )
    if len(all_predecessors) > len(lanelets):
        _LOGGER.debug(
            "Choosing %s for %s, because they have %s predecessors",
            NodeType.ZIPPER,
            lanelet_ids,
            len(all_predecessors),
        )
        return NodeType.ZIPPER
    elif len(all_predecessors) < len(lanelets):
        _LOGGER.debug(
            "Choosing %s for %s, because they have %s predecessors",
            NodeType.UNREGULATED,
            lanelet_ids,
            len(all_predecessors),
        )
        return NodeType.UNREGULATED

    return NodeType.PRIORITY


def _find_intersection_for_lanelet(
    lanelet_network: LaneletNetwork, lanelet: Lanelet
) -> Optional[Intersection]:
    for intersection in lanelet_network.intersections:
        for incoming in intersection.incomings:
            if lanelet.lanelet_id in incoming.incoming_lanelets:
                return intersection

    return None


def lanelets_are_highway(lanelets: Iterable[Lanelet]) -> bool:
    lanelet_types = list(
        itertools.chain.from_iterable([lanelet.lanelet_type for lanelet in lanelets])
    )
    if len(lanelet_types) == 0:
        return False

    # Collection of lanelet types that indicate a
    highway_lanelet_types = [
        LaneletType.HIGHWAY,
        LaneletType.ACCESS_RAMP,
        LaneletType.EXIT_RAMP,
        LaneletType.INTERSTATE,
    ]
    num_highway_lanelets = 0
    for lanelet_type in lanelet_types:
        if lanelet_type in highway_lanelet_types:
            num_highway_lanelets += 1

    highway_lanelet_types_to_other_ratio = num_highway_lanelets / len(lanelet_types)
    if highway_lanelet_types_to_other_ratio >= 0.5:
        is_highway = True
    else:
        is_highway = False

    return is_highway


def is_highway_intersection(
    lanelet_network: LaneletNetwork, intersection: Intersection
) -> bool:
    """
    Decides whether the intersection should be clustered as a zipper junction.

    :param lanelet_network: The `LaneletNetwork` which contains the `intersection`.
    :param intersection: The `Intersection` for which the cluster instruction is selected.

    :return: The relevant `ClusterInstruction`
    """
    incoming_lanelet_ids = _collect_incoming_lanelet_ids_of_intersection(
        lanelet_network, intersection
    )
    successor_lanelet_ids = _collect_successors_of_intersection(
        lanelet_network, intersection
    )
    incoming_lanelets = collect_lanelets_from_lanelet_ids(
        lanelet_network, incoming_lanelet_ids
    )
    successor_lanelets = collect_lanelets_from_lanelet_ids(
        lanelet_network, successor_lanelet_ids
    )

    return lanelets_are_highway(itertools.chain(incoming_lanelets, successor_lanelets))


def _collect_all_adjacent_lanelet_ids(
    lanelet_network: LaneletNetwork, lanelet_id: int
) -> Set[int]:
    adjacent_lanelets = set()
    lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)

    if lanelet.adj_left is not None:
        adjacent_lanelets.add(lanelet.adj_left)
    if lanelet.adj_right is not None:
        adjacent_lanelets.add(lanelet.adj_right)

    return adjacent_lanelets


def _collect_successors_of_intersection(
    lanelet_network: LaneletNetwork, intersection: Intersection
) -> Set[int]:
    all_successors = set()
    for incoming_element in intersection.incomings:
        all_successors.update(incoming_element.successors_right)
        all_successors.update(incoming_element.successors_left)
        all_successors.update(incoming_element.successors_straight)

    return all_successors


def _collect_successors_of_incoming_in_intersection(
    lanelet_network: LaneletNetwork, intersection: Intersection
):
    pass


def _collect_incoming_lanelet_ids_of_intersection(
    lanelet_network: LaneletNetwork, intersection: Intersection
) -> Set[int]:
    incoming_lanelet_ids = set()
    for incoming_element in intersection.incomings:
        incoming_lanelet_ids.update(incoming_element.incoming_lanelets)
    return incoming_lanelet_ids


def collect_lanelets_from_lanelet_ids(
    lanelet_network: LaneletNetwork, lanelet_ids: Iterable
) -> Set[Lanelet]:
    return set(
        lanelet_network.find_lanelet_by_id(lanelet_id) for lanelet_id in lanelet_ids
    )
