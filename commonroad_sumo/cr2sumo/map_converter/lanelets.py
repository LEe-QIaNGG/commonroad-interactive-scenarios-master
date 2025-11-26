from typing import Dict, List

from commonroad.scenario.lanelet import Lanelet, LaneletNetwork


def partition_lanelet_network_into_edges_and_lanes(
    lanelet_network: LaneletNetwork,
) -> Dict[int, List[int]]:
    """
    Transform the lanelet based CommonRoad representation, into the SUMO representation where each edge consists of multiple lanes.

    In CommonRoad lanelets are mostly independent of each other, as they are not grouped together e.g. to create logical streets.
    But in SUMO the network consists of edges with multiple lanes (~=lanelets).
    As such this method partitions the network into adjacency groups and classifies those as edges.
    Each lanelet in such adjacency group, is then classified as a lane in the resulting edge.

    :param lanelet_network: The lanelet network to partition.

    :returns: A mapping from edge ids to the lanelet ids which are part of that edge. The list of lanelet ids, is ordered by the rightmost lanelet.
    """
    lanelet_ids_by_edge_ids: Dict[int, List[int]] = {}
    partitioned_lanelets = set()
    for lanelet in lanelet_network.lanelets:
        if lanelet.lanelet_id in partitioned_lanelets:
            continue

        partitioned_lanelets.add(lanelet.lanelet_id)
        right_most_lanelet_id = find_right_most_adjacent_lanelet(
            lanelet, lanelet_network
        )
        right_most_lanelet = lanelet_network.find_lanelet_by_id(right_most_lanelet_id)
        ordered_lanelet_ids = find_left_adjacent_lanelet_ids_recursively(
            right_most_lanelet, lanelet_network
        )
        # All lanelets that were found as belonging to this adjacency group, should not be considered for another edge.
        partitioned_lanelets.update(ordered_lanelet_ids)

        lanelet_ids_by_edge_ids[right_most_lanelet_id] = ordered_lanelet_ids

    return lanelet_ids_by_edge_ids


def find_right_most_adjacent_lanelet(
    lanelet: Lanelet, lanelet_network: LaneletNetwork
) -> int:
    """
    Starting from `lanelet` the right most lanelet of this "Street" is identified, by traversing the adjacent lanelets.

    :param lanelet: The lanelet from which the right most lanelet should be selected.
    :param lanelet_network: Lanelet network which contains `lanelet` and all its adjacent lanelets.

    :returns: The ID of the right most lanelet. If `lanelet` does not have an right adjacent lanelet, `lanelet`s ID will be returned.
    """

    right_lanelet = lanelet
    while True:
        lanelet_id_adj_right = right_lanelet.adj_right
        if lanelet_id_adj_right is None:
            # Although this is not indicated by the type annotations of `Lanelet`,
            # `adj_right` can be None, if the lanelet does not have an adjacent lanelet.
            return right_lanelet.lanelet_id

        right_same_direction = right_lanelet.adj_right_same_direction
        if not right_same_direction:
            # If the
            return right_lanelet.lanelet_id

        right_lanelet = lanelet_network.find_lanelet_by_id(lanelet_id_adj_right)
        if right_lanelet is None:
            raise RuntimeError(
                f"Cannot find right most adjacent lanelet id of lanelet {lanelet.lanelet_id}: The lanelet {lanelet_id_adj_right} is set as an adjacent lanelet on {lanelet_id_adj_right}, but it does not exist in the lanelet network!"
            )

        if right_lanelet.lanelet_id == lanelet.lanelet_id:
            raise RuntimeError(
                f"Cannot find right most adjacent lanelet id of lanelet {lanelet.lanelet_id}: There is a loop in the adjacency relations!"
            )


def find_left_adjacent_lanelet_ids_recursively(
    lanelet: Lanelet, lanelet_network: LaneletNetwork
) -> List[int]:
    """
    Get a ordered list of lanelet ids which are left of `lanelet`, starting with `lanelet`s ID.

    Effectivly this returns a list of all lanes inside a logical road.

    :param lanelet: The lanelet from which the adjacent lanelets should be selected.
    :param lanelet_network: Lanelet network which contains `lanelet` and all its adjacent lanelets.

    :returns: Ordered list of lanelet IDs.
    """
    explored_lanelets = [lanelet.lanelet_id]
    left_same_direction = lanelet.adj_left_same_direction

    lanelet_id_adj_left = lanelet.adj_left
    while lanelet_id_adj_left is not None and left_same_direction:
        if lanelet_id_adj_left in explored_lanelets:
            raise RuntimeError(
                f"Cannot find left adjacent lanelet ids of lanelet {lanelet.lanelet_id}: There is a loop in the adjacency relations!"
            )
        explored_lanelets.append(lanelet_id_adj_left)

        left_lanelet = lanelet_network.find_lanelet_by_id(lanelet_id_adj_left)
        if left_lanelet is None:
            raise RuntimeError(
                f"Cannot find left adjacent lanelet ids of lanelet {lanelet.lanelet_id}: The lanelet {lanelet_id_adj_left} is set as an adjacent lanelet, but it does not exist in the lanelet network!"
            )
        lanelet_id_adj_left = left_lanelet.adj_left
        left_same_direction = left_lanelet.adj_left_same_direction

    return explored_lanelets
