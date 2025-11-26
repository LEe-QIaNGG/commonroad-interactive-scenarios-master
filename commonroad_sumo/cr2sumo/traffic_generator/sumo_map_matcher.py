import math
import statistics
from collections.abc import Sequence

import sumolib


class SumoMapMatcher:
    """
    Perform advanced map matching tasks on SUMO networks.
    """

    _sumo_net: sumolib.net.Net
    _map_matching_delta: float

    def __init__(self, sumo_net: sumolib.net.Net, map_matching_delta: float) -> None:
        self._sumo_net = sumo_net
        self._map_matching_delta = map_matching_delta

    def junction_at_position(
        self,
        position: tuple[float, float],
        incoming_edge: sumolib.net.edge.Edge | None = None,
        outgoing_edge: sumolib.net.edge.Edge | None = None,
    ) -> sumolib.net.node.Node | None:
        """
        Get the SUMO junction at the position.

        Optionally uses hints about the incoming or outgoing edge, to resolve overlapping or ambiguous junctions.

        :param position: Position for which to query for a junction.
        :param incoming_edge: Optionally filter for junctions, which have this edge as incoming.
        :param outgoing_edge: Optionally filter for junctions, which have this edge as outgoing.

        :returns: The most likely matching junction, or None if no junction is present at the position.
        """
        # Brute-force search which is very inefficient but does the job for now.
        matching_nodes: list[sumolib.net.node.Node] = []
        for node in self._sumo_net.getNodes():
            node_shape = node.getShape()
            if sumolib.geomhelper.isWithin(position, node_shape):
                # The junction encloses our position -> valid match.
                matching_nodes.append(node)

        # Filter for junctions which have `incoming_edge` as incoming edge.
        if incoming_edge is not None:
            matching_nodes = list(
                filter(lambda node: incoming_edge in node.getIncoming(), matching_nodes)
            )

        # Filter for junctions which have `outgoing_edge` as outgoing edge.
        if outgoing_edge is not None:
            matching_nodes = list(
                filter(lambda node: outgoing_edge in node.getOutgoing(), matching_nodes)
            )

        # No matching node was found.
        if len(matching_nodes) == 0:
            return None

        # Not very sophisticated selection of 'best' matching node, but its works well for now.
        best_matching_node = matching_nodes[0]

        return best_matching_node

    def get_neighboring_edges(
        self, position: tuple[float, float], include_junctions: bool = True
    ) -> list[tuple[sumolib.net.edge.Edge, float]]:
        """
        Get the edges close `position`.

        :param position: Cartesian position in SUMO network.
        :param include_junctions: Whether to include internal junction edges.

        :returns: List of possible neighboring edges, sorted by distance to position.
        """
        neighboring_edges = self._sumo_net.getNeighboringEdges(
            x=position[0],
            y=position[1],
            r=self._map_matching_delta,
            includeJunctions=include_junctions,
        )

        return list(
            sorted(neighboring_edges, key=lambda edge_dist_pair: edge_dist_pair[1])
        )

    def map_trace(
        self,
        position_trace: Sequence[tuple[float, float]],
    ) -> list[sumolib.net.edge.Edge]:
        """
        Map a position trace to a SUMO edge list.

        More robust version of `sumolib.route.mapTrace`. Since the SUMO implementation does not
        check the resulting edge list for connectivity, it might produce routes which cannot be simulated.
        This version ensures that the produced routes are indeed correct and can be simulated.

        :param position_trace: Position trace which covers every edge of the route.

        :returns: The most likely SUMO edge list, matching the position trace.
        """
        # Collect all edges which could be a matching edge at a specific position.
        candidates = [
            self.get_neighboring_edges(position, include_junctions=False)
            for position in position_trace
        ]

        # Since we explicitly do not match inside junctions, the candidate route can be empty,
        # if a vehicle starts and ends inside a junction.
        # In this case, the caller should determine the further behavior.
        if len(candidates) < 1:
            return list()

        # Collect all routes which are theoretically possible.
        # A route is considered possible, if all edges are connected.
        possible_routes: dict[sumolib.net.edge.Edge, list[sumolib.net.edge.Edge]] = {}
        # Remember the distance of each traced position to the edge, to later compute a confidence score for each route.
        possible_routes_dists = {}

        # Initialize the possible routes. The routes are built from each possible start edge.
        for edge, dist in candidates[0]:
            possible_routes[edge] = [edge]
            possible_routes_dists[edge] = [dist]

        # Continuously build the possible routes based on the next candidate edges.
        for candidate in candidates[1:]:
            # Check each candidate against each possible route.
            for start_edge, route in possible_routes.items():
                # A candidate consists of multiple individual edge distance pairs.
                # Each of those edges is evaluated individually however only one might match.
                for edge, dist in candidate:
                    if edge == route[-1]:
                        possible_routes_dists[start_edge].append(dist)
                        # Edge stays the same, continue with next possible route.
                        break

                    # Check whether there is a valid connection to at least
                    for connections in route[-1].getOutgoing().values():
                        for connection in connections:
                            if connection.getTo() == edge:
                                possible_routes[start_edge].append(edge)
                                possible_routes_dists[start_edge].append(dist)
                                break
                        else:
                            continue
                        break

        current_best_route = []
        current_best_score = math.inf
        for start_edge, route in possible_routes.items():
            # Simple method to determine the score for our route. The deciding metric here is
            # confidence in each route. The confidence for each point is quite directly given by
            # the distance of each position to each edge of the route. By combining them with the
            # median, high confidence routes are much more preferred even if they contain initial outliers.
            # This helps to select routes which start in junctions where the upstream route is not clearly defined.
            # The more fitting route should be longer and have a higher confidence at its tail.
            score = statistics.median(possible_routes_dists[start_edge])
            if score < current_best_score:
                # We found a better route -> make this the new best route.
                current_best_route = route
                current_best_score = score

        return current_best_route
