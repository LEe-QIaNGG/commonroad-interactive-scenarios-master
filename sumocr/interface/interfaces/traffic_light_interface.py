from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple

from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightState

from sumocr.interface.id_mapper import CommonRoadId, IdMapper, SumoId
from sumocr.interface.interfaces.base_interface import BaseInterface
from sumocr.interface.sumo_simulation_backend import SumoSimulationBackend

_STATE_MAPPING_SUMO2CR = {
    "g": TrafficLightState.GREEN,
    "G": TrafficLightState.GREEN,
    "r": TrafficLightState.RED,
    "u": TrafficLightState.RED_YELLOW,
    "y": TrafficLightState.YELLOW,
    "o": TrafficLightState.INACTIVE,
}

_STATE_MAPPING_CR2SUMO = {
    TrafficLightState.GREEN: "g",
    TrafficLightState.RED: "r",
    TrafficLightState.RED_YELLOW: "u",
    TrafficLightState.YELLOW: "y",
    TrafficLightState.INACTIVE: "o",
}


def sumo_traffic_light_state_to_cr_traffic_light_state(
    traffic_light_state: str,
) -> TrafficLightState:
    """Convert the state of a traffic light from SUMO to CommonRoad format"""
    if len(traffic_light_state) != 1:
        raise ValueError(
            f"Invalid SUMO traffic light state: '{traffic_light_state}'. A traffic light state must be exactly of length '1', but is '{len(traffic_light_state)}' "
        )
    converted_state = _STATE_MAPPING_SUMO2CR.get(traffic_light_state)
    if converted_state is None:
        raise ValueError(f"Unknown SUMO traffic light state: '{traffic_light_state}'")

    return converted_state


def cr_traffic_light_state_to_sumo_traffic_light_state(
    traffic_light_state: TrafficLightState,
) -> str:
    """Convert the state of a traffic light from CommonRoad to SUMO format"""
    converted_state = _STATE_MAPPING_CR2SUMO.get(traffic_light_state)
    if converted_state is None:
        raise ValueError(
            f"Unknown CommonRoad traffic light state: '{traffic_light_state}'"
        )

    return converted_state


class TlsProgram:
    """
    Helper class to
    """

    def __init__(self, tls_id: str, traffic_light_ids: List[CommonRoadId]):
        self._tls_id = tls_id

        # Internal traffic light state as list, to make manipulation via indices easier
        self._state: List[str] = ["o"] * len(traffic_light_ids)
        # self._indices contains the index inside self._state for each traffic_light_id.
        # In SUMO TLS programs, a traffic light can control multiple links.
        # Therefore we need to store the target indices as a list (referencing the different links)
        self._indices: Dict[CommonRoadId, List[int]] = defaultdict(list)

        # Initialization of self._indices according to the order of the passed traffic_light_ids
        for i, traffic_light_id in enumerate(traffic_light_ids):
            self._indices[traffic_light_id].append(i)

    def set_state(self, traffic_light_id: CommonRoadId, state: str):
        indices = self._indices[traffic_light_id]
        # Make sure to update all targeted links
        for index in indices:
            self._state[index] = state

    @property
    def tls_id(self):
        return self._tls_id

    def get_state(self, traffic_light_id: CommonRoadId) -> str:
        indicies = self._indices[traffic_light_id]
        # Use the first index in the list, as all referenced links, are controlled by the same traffic light
        # Therefore all (should) have the same state attached
        return self._state[indicies[0]]

    def as_sumo_state_string(self) -> str:
        return "".join(self._state)

    def __hash__(self):
        return hash(self._tls_id)


class TrafficlightInterface(BaseInterface[TrafficLight]):
    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
        scenario: Scenario,
    ):
        super().__init__(simulation_backend, id_mapper)
        self._scenario = scenario

        self._programs: Set[TlsProgram] = set()
        # self._program_mapping helps us to ease the access to the corresponding TlsProgram for a TrafficLight without
        # having to iterate of self._programs
        self._program_mapping: Dict[CommonRoadId, TlsProgram] = {}

    def simulate_step(self) -> bool:
        for tls_program in self._programs:
            # TODO: do we have to check for collisions with changes made by sumo itself?
            state_string = tls_program.as_sumo_state_string()
            self._simulation_backend.traffic_light_domain.setRedYellowGreenState(
                tls_program.tls_id, state_string
            )
        return True

    def fetch_new_from_sumo_simulation(self) -> List[SumoId]:
        raise NotImplementedError()

    def sync_from_sumo_simulation(self, traffic_light_id: CommonRoadId) -> TrafficLight:
        raise NotImplementedError()

    def _fetch_controlled_lanelets_of_tls_program(
        self, tls_id: SumoId
    ) -> List[Lanelet]:
        """
        Fetch the lanelets in the CommonRoad scenario, that the
        To construct the connection between the CommonRoad scenario and the SUMO simulation we rely on the stable edge ID generation
        from the commonroad-scenario-designer.

        :param tls_id: The TlsProgram to query
        :return: The ordered list of lanelets in the CommonRoad scenario that are controlled by the tls_id program
        """
        lanelets: List[Lanelet] = []
        controlled_links: List[
            List[Tuple[str, str, str]]
        ] = self._simulation_backend.traffic_light_domain.getControlledLinks(tls_id)
        # NOTE: The iteration order is important, as the index of each link_tuple corresponds to the index inside the TLS program.
        # We therefore need to ensure that the returned lanelets lists is in the same order as the controlled_links
        for link_list in controlled_links:
            # All links correspond to the same lane, therefore we can just use the first one
            link_tuple = link_list[0]
            from_lane, to_lane, via_lane = link_tuple

            try:
                # Here we use the stable edge Id generation of the commonroad-scenario-designer, to directly map the SUMO edge ID to a CommonRoad lanelet ID
                from_lanelet_id = int(
                    self._simulation_backend.lane_domain.getEdgeID(from_lane)
                )
            except ValueError:
                raise ValueError(
                    f"The edge ID '{from_lane}' in SUMO is not a valid integer, therefore we cannot map it to a CommonRoad scenario."
                )

            try:
                # Get the first (and only) matching lanelet
                from_lanelet = next(
                    filter(
                        lambda lanelet: lanelet.lanelet_id == from_lanelet_id,
                        self._scenario.lanelet_network.lanelets,
                    )
                )
                lanelets.append(from_lanelet)
            except StopIteration:
                raise ValueError(
                    f"We tried to get a lanelet from CommonRoad with ID '{from_lanelet_id}' from SUMO, but this ID does not map to a lanelet in the current CommonRoad scenario."
                )

        return lanelets

    def _save_tls_program(
        self, traffic_light_ids: List[CommonRoadId], tls_program: TlsProgram
    ) -> None:
        """Assign the traffic_light_ids to the tls_program"""
        self._programs.add(tls_program)
        for traffic_light_id in traffic_light_ids:
            if traffic_light_id not in self._program_mapping:
                self._program_mapping[traffic_light_id] = tls_program

    def _fetch_tls_program_from_sumo_simulation(
        self, traffic_light_id: CommonRoadId
    ) -> Optional[TlsProgram]:
        """
        Fetch the TLS program from SUMO that corresponds to the given traffic light and save the mapping internally.
        """
        tls_id_list = self._simulation_backend.traffic_light_domain.getIDList()
        for tls_id in tls_id_list:
            lanelets = self._fetch_controlled_lanelets_of_tls_program(tls_id)

            # Construct the traffic_light_id list
            traffic_light_ids = []
            for lanelet in lanelets:
                # TODO: Is it correct, to always use the first traffic light?
                traffic_light_ids.append(next(iter(lanelet.traffic_lights)))

            # Check if this is the TLS program that we are looking for
            if traffic_light_id in traffic_light_ids:
                tls_program = TlsProgram(tls_id, traffic_light_ids)
                # Save the TLS program and the traffic light id mappings for later
                self._save_tls_program(traffic_light_ids, tls_program)
                return tls_program

        return None

    def _fetch_tls_state_from_sumo_simulation(self, tls_id: str) -> str:
        state_str: str = (
            self._simulation_backend.traffic_light_domain.getRedYellowGreenState(tls_id)
        )
        return state_str

    def _get_tls_program_for_traffic_light(
        self, traffic_light_id: CommonRoadId
    ) -> Optional[TlsProgram]:
        """
        Get the corresponding SUMO TLS program for a given CommonRoad traffic light.
        If the traffic light is not yet mapped internally, it will fetch the TLS program from SUMO, otherwise the internal mapping will be used.
        """
        if traffic_light_id in self._program_mapping:
            return self._program_mapping[traffic_light_id]
        else:
            tls_program = self._fetch_tls_program_from_sumo_simulation(traffic_light_id)
            return tls_program

    def sync_to_sumo_simulation(
        self, traffic_light: TrafficLight, time_step: int
    ) -> bool:
        cr_traffic_light_state = traffic_light.get_state_at_time_step(time_step)
        sumo_traffic_light_state = cr_traffic_light_state_to_sumo_traffic_light_state(
            cr_traffic_light_state
        )

        tls_program = self._get_tls_program_for_traffic_light(
            traffic_light.traffic_light_id
        )
        if tls_program is None:
            raise RuntimeError(
                f"Failed to get the corresponding SUMO TLS program for traffic light '{traffic_light.traffic_light_id}'"
            )
        tls_program.set_state(traffic_light.traffic_light_id, sumo_traffic_light_state)
        return True


__all__ = ["TrafficlightInterface"]
