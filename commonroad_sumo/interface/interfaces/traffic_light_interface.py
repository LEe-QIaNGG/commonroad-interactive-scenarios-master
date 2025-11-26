import logging
from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
    TrafficLightState,
)

from commonroad_sumo.backend import SumoSimulationBackend
from commonroad_sumo.interface.id_mapper import CommonRoadId, IdMapper, SumoId
from commonroad_sumo.interface.interfaces.base_interface import BaseInterface

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

_logger = logging.getLogger("commonroad_sumo")


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
    Helper class to enable clean interactions between CommonRoad Trafficlights and their corresponding SUMO TLS programs,
    without the caller having to have concret knowledge, about the underlying TLS program.

    For manipulating Trafficlights in SUMO we need to edit the TLS programs state string at a specific index.
    To make this easier, this class adds the indirection: CommonRoad Trafficlight -> TLS index -> TLS state string,
    which allows the caller to directly work with CommonRoad Trafficlights, without having to care about indices.
    """

    def __init__(self, tls_id: str, traffic_light_ids: List[CommonRoadId]):
        """
        :param tls_id: The ID of the TLS program
        :param traffic_light_ids: The list of CommonRoad Trafficlight IDs, that is ordered according to the controlled links of the TLS program
        """
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

    def update_from_state_string(
        self, traffic_light_id: CommonRoadId, state_string: str
    ):
        indices = self._indices[traffic_light_id]
        for index in indices:
            self._state[index] = state_string[index]

    def set_state(self, traffic_light_id: CommonRoadId, state: str):
        """
        :param traffic_light_id: The ID of the traffic light that should be updated
        :param state: The new traffic light state
        """
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
    """
    Interface for Syncing CommonRoad TrafficLights with SUMO TLS Programs
    """

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

        # Track if the TLS program was updated since the last simulation step was performed
        self._updated_since_last_step = False

    def simulate_step(self) -> bool:
        super().simulate_step()
        if self._updated_since_last_step:
            for tls_program in self._programs:
                state_string = tls_program.as_sumo_state_string()
                self._simulation_backend.set_traffic_light_state(
                    tls_program.tls_id, state_string
                )
            self._updated_since_last_step = False
            return True
        return False

    def _has_tls_program(self, tls_id: SumoId) -> bool:
        for program in self._programs:
            if program.tls_id == tls_id:
                return True
        return False

    def fetch_new_from_sumo_simulation(self) -> List[TrafficLight]:
        new_traffic_lights = []
        traffic_light_ids = self._simulation_backend.get_traffic_light_ids()

        for traffic_light_id in traffic_light_ids:
            if not self._has_tls_program(traffic_light_id):
                cr_traffic_light_ids = (
                    self._fetch_cr_traffic_light_ids_controlled_by_tls_program(
                        traffic_light_id
                    )
                )
                for cr_traffic_light_id in cr_traffic_light_ids:
                    # TODO: Some features are missing here like syncing position, cycles and directions
                    traffic_light = TrafficLight(
                        cr_traffic_light_id, position=np.array([0.0, 0.0])
                    )
                    new_traffic_lights.append(traffic_light)

                tls_program = TlsProgram(traffic_light_id, cr_traffic_light_ids)
                self._save_tls_program(cr_traffic_light_ids, tls_program)

        return new_traffic_lights

    def sync_from_sumo_simulation(self, traffic_light: TrafficLight) -> bool:
        tls_program = self._get_tls_program_for_traffic_light(
            traffic_light.traffic_light_id
        )
        if tls_program is None:
            raise RuntimeError(
                f"Failed to get the corresponding SUMO TLS program for traffic light '{traffic_light.traffic_light_id}'"
            )
        state_string = self._simulation_backend.get_traffic_light_state(
            tls_program.tls_id
        )
        tls_program.update_from_state_string(
            traffic_light.traffic_light_id, state_string
        )
        traffic_light_state = sumo_traffic_light_state_to_cr_traffic_light_state(
            tls_program.get_state(traffic_light.traffic_light_id)
        )

        if traffic_light.traffic_light_cycle is None:
            # Construct a new TrafficLightCycle
            traffic_light_cycles = [
                TrafficLightCycleElement(state=traffic_light_state, duration=1)
            ]
            traffic_light.traffic_light_cycle = TrafficLightCycle(
                cycle_elements=traffic_light_cycles
            )
        elif traffic_light.traffic_light_cycle.cycle_elements is None:
            # Update the current cycle and create the cycle elements list
            traffic_light_cycle_element = TrafficLightCycleElement(
                state=traffic_light_state, duration=1
            )

            traffic_light.traffic_light_cycle.cycle_elements = [
                traffic_light_cycle_element
            ]
        else:
            previous_cycle_element = traffic_light.traffic_light_cycle.cycle_elements[
                -1
            ]
            if previous_cycle_element.state == traffic_light_state:
                # The state of the traffic light did not change, so we can simple extend the duration of the previous cycle element
                previous_cycle_element.duration += 1
            else:
                # The state of the traffic light changed, so we need a new cycle element
                traffic_light_cycle_element = TrafficLightCycleElement(
                    state=traffic_light_state, duration=1
                )
                traffic_light.traffic_light_cycle.cycle_elements.append(
                    traffic_light_cycle_element
                )

        return True

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
        controlled_links: List[List[Tuple[SumoId, SumoId, SumoId]]] = (
            self._simulation_backend.get_traffic_light_controlled_links(tls_id)
        )
        # The iteration order is important, as the index of each link_tuple corresponds to the index inside the TLS program.
        # We therefore need to ensure that the returned lanelets lists is in the same order as the controlled_links
        for link_list in controlled_links:
            # All links correspond to the same edge, therefore we can just use the first one
            link_tuple = link_list[0]
            from_lane, to_lane, via_lane = link_tuple
            from_lanelet_id_str = self._simulation_backend.get_edge_id_of_lane(
                from_lane
            )

            try:
                # Here we use the stable edge Id generation of the commonroad-scenario-designer, to directly map the SUMO edge ID to a CommonRoad lanelet ID
                # This is possible because the commonroad-scenario-designer uses the CommonRoad lanelet ID for the edges
                from_lanelet_id = int(from_lanelet_id_str)
            except ValueError:
                raise ValueError(
                    f"The edge ID '{from_lanelet_id_str}' in SUMO is not a valid integer, therefore we cannot map it to a CommonRoad scenario."
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

    def _fetch_cr_traffic_light_ids_controlled_by_tls_program(
        self, tls_id: SumoId
    ) -> List[CommonRoadId]:
        lanelets = self._fetch_controlled_lanelets_of_tls_program(tls_id)

        # Construct the traffic_light_id list
        traffic_light_ids = []
        for lanelet in lanelets:
            if len(lanelet.traffic_lights) > 1:
                # TODO: handle lanelets with multiple traffic lights
                raise ValueError(
                    f"The lanelet '{lanelet.lanelet_id}' has {len(lanelet.traffic_lights)} attached to it, but currently only one traffic light per lanelet is supported."
                )
            if len(lanelet.traffic_lights) == 0:
                raise ValueError(
                    f"The lanelet '{lanelet.lanelet_id}' is associated with the TLS program '{tls_id}', but it is not controlled by a CommonRoad traffic light."
                )
            # lanelet.traffic_lights is a set, therfore we cannot use simple index based access
            traffic_light_ids.append(next(iter(lanelet.traffic_lights)))
        return traffic_light_ids

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
        tls_id_list = self._simulation_backend.get_traffic_light_ids()
        for tls_id in tls_id_list:
            traffic_light_ids = (
                self._fetch_cr_traffic_light_ids_controlled_by_tls_program(tls_id)
            )
            # Check if this is the TLS program that we are looking for
            if traffic_light_id in traffic_light_ids:
                tls_program = TlsProgram(tls_id, traffic_light_ids)
                # Save the TLS program and the traffic light id mappings for later
                self._save_tls_program(traffic_light_ids, tls_program)
                return tls_program

        return None

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

    def sync_to_sumo_simulation(self, traffic_light: TrafficLight) -> bool:
        """
        Sync a CommonRoad TrafficLight to the SUMO simulation.
        The sync required that a corresponding TLS program already exists in the SUMO simulation and is attached to the junction.

        :return: Whether a change in SUMO was performed.
        """
        tls_program = self._get_tls_program_for_traffic_light(
            traffic_light.traffic_light_id
        )
        if tls_program is None:
            raise RuntimeError(
                f"Failed to get the corresponding SUMO TLS program for traffic light '{traffic_light.traffic_light_id}'. Available TLS programs are: '{self._simulation_backend.get_traffic_light_ids()}'"
            )

        # Perform safety checks, as the current commonroad-io version sadly does not perform those
        if traffic_light.traffic_light_cycle is None:
            return False

        if (
            traffic_light.traffic_light_cycle.cycle_elements is None
            or len(traffic_light.traffic_light_cycle.cycle_elements) == 0
        ):
            return False

        cr_traffic_light_state = traffic_light.get_state_at_time_step(
            self._current_time_step
        )

        sumo_traffic_light_state = cr_traffic_light_state_to_sumo_traffic_light_state(
            cr_traffic_light_state
        )

        if (
            tls_program.get_state(traffic_light.traffic_light_id)
            == sumo_traffic_light_state
        ):
            # The state already matches, nothing to do for us
            return False

        tls_program.set_state(traffic_light.traffic_light_id, sumo_traffic_light_state)
        self._updated_since_last_step = True
        return True


__all__ = ["TrafficlightInterface"]
