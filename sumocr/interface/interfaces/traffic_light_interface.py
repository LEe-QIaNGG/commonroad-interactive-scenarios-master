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
        """
        Store the desired state for a traffic light.
        Note: This doesn't directly modify SUMO, it's used to track what state we want to set.
        """
        # Store state per traffic light for later use
        if not hasattr(self, '_desired_states'):
            self._desired_states: Dict[CommonRoadId, str] = {}
        self._desired_states[traffic_light_id] = state

    @property
    def tls_id(self):
        return self._tls_id

    def get_state(self, traffic_light_id: CommonRoadId, current_sumo_state: Optional[str] = None) -> str:
        """
        Get the state for a traffic light.
        If current_sumo_state is provided, extract from it; otherwise use desired state if available.
        """
        if current_sumo_state:
            indices = self._indices[traffic_light_id]
            if indices:
                index = indices[0]
                if index < len(current_sumo_state):
                    return current_sumo_state[index]
        
        # Fallback to desired state if set
        if hasattr(self, '_desired_states') and traffic_light_id in self._desired_states:
            return self._desired_states[traffic_light_id]
        
        # Default fallback
        return "o"

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

    def simulate_step(self, time_step: Optional[int] = None) -> bool:
        print(f"[DEBUG] TrafficlightInterface.simulate_step called with time_step={time_step}")
        print(f"[DEBUG] Number of TLS programs: {len(self._programs)}")
        
        # First, sync CommonRoad states to SUMO (existing behavior)
        # But we need to be careful: we should only update the parts we control,
        # not replace the entire state string
        for tls_program in self._programs:
            print(f"[DEBUG] Processing TLS program: {tls_program.tls_id}")
            # Get current state from SUMO to ensure we have the correct length
            try:
                current_sumo_state = self._fetch_tls_state_from_sumo_simulation(tls_program.tls_id)
                if not current_sumo_state:
                    # Skip if we can't get current state
                    continue
                
                # Convert current state to list for manipulation
                state_list = list(current_sumo_state)
                
                # Update only the parts we control based on TlsProgram
                for traffic_light_id in tls_program._indices.keys():
                    # Get desired state from TlsProgram (set via sync_to_sumo_simulation)
                    sumo_state = tls_program.get_state(traffic_light_id, current_sumo_state)
                    # Update all indices for this traffic light
                    for index in tls_program._indices[traffic_light_id]:
                        if index < len(state_list):
                            state_list[index] = sumo_state
                
                # Convert back to string and set in SUMO
                updated_state_string = "".join(state_list)
                self._simulation_backend.traffic_light_domain.setRedYellowGreenState(
                    tls_program.tls_id, updated_state_string
                )
            except Exception as e:
                # If we can't sync to SUMO, log and continue
                import logging
                logging.debug(f"Failed to sync traffic light {tls_program.tls_id} to SUMO: {e}")
                continue
        
        # Then, sync SUMO states back to CommonRoad (new behavior)
        # This ensures that SUMO's automatic traffic light changes are reflected in CommonRoad
        if time_step is not None:
            print(f"[DEBUG] Calling _sync_all_traffic_lights_from_sumo with time_step={time_step}")
            self._sync_all_traffic_lights_from_sumo(time_step)
        else:
            print(f"[DEBUG] WARNING: time_step is None, skipping sync from SUMO!")
        
        return True

    def fetch_new_from_sumo_simulation(self) -> List[SumoId]:
        raise NotImplementedError()

    def sync_from_sumo_simulation(self, traffic_light_id: CommonRoadId, time_step: int) -> TrafficLight:
        """
        Sync the state of a TrafficLight from SUMO simulation to CommonRoad scenario.
        
        :param traffic_light_id: The CommonRoad traffic light ID to sync
        :param time_step: The current time step
        :return: The updated TrafficLight object
        """
        # Get the TLS program for this traffic light
        tls_program = self._get_tls_program_for_traffic_light(traffic_light_id)
        if tls_program is None:
            raise RuntimeError(
                f"Failed to get the corresponding SUMO TLS program for traffic light '{traffic_light_id}'"
            )
        
        # Fetch current state from SUMO to get the actual state
        sumo_state_string = self._fetch_tls_state_from_sumo_simulation(tls_program.tls_id)
        print(f"[DEBUG] sync_from_sumo_simulation: TLS {tls_program.tls_id} state string: '{sumo_state_string}'")
        
        # Get the state for this specific traffic light from the SUMO state string
        sumo_state = tls_program.get_state(traffic_light_id, sumo_state_string)
        print(f"[DEBUG] sync_from_sumo_simulation: Traffic light {traffic_light_id} SUMO state: '{sumo_state}'")
        
        # Convert SUMO state to CommonRoad state
        cr_state = sumo_traffic_light_state_to_cr_traffic_light_state(sumo_state)
        print(f"[DEBUG] sync_from_sumo_simulation: Converted to CR state: {cr_state}")
        
        # Get the TrafficLight object from the scenario
        traffic_light = None
        for tl in self._scenario.lanelet_network.traffic_lights:
            if tl.traffic_light_id == traffic_light_id:
                traffic_light = tl
                break
        
        if traffic_light is None:
            raise ValueError(
                f"Traffic light with ID '{traffic_light_id}' not found in the scenario"
            )
        
        # Update the TrafficLight's cycle with the new state at the current time step
        # CommonRoad TrafficLight uses a cycle list to store states over time
        # We need to ensure the cycle has the state for the current time step
        # Only print detailed debug info for first few steps or every 50 steps
        verbose_debug = (time_step < 5 or time_step % 50 == 0)
        
        if verbose_debug:
            print(f"[DEBUG] sync_from_sumo_simulation: Checking cycle attribute...")
            print(f"[DEBUG] sync_from_sumo_simulation: hasattr(cycle)={hasattr(traffic_light, 'cycle')}")
            if hasattr(traffic_light, 'cycle'):
                cycle_preview = traffic_light.cycle[:5] if len(traffic_light.cycle) > 5 else traffic_light.cycle
                print(f"[DEBUG] sync_from_sumo_simulation: cycle preview (first 5): {cycle_preview}, length={len(traffic_light.cycle)}, type={type(traffic_light.cycle)}")
        
        if hasattr(traffic_light, 'cycle') and traffic_light.cycle is not None:
            # Extend cycle if needed to include the current time step
            cycle_length = len(traffic_light.cycle)
            if verbose_debug:
                print(f"[DEBUG] sync_from_sumo_simulation: Current cycle length: {cycle_length}, time_step: {time_step}")
            if time_step >= cycle_length:
                # Extend cycle by repeating the last state or using the new state
                last_state = traffic_light.cycle[-1] if cycle_length > 0 else cr_state
                if verbose_debug:
                    print(f"[DEBUG] sync_from_sumo_simulation: Extending cycle from {cycle_length} to {time_step + 1}")
                traffic_light.cycle.extend([last_state] * (time_step - cycle_length + 1))
            elif time_step < 0:
                # If time_step is negative, something is wrong
                print(f"[DEBUG] sync_from_sumo_simulation: WARNING: time_step {time_step} is negative!")
                return traffic_light
            
            # Update the state at the current time step
            old_value = traffic_light.cycle[time_step] if time_step < len(traffic_light.cycle) else 'N/A'
            if verbose_debug:
                print(f"[DEBUG] sync_from_sumo_simulation: Setting cycle[{time_step}] = {cr_state} (old value: {old_value})")
            traffic_light.cycle[time_step] = cr_state
            if verbose_debug:
                print(f"[DEBUG] sync_from_sumo_simulation: After update, cycle[{time_step}] = {traffic_light.cycle[time_step]}")
        else:
            # If cycle doesn't exist, create it with states up to current time step
            # First, try to get initial state
            print(f"[DEBUG] sync_from_sumo_simulation: Cycle doesn't exist, creating new cycle")
            initial_state = None
            try:
                initial_state = traffic_light.get_state_at_time_step(0)
                print(f"[DEBUG] sync_from_sumo_simulation: Got initial state from get_state_at_time_step(0): {initial_state}")
            except (AttributeError, IndexError, KeyError) as e:
                print(f"[DEBUG] sync_from_sumo_simulation: Could not get initial state: {e}, using cr_state")
                initial_state = cr_state
            
            # Create cycle list
            traffic_light.cycle = [initial_state] * (time_step + 1)
            traffic_light.cycle[time_step] = cr_state
            print(f"[DEBUG] sync_from_sumo_simulation: Created new cycle with length {len(traffic_light.cycle)}, cycle[{time_step}] = {traffic_light.cycle[time_step]}")
        
        print(f"[DEBUG] sync_from_sumo_simulation: Final cycle state: {traffic_light.cycle if hasattr(traffic_light, 'cycle') else 'N/A'}")
        return traffic_light
    
    def _sync_all_traffic_lights_from_sumo(self, time_step: int):
        """
        Sync all traffic lights from SUMO to CommonRoad scenario.
        This method is called during simulate_step to ensure SUMO's automatic
        traffic light changes are reflected in CommonRoad.
        
        :param time_step: The current time step.
        """
        if time_step % 50 == 0 or time_step < 10:  # Only print every 50 steps or first 10 steps
            print(f"[DEBUG] _sync_all_traffic_lights_from_sumo called with time_step={time_step}")
        
        # Get all TLS programs from SUMO
        tls_id_list = self._simulation_backend.traffic_light_domain.getIDList()
        if time_step % 50 == 0 or time_step < 10:
            print(f"[DEBUG] Found {len(tls_id_list)} TLS programs in SUMO: {tls_id_list}")
        
        # Track which traffic lights we've synced to avoid duplicates
        synced_traffic_light_ids = set()
        
        for tls_id in tls_id_list:
            print(f"[DEBUG] Processing TLS ID: {tls_id}")
            # Fetch current state from SUMO
            sumo_state_string = self._fetch_tls_state_from_sumo_simulation(tls_id)
            print(f"[DEBUG] TLS {tls_id} SUMO state: '{sumo_state_string}'")
            
            if not sumo_state_string:
                print(f"[DEBUG] WARNING: No state string for TLS {tls_id}, skipping")
                continue
            
            # Get controlled lanelets for this TLS program
            try:
                lanelets = self._fetch_controlled_lanelets_of_tls_program(tls_id)
            except (ValueError, StopIteration):
                # Skip if we can't map this TLS program
                continue
            
            # Get traffic light IDs for this TLS program
            traffic_light_ids = []
            for lanelet in lanelets:
                if lanelet.traffic_lights:
                    traffic_light_ids.append(next(iter(lanelet.traffic_lights)))
            
            print(f"[DEBUG] TLS {tls_id} controls {len(traffic_light_ids)} traffic lights: {traffic_light_ids}")
            
            if not traffic_light_ids:
                print(f"[DEBUG] WARNING: No traffic light IDs found for TLS {tls_id}, skipping")
                continue
            
            # Get or create TlsProgram
            tls_program = None
            for tl_id in traffic_light_ids:
                if tl_id in self._program_mapping:
                    tls_program = self._program_mapping[tl_id]
                    break
            
            if tls_program is None:
                # Create new TlsProgram if it doesn't exist
                tls_program = TlsProgram(tls_id, traffic_light_ids)
                self._save_tls_program(traffic_light_ids, tls_program)
            
            # Update each traffic light's state from SUMO
            # The state string contains states for all links in order
            # Each link corresponds to a traffic light
            for i, traffic_light_id in enumerate(traffic_light_ids):
                if traffic_light_id in synced_traffic_light_ids:
                    continue
                
                # Get the state for this traffic light from the state string
                # Use the index to get the corresponding state character
                if i < len(sumo_state_string):
                    sumo_state = sumo_state_string[i]
                else:
                    # Fallback: use the first character if index is out of range
                    sumo_state = sumo_state_string[0] if sumo_state_string else "o"
                
                print(f"[DEBUG] Traffic light {traffic_light_id} (index {i}) state from SUMO: '{sumo_state}'")
                
                # Update TlsProgram internal state
                tls_program.set_state(traffic_light_id, sumo_state)
                
                # Sync this traffic light from SUMO to CommonRoad scenario
                try:
                    if time_step % 50 == 0 or time_step < 10:
                        print(f"[DEBUG] Calling sync_from_sumo_simulation for traffic_light_id={traffic_light_id}, time_step={time_step}")
                    self.sync_from_sumo_simulation(traffic_light_id, time_step)
                    synced_traffic_light_ids.add(traffic_light_id)
                    if time_step % 50 == 0 or time_step < 10:
                        print(f"[DEBUG] Successfully synced traffic light {traffic_light_id}")
                except (ValueError, RuntimeError) as e:
                    # Log error but continue with other traffic lights
                    print(f"[DEBUG] ERROR: Failed to sync traffic light {traffic_light_id} at time_step={time_step}: {e}")
                    import traceback
                    traceback.print_exc()
                    continue

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
