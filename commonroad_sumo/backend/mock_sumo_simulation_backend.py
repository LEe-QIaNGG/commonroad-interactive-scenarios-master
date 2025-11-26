import logging
from typing import Any, Dict, List

import libsumo
import traci
import traci.domain

from commonroad_sumo.backend.sumo_simulation_backend import (
    SumoSimulationBackend,
    SumoSimulationBackendConfiguration,
)

_LOGGER = logging.getLogger()


def _var_id_to_possible_names(varID: int) -> str:
    """Convert a traci Variable ID to all possible names. Used for debugging"""
    possible_names = []
    for key, value in libsumo.constants.__dict__.items():
        if value == varID:
            possible_names.append(key)

    if len(possible_names) == 0:
        return str(varID)
    elif len(possible_names) == 0:
        return possible_names[0]
    else:
        return "One of " + ", ".join(possible_names)


# The MockSumoSimulationBackend can be used in place of any SumoSimulationBackend,
# but does not perform any actions and also does not need SUMO.
# It acts as a Key-Value store, but tries to replicate the behaviour of the
# simulation as close as possible, e.g by modifying the position in MOVE_TO_XY commands.
# For this purpose it intercepts the TraCI messages and replaces them with stubs.
# This way, a 'MockTraCI' is created, which has the same API as real TraCI,
# and therefore can be easily used in place of the original API.


class MockDomain(traci.domain.Domain):
    """
    Mock object for a TraCI domain. It can be used as a mixin for all TraCI domains, to act as a Key-Value store instead of passing messages between the SUMO simulation and the python enivronment.
    """

    def __init__(self, *args, **kwargs) -> None:
        self._object_registry: Dict[str, Dict[int, Any]] = dict()

    # Override the _getUniveral method from traci.domain.Domain, which is called in all 'get' methods (like vehicle.getSpeed).
    def _getUniversal(self, varID, objectID="", format="", *values):
        _LOGGER.debug(
            f"_getUniveral for '{_var_id_to_possible_names(varID)}' for '{objectID}' with '{values}'"
        )
        if varID == traci.constants.TRACI_ID_LIST:
            return tuple(self._object_registry.keys())

        if objectID not in self._object_registry:
            raise ValueError(
                f"Cannot get variable '{_var_id_to_possible_names(varID)}' for object '{objectID}', because object does not exist!"
            )

        obj = self._object_registry[objectID]
        if varID not in obj:
            _LOGGER.error(
                f"No value for '{_var_id_to_possible_names(varID)}' for object '{objectID}'"
            )
            return None

        val = obj[varID]
        if varID == traci.constants.VAR_PARAMETER:
            val = val[2]
        _LOGGER.debug(
            f"_getUniveral found value '{val}' for '{_var_id_to_possible_names(varID)}' for '{objectID}'"
        )
        return val

    # Overide the _setCmd method of traci.domain.Domain, which is called in all set methods (like vehicle.setSpeed), and also for other mutating calls.
    def _setCmd(self, varID, objectID, format="", *values):
        _LOGGER.debug(
            f"_setCmd with '{_var_id_to_possible_names(varID)}' for '{objectID}' with '{values}'"
        )
        if varID == traci.constants.ADD_FULL:
            # normally called for vehicles
            self._object_registry[objectID] = dict()
            # VAR_SPEED_LAT is present in the SUMO simulation, but cannot be set in any other way.
            # Because the interfaces rely on this value to be present, we make sure it is there.
            self._object_registry[objectID][traci.constants.VAR_SPEED_LAT] = 0.0
        elif varID == traci.constants.ADD:
            # normally called for persons
            self._object_registry[objectID] = dict()
        elif objectID not in self._object_registry:
            # No addition by default, to catch logic errors
            raise ValueError(
                f"Cannot execute _setCmd with '{_var_id_to_possible_names(varID)}' for object '{objectID}', because object does not exist!"
            )

        if varID == traci.constants.MOVE_TO_XY:
            # MoveToXY needs special handling, because it would otherwise not set the position and angle correctly
            self._object_registry[objectID][traci.constants.VAR_POSITION] = (
                values[3],
                values[4],
            )
            self._object_registry[objectID][traci.constants.VAR_ANGLE] = values[5]
        else:
            if len(values) == 1:
                self._object_registry[objectID][varID] = values[0]
            else:
                self._object_registry[objectID][varID] = values


# By mixing each existing TraCI domain (e.g. traci.main._vehicle.VehicleDomain) with our MockDomain, we override the important get/set methods and get Mock<Name>Domain.
# Those Domains can then be used exactly like one would use the TraCI domains.


# Sadly TraCI does not expose any types for their domains, so we must use the private modules
class MockVehicleDomain(MockDomain, traci.main._vehicle.VehicleDomain): ...


class MockPersonDomain(MockDomain, traci.main._person.PersonDomain): ...


class MockTrafficLightDomain(MockDomain, traci.main._trafficlight.TrafficLightDomain):
    def _setCmd(self, varID, objectID, format="", *values):
        # For traffic lights, we always create the traffic light if it does not exist, because TraCI has no method for adding traffic lights during the simulation...
        if objectID not in self._object_registry:
            self._object_registry[objectID] = dict()
        return super()._setCmd(varID, objectID, format, *values)


class MockLaneDomain(MockDomain, traci.main._lane.LaneDomain):
    def _setCmd(self, varID, objectID, format="", *values):
        # For lanes, we always create the lane object, because TraCI has no method for adding lanes during the simulation...
        if objectID not in self._object_registry:
            self._object_registry[objectID] = dict()
        return super()._setCmd(varID, objectID, format, *values)


class MockVehicleTypeDomain(MockDomain, traci.main._vehicletype.VehicleTypeDomain): ...


class MockEdgeDomain(MockDomain, traci.main._edge.EdgeDomain): ...


class MockSimulationDomain(MockDomain, traci.main._simulation.SimulationDomain): ...


class MockRouteDomain(MockDomain, traci.main._route.RouteDomain): ...


class MockTraci:
    """
    Mimicks the TraCI API, but uses the mocked domains under the hood.
    """

    def __init__(self):
        self.vehicle = MockVehicleDomain()
        self.vehicletype = MockVehicleTypeDomain()
        self.person = MockPersonDomain()
        self.trafficlight = MockTrafficLightDomain()
        self.lane = MockLaneDomain()
        self.edge = MockEdgeDomain()
        self.simulation = MockSimulationDomain()

    def start(self, cmd: List[str]) -> None: ...

    def simulationStep(self) -> None: ...

    def close(self) -> None: ...


class MockSumoSimulationBackend(SumoSimulationBackend):
    """
    Mock SUMO backend, that does not execute any real commands and instead saves all values in a key-value store.
    """

    def __init__(self) -> None:
        super().__init__(
            SumoSimulationBackendConfiguration(
                dt=1.0, delta_steps=1.0, lateral_resolution=1.0
            ),
            sumo_api=MockTraci(),  # type: ignore
            sumo_binary_path="",  # type: ignore
        )
