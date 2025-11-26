from commonroad_sumo.errors import SumoTrafficGenerationError

from .flow_traffic_generator import (
    DemandTrafficGenerator,
    InfrastructureTrafficGenerator,
    RandomTrafficGenerator,
)
from .random_trips_traffic_generator import RandomTripsTrafficGenerator
from .traffic_generator import AbstractTrafficGenerator, SumoTrafficGenerationMode
from .trajectory_traffic_generator import (
    SafeResimulationTrafficGenerator,
    UnsafeResimulationTrafficGenerator,
)


def create_traffic_generator_for_mode(
    mode: SumoTrafficGenerationMode,
) -> AbstractTrafficGenerator:
    if mode == SumoTrafficGenerationMode.RANDOM:
        return RandomTrafficGenerator()
    elif mode == SumoTrafficGenerationMode.DEMAND:
        return DemandTrafficGenerator()
    elif mode == SumoTrafficGenerationMode.INFRASTRUCTURE:
        return InfrastructureTrafficGenerator()
    elif mode == SumoTrafficGenerationMode.SAFE_RESIMULATION:
        return SafeResimulationTrafficGenerator()
    elif mode == SumoTrafficGenerationMode.UNSAFE_RESIMULATION:
        return UnsafeResimulationTrafficGenerator()
    elif mode == SumoTrafficGenerationMode.RANDOM_TRIPS:
        return RandomTripsTrafficGenerator()
    else:
        raise SumoTrafficGenerationError(
            f"Cannot create traffic generator for mode {mode}: This mode is unkown!"
        )
