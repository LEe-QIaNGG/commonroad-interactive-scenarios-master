from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario


class SumoInterfaceError(Exception):
    """Generic error base for all errors raised by commonroad-sumo."""

    ...


class SumoSimulationError(SumoInterfaceError):
    """Error to capture issues in the SUMO simulation which are most likely not a fault of the commonroad-sumo interface."""

    ...


class SumoTrafficGenerationError(SumoInterfaceError): ...


class SumoInteractivePlanningFailedError(SumoInterfaceError):
    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem) -> None:
        super().__init__(
            f"Planning for {planning_problem.planning_problem_id} at time step {planning_problem.initial_state.time_step} in scenario {scenario.scenario_id} failed"
        )
        self.scenario = scenario
        self.planning_problem = planning_problem
