import dataclasses
from typing import (
    TypeVar,
)

from commonroad.common.util import Interval
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import (
    CustomState,
    State,
    TraceState,
)
from commonroad.scenario.trajectory import Trajectory

_StateT = TypeVar("_StateT", bound=State)


def convert_state_to_state_type(
    input_state: TraceState, target_state_type: type[_StateT]
) -> _StateT:
    """
    Alternative to `State.convert_state_to_state`, which also accepts type parameters.

    If `input_state` is not already `target_state_type`,
    a new state of type `target_state_type` is created and all attributes,
    that both state types have in common, are copied from `input_state` to the new state.

    :param input_state: State that should be copied. Will not be modified.
    :param target_state_type: State type to which the input will be converted.

    :returns: `input_state` if it already has `target_state_type`, else a new state of type `target_state_type`.
    """
    if isinstance(input_state, target_state_type):
        return input_state

    resulting_state = target_state_type()
    # Make sure that all fields are populated in the end, and no fields are set to 'None'
    resulting_state.fill_with_defaults()

    # Copy over all fields that are common to both state types
    for to_field in dataclasses.fields(target_state_type):
        if to_field.name in input_state.attributes:
            input_state_attribute_value = getattr(input_state, to_field.name)
            setattr(resulting_state, to_field.name, input_state_attribute_value)
    return resulting_state


def convert_state_to_state(
    input_state: TraceState, reference_state: _StateT
) -> _StateT:
    """
    Alternative to `State.convert_state_to_state`, which can also handle `CustomState`.

    :param input_state: The state which should be converted. If the attributes already match those of `reference_state`, `input_state` will be returned.
    :param reference_state: The state which will be used as a reference, for which attributes should be available of the resulting state. All attributes which are not yet present on `input_state` will be set to their defaults.

    :returns: Either the `input_state`, if the attributes already match. Otherwise, a new state with the attributes from `reference_state` and values from `input_state`. If not all attributes of `reference_state` are available in `input_state` they are not included in the new state.
    """
    if set(input_state.used_attributes) == set(reference_state.used_attributes):
        return input_state

    new_state = type(reference_state)()
    new_state.fill_with_defaults()
    for attribute in reference_state.used_attributes:
        if input_state.has_value(attribute):
            setattr(new_state, attribute, getattr(input_state, attribute))

    return new_state


def get_full_state_list_of_obstacle(
    dynamic_obstacle: DynamicObstacle, target_state_type: type[State] | None = None
) -> list[TraceState]:
    """
    Get the state list of the :param:`dynamic_obstacle` including its initial state.
    Will harmonize all states to the same state type, which can be controlled through :param:`target_state_type`.

    :param dynamic_obstacle: The obstacle from which the states should be extracted
    :param target_state_type: Provide an optional state type, to which all resulting states should be converted

    :returns: The full state list of the obstacle where all states have the same state type. This does however not guarantee that all states also have the same attributes, if `CustomState`s are used. See `convert_state_to_state` for more information.
    """
    if target_state_type == CustomState:
        raise ValueError(
            "Cannot convert to state type 'CustomState', because the needed attributes cannot be determined."
        )

    state_list = [dynamic_obstacle.initial_state]
    if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
        state_list += dynamic_obstacle.prediction.trajectory.state_list

    if target_state_type is None:
        # Use the last state from the state_list as the reference state,
        # because for all cases this indicates the correct state type:
        # * If state_list only contains the initial state, it is this state
        #    and this function keeps the state as InitialState
        # * If state_list also contains the trajectory prediction,
        #    the reference state is the last state of this trajectory,
        #    and so the initial state will be converted to the same state type
        #    as all other states in the trajectory.
        reference_state = state_list[-1]
        if isinstance(reference_state, CustomState):
            # If the reference state is a custom state, it needs special treatment,
            # because custom states do not have a pre-definied list of attributes
            # that can be used in the conversion.
            # Instead, the conversion needs to consider the reference state instance.
            return [
                convert_state_to_state(state, reference_state) for state in state_list
            ]
        else:
            target_state_type = type(reference_state)

    # Harmonizes the state types: If the caller wants to construct a trajectory
    # from this state list, all states need to have the same attributes aka. the same state type.
    return [
        convert_state_to_state_type(state, target_state_type) for state in state_list
    ]


def get_full_trajectory_of_obstacle(dynamic_obstacle: DynamicObstacle) -> Trajectory:
    """
    Get the trajectory of `dynamic_obstacle` including its initial state.

    Useful to, e.g., extract a solution trajectory from a dynamic obstacle.

    :param dynamic_obstacle: The obstacle for which to get the trajectory.

    :returns: The full trajectory of the obstacle.
    """
    full_state_list = get_full_state_list_of_obstacle(dynamic_obstacle)
    assert isinstance(full_state_list[0].time_step, int)
    trajectory = Trajectory(
        initial_time_step=full_state_list[0].time_step, state_list=full_state_list
    )
    return trajectory


def get_scenario_final_time_step(scenario: Scenario) -> int:
    """
    Determines the maximum time step in a scenario. This is useful, to determine the length of a scenario.

    :param scenario: The scenario to analyze.

    :return: The final time step in the scenario, or 0 if no obstacles are in the scenario/
    """
    max_time_step = 0
    for dynamic_obstacle in scenario.dynamic_obstacles:
        if dynamic_obstacle.prediction is None:
            max_time_step = max(max_time_step, dynamic_obstacle.initial_state.time_step)
            continue

        max_time_step = max(max_time_step, dynamic_obstacle.prediction.final_time_step)

    if isinstance(max_time_step, Interval):
        return int(max_time_step.end)
    else:
        return max_time_step


def get_scenario_start_time_step(scenario: Scenario) -> int:
    """
    Determines the minimum time step in a scenario.

    :param scenario: The scenario to analyze.

    :return: The first time step in the scenario, or 0 if no obstacles are in the scenario.
    """
    time_steps = [
        dynamic_obstacle.initial_state.time_step
        for dynamic_obstacle in scenario.dynamic_obstacles
    ]
    if len(time_steps) == 0:
        return 0

    min_time_step = min(time_steps)

    if isinstance(min_time_step, Interval):
        return int(min_time_step.start)
    else:
        return min_time_step
