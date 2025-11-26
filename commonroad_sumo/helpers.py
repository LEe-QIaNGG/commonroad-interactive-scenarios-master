__all__ = [
    "get_sumo_gui_binary_path",
    "get_sumo_binary_path",
    "get_path_for_sumo_tool",
    "get_path_for_sumo_application",
]

import copy
import logging
import os
import shutil
import subprocess
from enum import Enum, auto
from pathlib import Path
from typing import Optional, Union

from commonroad.scenario.scenario import Scenario

_LOGGER = logging.getLogger(__name__)


class SumoApplication(Enum):
    SUMO = auto()
    SUMO_GUI = auto()
    DUAROUTER = auto()
    NETCONVERT = auto()

    def get_application_name(self) -> str:
        if self == SumoApplication.SUMO:
            return "sumo"
        elif self == SumoApplication.SUMO_GUI:
            return "sumo-gui"
        elif self == SumoApplication.DUAROUTER:
            return "duarouter"
        elif self == SumoApplication.NETCONVERT:
            return "netconvert"
        else:
            raise ValueError(f"Unknown application: {self}")


class SumoTool(Enum):
    RANDOM_TRIPS = auto()

    def get_tool_name(self) -> str:
        if self == SumoTool.RANDOM_TRIPS:
            return "randomTrips"
        else:
            raise RuntimeError(f"Unknown tool: {self}")

    def get_tool_category(self) -> Optional[str]:
        return None


def _get_binary_path_with_which(binary_name: str) -> Optional[Path]:
    """
    Searchs for the binary path of :param:`binary_name` by consulting 'which'.
    """
    binary_path_raw = shutil.which(binary_name)
    if binary_path_raw is not None and len(binary_path_raw) > 0:
        return Path(binary_path_raw)

    return None


def _get_binary_path_from_sumo_home(binary_name: str) -> Optional[Path]:
    """
    Tries to construct the binary path for :param:`binary_name` from the 'SUMO_HOME' environment variable and checks if the path exists.
    """
    sumo_home_path = os.getenv("SUMO_HOME")
    if sumo_home_path is not None:
        binary_path = Path(sumo_home_path) / "bin" / binary_name
        if binary_path.exists():
            return binary_path

    return None


def _get_binary_path(binary_name: str) -> Path:
    """
    Tries to find the path for :param:`binary_name` by consulting which and 'SUMO_HOME' in that order.
    """
    binary_path = _get_binary_path_with_which(binary_name)
    if binary_path is not None:
        _LOGGER.debug(f"Found binary {binary_name} at {binary_path} using 'which'")
        return binary_path

    binary_path = _get_binary_path_from_sumo_home(binary_name)
    if binary_path is not None:
        _LOGGER.debug(f"Found binary {binary_name} at {binary_path} using 'SUMO_HOME'")
        return binary_path

    raise RuntimeError(f"Unable to find the binary for '{binary_name}'")


def get_path_for_sumo_application(
    application_or_application_name: Union[str, SumoApplication],
) -> Path:
    """
    Searches for the binary of :param`application_name` in the well-known locations for SUMO binaries.

    :param application_name: The name of the SUMO application (e.g. netconvert)

    :returns: The path to the application. The path is quaranteed to exist.
    :raises RuntimeError: If no binary could be found.
    """
    if isinstance(application_or_application_name, SumoApplication):
        application_name = application_or_application_name.get_application_name()
    else:
        application_name = application_or_application_name

    binary_path = _get_binary_path(application_name)
    return binary_path


def get_path_for_sumo_tool(tool_name: str, category: Optional[str] = None) -> Path:
    """
    Searches for the :param:`tool_name` in the SUMO well-known tool path.

    :param tool_name: The name of the tool (e.g. randomTrips or randomTrips.py). Suffix '.py' is optional.

    :returns: The path to the tool. The path is quaranteed to exist.
    :raises RuntimeError: If the tool could be not found.
    """
    if not tool_name.endswith(".py"):
        tool_name = tool_name + ".py"

    sumo_home_path_raw = os.getenv("SUMO_HOME")
    if sumo_home_path_raw is None:
        raise RuntimeError(
            f"Failed to get path for SUMO tool '{tool_name}': The environment variable 'SUMO_HOME' is not set, but is required to use SUMO tools!"
        )

    sumo_tools_path = Path(sumo_home_path_raw) / "tools"
    if category is not None:
        sumo_tools_path /= category

        if not sumo_tools_path.exists():
            raise RuntimeError(
                f"Failed to get path for SUMO tool '{tool_name}': The category {category} does not exist at {sumo_tools_path}!"
            )

    tool_path = sumo_tools_path / tool_name
    if not tool_path.exists():
        raise RuntimeError(
            f"Failed to get path for SUMO tool '{tool_name}': Expected the tool to be at path {tool_path}, but the path does not exist. Did you specify the correct category for the tool?"
        )

    _LOGGER.debug(f"Found script for SUMO tool '{tool_name}' at {tool_path}")

    return tool_path


def get_sumo_gui_binary_path() -> Path:
    """
    Searches for the 'sumo-gui' binary at the most common locations on the current system.

    :returns: The path to the 'sumo-gui' binary. The path is guaranteed to be valid.
    :raises RuntimeError: If no 'sumo-gui' binary could be found on the system.
    """
    return get_path_for_sumo_application("sumo-gui")


def get_sumo_binary_path() -> Path:
    """
    Searches for the 'sumo' binary at the most common locations on the current system.

    :returns: The path to the 'sumo' binary. The path is guaranteed to be valid.
    :raises RuntimeError: If no 'sumo' binary could be found on the system.
    """
    return get_path_for_sumo_application("sumo")


def execute_sumo_tool(tool: SumoTool, args: list[str]) -> str | None:
    python_executable = shutil.which("python")
    if python_executable is None:
        return None

    tool_path = str(
        get_path_for_sumo_tool(tool.get_tool_name(), tool.get_tool_category())
    )
    cmd = [python_executable, tool_path]
    cmd.extend(args)
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode > 0:
            _LOGGER.error(
                f"Invocation of SUMO tool {tool.get_tool_name()} failed with status code {result.returncode}. Enable debug logging to see its output."
            )
            _LOGGER.debug(result.stdout.decode())
            return None
        return result.stdout.decode()
    except subprocess.CalledProcessError as e:
        _LOGGER.error(f"Invocation of SUMO tool {tool.get_tool_name()} failed: {e}")
        return None


def execute_sumo_application(
    application: SumoApplication, args: list[str]
) -> str | None:
    application_executable = get_path_for_sumo_application(application)
    cmd = [str(application_executable)]
    cmd.extend(args)
    _LOGGER.debug("Executing %s", " ".join(cmd))
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode > 0:
            _LOGGER.error(
                f"Invocation of SUMO application {application.get_application_name()} failed with status code {result.returncode}. Enable debug logging to see its output."
            )
            _LOGGER.debug(result.stdout.decode())
            return None
        return result.stdout.decode()
    except subprocess.CalledProcessError as e:
        _LOGGER.error(
            f"Invocation of SUMO application {application.get_application_name()} failed: {e}"
        )
        return None


def create_new_scenario_with_metadata_from_old_scenario(
    scenario: Scenario,
) -> Scenario:
    """
    Create a new scenario from an old scenario and include all its metadata.

    :param scenario: The old scenario, from which the metadata will be taken

    :returns: The new scenario with all metadata, which is safe to modify.
    """
    new_scenario = Scenario(
        dt=scenario.dt,
        # The following metadata values are all objects. As they could be arbitrarily modified in-place they need to be copied.
        scenario_id=copy.deepcopy(scenario.scenario_id),
        location=copy.deepcopy(scenario.location),
        tags=copy.deepcopy(scenario.tags),
        # Author, affiliation and source are plain strings and do not need to be copied
        author=scenario.author,
        affiliation=scenario.affiliation,
        source=scenario.source,
    )

    return new_scenario
