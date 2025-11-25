import os
import shutil
from pathlib import Path
from typing import Optional

__all__ = ["get_sumo_gui_binary_path", "get_sumo_binary_path"]


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
    tries to find the path for :param:`binary_name` by consulting which and 'SUMO_HOME' in that order.
    """
    binary_path = _get_binary_path_with_which(binary_name)
    if binary_path is not None:
        return binary_path

    binary_path = _get_binary_path_from_sumo_home(binary_name)
    if binary_path is not None:
        return binary_path

    raise RuntimeError(f"Unable to find the binary for '{binary_name}'")


def get_sumo_gui_binary_path() -> Path:
    """
    searchs for the 'sumo-gui' binary at the most common locations on the current system.

    :returns: The path to the 'sumo-gui' binary. The path is guaranteed to be valid.
    :raises RuntimeError: If no 'sumo-gui' binary could be found on the system.
    """
    return _get_binary_path("sumo-gui")


def get_sumo_binary_path() -> Path:
    """
    searchs for the 'sumo' binary at the most common locations on the current system.

    :returns: The path to the 'sumo' binary. The path is guaranteed to be valid.
    :raises RuntimeError: If no 'sumo' binary could be found on the system.
    """
    return _get_binary_path("sumo")
