"""Helpers for locating files bundled with :mod:`gym_pybullet_drones`."""

from importlib.resources import files


def asset_path(filename: str) -> str:
    """Return the filesystem path of an asset bundled with the package."""

    return str(files("gym_pybullet_drones").joinpath("assets", filename))
