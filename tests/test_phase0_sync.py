"""Dependency-free invariants for the Phase-0 source synchronization."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def _quoted_value(text: str, key: str) -> str:
    match = re.search(rf'^\s*{re.escape(key)}\s*=\s*"([^"]*)"\s*$', text, re.MULTILINE)
    assert match is not None, f"missing TOML key: {key}"
    return match.group(1)


def test_upstream_provenance_is_recorded():
    metadata = (ROOT / "upstream-sync.toml").read_text()
    assert _quoted_value(metadata, "synced_commit") == (
        "e712698a05a80728b06572819dcf044596707754"
    )
    assert _quoted_value(metadata, "repository").endswith(
        "learnsyslab/gym-pybullet-drones"
    )


def test_current_upstream_package_metadata():
    pyproject = (ROOT / "pyproject.toml").read_text()

    assert _quoted_value(pyproject, "version") == "2.1.0"
    assert _quoted_value(pyproject, "readme") == "README.md"
    assert _quoted_value(pyproject, "repository").endswith(
        "learnsyslab/gym-pybullet-drones"
    )
    assert re.search(r'^numpy\s*=\s*"\^2\.2"$', pyproject, re.MULTILINE)
    assert re.search(r'^gymnasium\s*=\s*"\^1\.2"$', pyproject, re.MULTILINE)
    assert re.search(r'^stable-baselines3\s*=\s*"\^2\.8"$', pyproject, re.MULTILINE)


def test_ros_source_packages_are_tracked():
    required = [
        ROOT / "ros2/pybullet_ros/package.xml",
        ROOT / "ros2/swarm_msgs/package.xml",
        ROOT / "ros2/swarm_msgs/msg/SwarmSensing.msg",
        ROOT / "ros2/swarm_tools/package.xml",
    ]
    assert all(path.is_file() for path in required)

    ignore = (ROOT / ".gitignore").read_text()
    assert "ros2/swarm_msgs/" not in ignore
    assert "ros2/swarm_tools/" not in ignore
    assert "ros2/build/" in ignore
    assert "ros2/install/" in ignore
    assert "ros2/log/" in ignore


def test_container_install_matches_current_readme_metadata():
    dockerfile = (ROOT / "Dockerfile").read_text()
    compose = (ROOT / "docker-compose.yml").read_text()

    assert "COPY pyproject.toml README.md ./" in dockerfile
    assert "/opt/miniconda3/etc/profile.d/conda.sh" in dockerfile
    assert "PYTHONPATH /opt/miniconda3/envs/ros_env/bin/python" not in dockerfile
    assert "image: ros-pybullet-drones:humble" in compose


def test_downstream_cf2x_torque_correction_is_preserved():
    source = (ROOT / "gym_pybullet_drones/envs/BaseAviary.py").read_text()
    assert "elif self.DRONE_MODEL==DroneModel.CF2X:" in source
    assert "x_torque = - (forces[0] + forces[1] - forces[2] - forces[3])" in source


def test_new_upstream_observation_modes_are_present():
    source = (ROOT / "gym_pybullet_drones/utils/enums.py").read_text()
    assert 'DEP = "dep"' in source
    assert 'ALL = "all"' in source
