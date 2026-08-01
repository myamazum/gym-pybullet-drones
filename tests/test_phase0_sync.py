"""Dependency-free invariants for the Phase-0 source synchronization."""

import re
import xml.etree.ElementTree as etxml
from pathlib import Path

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
    assert _quoted_value(metadata, "strategy") == "merge"


def test_current_upstream_package_metadata():
    pyproject = (ROOT / "pyproject.toml").read_text()

    assert _quoted_value(pyproject, "version") == "2.1.0"
    assert _quoted_value(pyproject, "readme") == "README.md"
    assert _quoted_value(pyproject, "repository").endswith(
        "myamazum/gym-pybullet-drones"
    )
    assert re.search(r'^numpy\s*=\s*"\^2\.2"$', pyproject, re.MULTILINE)
    assert re.search(r'^gymnasium\s*=\s*"\^1\.2"$', pyproject, re.MULTILINE)
    assert re.search(r'^stable-baselines3\s*=\s*"\^2\.8"$', pyproject, re.MULTILINE)


def test_ros_source_packages_are_present_and_not_ignored():
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
    dockerignore = (ROOT / ".dockerignore").read_text()
    compose = (ROOT / "docker-compose.yml").read_text()

    assert "COPY pyproject.toml README.md ./" in dockerfile
    assert "/opt/miniconda3/etc/profile.d/conda.sh" in dockerfile
    assert "ros-humble-tf-transformations" in dockerfile
    assert "ros-humble-xacro" in dockerfile
    assert "'setuptools>=77,<80'" in dockerfile
    assert "'pytest>=7,<8'" in dockerfile
    assert "colcon-notification" in dockerfile
    assert "python -m pip check" in dockerfile
    assert "PYTHONPATH /opt/miniconda3/envs/ros_env/bin/python" not in dockerfile
    assert "!gym_pybullet_drones/**" in dockerignore
    assert "**/__pycache__/" in dockerignore
    assert "!temp" not in dockerignore
    assert "image: ros-pybullet-drones:humble" in compose
    assert not re.search(r"^version\s*:", compose, re.MULTILINE)


def test_ci_validates_pull_requests_with_current_actions():
    workflow = (ROOT / ".github/workflows/test.yml").read_text()
    assert re.search(r"^\s{2}pull_request:$", workflow, re.MULTILINE)
    checkout = re.search(r"actions/checkout@v(\d+)", workflow)
    setup_python = re.search(r"actions/setup-python@v(\d+)", workflow)
    assert checkout and int(checkout.group(1)) >= 7
    assert setup_python and int(setup_python.group(1)) >= 7
    assert "python-version: ['3.10', '3.12']" in workflow
    assert "'pytest>=9,<10'" in workflow
    assert "python -m build" in workflow


def test_ros_runtime_dependencies_and_entry_points_are_declared():
    pybullet_manifest = etxml.parse(ROOT / "ros2/pybullet_ros/package.xml").getroot()
    pybullet_dependencies = {
        element.text
        for element in pybullet_manifest
        if element.tag in {"depend", "exec_depend"}
    }
    assert {
        "ament_index_python",
        "launch",
        "launch_ros",
        "rclpy",
        "tf2_ros",
        "tf_transformations",
    } <= pybullet_dependencies

    swarm_manifest = etxml.parse(ROOT / "ros2/swarm_tools/package.xml").getroot()
    swarm_dependencies = {
        element.text
        for element in swarm_manifest
        if element.tag in {"depend", "exec_depend"}
    }
    assert "rclpy" in swarm_dependencies
    assert "rclcpp" not in swarm_dependencies

    swarm_setup = (ROOT / "ros2/swarm_tools/setup.py").read_text()
    assert "swarm_control = swarm_tools.swarm_test:main" in swarm_setup


def test_tf_bridge_contract_uses_simulator_owned_dynamic_frames():
    source = (
        ROOT / "ros2/pybullet_ros/pybullet_ros/simple_drone_tf.py"
    ).read_text()
    urdf = (ROOT / "ros2/pybullet_ros/urdf/drone.urdf.xacro").read_text()
    launch = (ROOT / "ros2/pybullet_ros/launch/tf_drone.launch.py").read_text()

    assert 'self.baselink_tf.header.frame_id = "world"' in source
    assert 'self.baselink_tf.child_frame_id = "baselink_" + self._suffix' in source
    assert "self.baselink_tf.header.stamp =" in source
    assert "base_tf" not in source
    assert "odom_tf" not in source
    assert "lookup_transform" not in source

    assert '<joint name="baselink_${suffix}_joint" type="floating">' in urdf
    assert '<parent link="${parent}"/>' in urdf
    assert "base_${suffix}" not in urdf
    assert "odom_${suffix}" not in urdf

    assert 'DeclareLaunchArgument("gui", default_value="false")' in launch
    assert 'DeclareLaunchArgument("plot", default_value="false")' in launch


def test_new_upstream_observation_modes_are_present():
    source = (ROOT / "gym_pybullet_drones/utils/enums.py").read_text()
    assert 'DEP = "dep"' in source
    assert 'ALL = "all"' in source
