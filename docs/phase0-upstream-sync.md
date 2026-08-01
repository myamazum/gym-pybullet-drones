# Phase 0: upstream synchronization record

## 1. Purpose

This phase establishes a reproducible starting point before refactoring the ROS 2 integration. It combines:

1. the supplied `gym-pybullet-drones-main(2).zip` snapshot;
2. the supplied `ros-puybullet-drone-dev-develop.zip` development snapshot; and
3. `learnsyslab/gym-pybullet-drones:main` at commit `e712698a05a80728b06572819dcf044596707754` (2026-07-11).

The ZIP archives do not contain Git metadata, so their exact source commit cannot be proven from the archives alone. The upstream synchronization range was therefore anchored at the known common upstream commit `5404871f32697b7c568d9e2520368e81d46f0ab3` and applied through `e712698a05a80728b06572819dcf044596707754`.

## 2. Merge policy

- Current upstream implementation is authoritative for the Python simulator, dependencies, examples, and CI.
- ROS 2 packages and container integration from the supplied fork are retained as downstream additions.
- A downstream change is preserved when it changes intended behavior rather than merely reflecting an old upstream version.
- Missing files in an archive are not treated as deletions without supporting evidence.

## 3. Upstream changes incorporated

- Package version and dependency update to the current `2.1.0` metadata.
- Repository links moved from `utiasDSL` to `learnsyslab`.
- Gymnasium seed initialization in `BaseAviary.reset()`.
- Integer-division correction in `DSLPIDControl._one23DInterface()`.
- Corrected URDF parameter lists in `BaseControl` and `CTBRControl`.
- Added `DEP` and `ALL` reinforcement-learning observation modes.
- Added MRAC controller and example.
- Added trained-policy playback example.
- Updated learning example output/plot behavior.
- Narrowed bare exception handlers in the SITL examples.
- Current test workflow and Dependabot configuration.
- Removed the obsolete release workflow and the obsolete `build_project.sh` / `pypi_description.md` packaging helpers.

## 4. Development changes retained

- `ros2/pybullet_ros`.
- `ros2/swarm_msgs`.
- `ros2/swarm_tools`.
- NVIDIA CUDA + RoboStack ROS 2 Humble container integration.
- Docker Compose source build.
- Downstream MIT attribution.
- CF2X X-axis torque-sign correction in `BaseAviary._physics()`.

The development archive did not contain `ros2/swarm_msgs`, while the supplied original snapshot did. Its own `.gitignore` also ignored that source directory. This was treated as archive omission rather than a requested deletion, so `swarm_msgs` remains tracked.

The development archive added `play.py`, but that file is also present in current upstream. The upstream-equivalent implementation is therefore retained without a separate downstream fork.

## 5. Integration corrections made in Phase 0

These corrections are required solely to connect the two source lines:

- The current upstream `pyproject.toml` references `README.md`; the Dockerfile now copies `README.md` before editable installation.
- The Miniconda activation path is `/opt/miniconda3`, not `/opt/conda`.
- The Python environment is exposed through `PATH`; an interpreter path is no longer assigned to `PYTHONPATH`.
- Compose now assigns a valid image tag and builds from the local Dockerfile.
- ROS source packages remain tracked; only colcon output directories are ignored.

## 6. Deliberately deferred work

The following are refactoring targets, not Phase-0 synchronization changes:

- Ubuntu 22.04 / ROS 2 Humble to Ubuntu 24.04 / ROS 2 Jazzy migration.
- Evaluation of RoboStack versus native ROS installation.
- ROS node lifecycle, executor, QoS, namespace, and launch restructuring.
- `tf2` message-volume and shared-memory analysis.
- Replacement of `PoseArray` waypoint transport with explicit typed interfaces.
- Completion of `swarm_tools` package metadata and executable entry points.
- Resolution of the current upstream dependency jump (notably NumPy 2.x) against the RoboStack binary environment.
- End-to-end container build and GPU/GUI validation on the target machine.

## 7. Validation commands

```bash
python -m compileall gym_pybullet_drones ros2
pytest -q tests

docker compose config
# Network/GPU/ROS validation on the target host:
docker compose build ros
docker compose run --rm ros bash
```
