> [!IMPORTANT]
> This repository is a ROS 2-enabled downstream fork of `learnsyslab/gym-pybullet-drones`.
> Phase 0 is synchronized with upstream `main` through commit
> `e712698a05a80728b06572819dcf044596707754` (2026-07-11), while retaining
> the ROS 2 packages and container environment.
> See [`docs/phase0-upstream-sync.md`](docs/phase0-upstream-sync.md) for the merge record.

> [!TIP]
> For research work with **symbolic dynamics and constraints**, also try [`safe-control-gym`](https://github.com/learnsyslab/safe-control-gym).
>
> For GPU-accelerated, **differentiable, JAX-based simulation**, also try [`crazyflow`](https://github.com/learnsyslab/crazyflow).
>
> For production-grade deployment of **ROS2 + PX4/ArduPilot + YOLO/LiDAR**, use [`aerial-autonomy-stack`](https://github.com/JacopoPan/aerial-autonomy-stack).

# gym-pybullet-drones — ROS 2-enabled fork

`gym-pybullet-drones` provides lightweight PyBullet environments for single- and multi-drone control, Gymnasium reinforcement learning, and Betaflight/Crazyflie SITL. This fork adds a ROS 2 workspace that exposes simulator state and control interfaces for multi-robot experiments.

The original IROS 2021 codebase remains available in the upstream `paper` and `master` branches.

<img src="gym_pybullet_drones/assets/helix.gif" alt="formation flight" width="325"> <img src="gym_pybullet_drones/assets/helix.png" alt="control info" width="425">

## Repository layout

```text
.
├── gym_pybullet_drones/    # Upstream simulator, controllers, environments, examples
├── ros2/
│   ├── pybullet_ros/       # Simulator/ROS 2 bridge and tf examples
│   ├── swarm_msgs/         # Custom swarm interface definitions
│   └── swarm_tools/        # Experimental multi-agent sensing/control utilities
├── Dockerfile              # CUDA + Miniconda + RoboStack ROS 2 Humble image
├── docker-compose.yml      # X11/GPU-enabled development service
└── docs/                   # Synchronization and design records
```

## Native Python installation

The upstream Python package targets Python 3.10.

```sh
git clone https://github.com/myamazum/gym-pybullet-drones.git
cd gym-pybullet-drones/

conda create -n drones python=3.10
conda activate drones

python -m pip install --upgrade pip
python -m pip install -e .
```

The synchronized dependency metadata currently follows upstream version `2.1.0`, including Gymnasium 1.2, Stable-Baselines3 2.8, NumPy 2.2, SciPy 1.15, and PyBullet 3.2.7-compatible constraints. The package repository points to this fork, but this downstream tree is not intended to be published to PyPI under the upstream version.

## ROS 2 container installation

The current downstream container remains based on Ubuntu 22.04, CUDA 12.4.1, and RoboStack ROS 2 Humble. Migration to Ubuntu 24.04 / ROS 2 Jazzy is intentionally deferred to the next refactoring phase.

Prerequisites on the host:

- Docker Engine with Compose v2;
- NVIDIA Container Toolkit for GPU passthrough;
- an X11 display when running PyBullet GUI or RViz.

Build and start the development container from the repository root:

```sh
docker compose build ros
xhost +local:docker  # grant only for the active development session
docker compose run --rm ros
```

Inside the container, the Conda environment is activated by `.bashrc`. If necessary, activate it explicitly, build the mounted ROS workspace, and source it:

```sh
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate ros_env

cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Do not source a system `/opt/ros/humble/setup.bash` in the same shell; this image uses the RoboStack environment under `/opt/miniconda3/envs/ros_env`.

### ROS 2 examples

```sh
# Original PID example as one ROS node
ros2 run pybullet_ros test_pid

# Multi-node-oriented simulator bridge
ros2 run pybullet_ros drone

# Simulator bridge with tf publication
ros2 run pybullet_ros drone_tf

# Launch robot_state_publisher / tf setup
ros2 launch pybullet_ros tf_drone.launch.py
```

The tf launch is headless by default. Add `gui:=true` (and optionally
`plot:=true`) when a display is available.

`swarm_tools` remains experimental. Its `ros2 run swarm_tools swarm_control`
entry point aligns sensing timestamps and publishes a zero placeholder waypoint
horizon; a mission-specific swarm control law still needs to be supplied.

## Python examples

### PID control

```sh
cd gym_pybullet_drones/examples/
python3 pid.py          # position and velocity reference
python3 pid_velocity.py # desired velocity reference
```

### Downwash effect

```sh
cd gym_pybullet_drones/examples/
python3 downwash.py
```

### Model-reference adaptive control

```sh
cd gym_pybullet_drones/examples/
python3 mrac.py
```

### Reinforcement learning with Stable-Baselines3 PPO

```sh
cd gym_pybullet_drones/examples/
python learn.py                   # single-drone hover at z == 1.0
python learn.py --multiagent true # two-drone hover

LATEST_MODEL=$(ls -t results | head -n 1)
python play.py --model_path "results/${LATEST_MODEL}/best_model.zip"
```

<img src="gym_pybullet_drones/assets/rl.gif" alt="rl example" width="375"> <img src="gym_pybullet_drones/assets/marl.gif" alt="marl example" width="375">

### Betaflight SITL (Ubuntu only)

```sh
git clone https://github.com/betaflight/betaflight
cd betaflight/
git checkout cafe727
make arm_sdk_install
make TARGET=SITL
cp ~/gym-pybullet-drones/gym_pybullet_drones/assets/eeprom.bin ./
obj/main/betaflight_SITL.elf
```

In another terminal:

```sh
conda activate drones
cd gym_pybullet_drones/examples/
python3 beta.py --num_drones 1
```

### `pycffirmware` Python bindings

Install [`pycffirmware`](https://github.com/learnsyslab/pycffirmware?tab=readme-ov-file#installation), then run:

```sh
cd gym_pybullet_drones/examples/
python3 cf.py
```

## Tests

```sh
# Python package tests
python -m pytest tests/

# Static syntax check, including ROS Python packages
python -m compileall gym_pybullet_drones ros2

# Compose model validation
docker compose config
```

The full ROS/GPU/GUI path must also be validated on a host with Docker, NVIDIA Container Toolkit, X11, and network access to the Conda channels.

## Citation

When using the simulator, cite the upstream IROS 2021 paper:

```bibtex
@INPROCEEDINGS{panerati2021learning,
      title={Learning to Fly---a Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control},
      author={Jacopo Panerati and Hehui Zheng and SiQi Zhou and James Xu and Amanda Prorok and Angela P. Schoellig},
      booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
      year={2021},
      pages={7512-7519},
      doi={10.1109/IROS51168.2021.9635857}
}
```

## License and provenance

The project is distributed under the MIT License. Upstream code is derived from `learnsyslab/gym-pybullet-drones`; downstream ROS 2, Docker, and physics modifications are identified in [`LICENSE`](LICENSE) and the Phase-0 synchronization record.
