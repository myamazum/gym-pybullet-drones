"""Runtime regressions for the Phase-0 upstream and ROS integration."""

from pathlib import Path

import numpy as np
import pybullet as p
import pytest

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary
from gym_pybullet_drones.examples.mrac import run as run_mrac
from gym_pybullet_drones.utils.enums import (
    ActionType,
    DroneModel,
    ObservationType,
    Physics,
)
from gym_pybullet_drones.utils.resources import asset_path


def test_packaged_asset_lookup_uses_stdlib_resources():
    assert Path(asset_path("cf2x.urdf")).is_file()
    assert Path(asset_path("cf2.dae")).is_file()


@pytest.mark.parametrize("env_type", [CtrlAviary, VelocityAviary])
def test_control_observations_match_declared_spaces(env_type):
    env = env_type(gui=False)
    try:
        obs, _ = env.reset(seed=42)
        assert env.observation_space.contains(obs)
        obs, *_ = env.step(np.zeros(env.action_space.shape, dtype=np.float32))
        assert env.observation_space.contains(obs)
    finally:
        env.close()


@pytest.mark.parametrize("obs_type", list(ObservationType))
def test_observations_match_declared_spaces(obs_type):
    env = HoverAviary(
        obs=obs_type,
        act=ActionType.RPM,
        ctrl_freq=30,
        gui=False,
    )
    try:
        obs, _ = env.reset(seed=42)
        assert env.observation_space.contains(obs)

        obs, *_ = env.step(np.zeros(env.action_space.shape, dtype=np.float32))
        assert env.observation_space.contains(obs)

        if obs_type in (ObservationType.RGB, ObservationType.DEP, ObservationType.ALL):
            assert env.IMG_CAPTURE_FREQ % env.PYB_STEPS_PER_CTRL == 0
            assert p.getNumBodies(physicsClientId=env.CLIENT) == env.NUM_DRONES + 5
    finally:
        env.close()


def test_cf2x_dynamics_uses_rotor_geometry_for_roll_torque():
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        physics=Physics.DYN,
        gui=False,
        obstacles=False,
        user_debug_gui=False,
    )
    try:
        env.reset(seed=42)
        rpm = env.HOVER_RPM
        obs, *_ = env.step(np.array([[rpm, rpm, 0, 0]], dtype=np.float32))
        expected_roll_rate = (
            -(2 * rpm**2 * env.KF)
            * (env.L / np.sqrt(2))
            * env.J_INV[0, 0]
            * env.PYB_TIMESTEP
        )
        np.testing.assert_allclose(
            obs[0, 13:16],
            np.array([expected_roll_rate, 0, 0]),
            rtol=1e-6,
            atol=1e-8,
        )
    finally:
        env.close()


def test_mrac_example_supports_multiple_drones(tmp_path):
    run_mrac(
        num_drones=2,
        gui=False,
        plot=False,
        duration_sec=0.05,
        output_folder=str(tmp_path),
    )
