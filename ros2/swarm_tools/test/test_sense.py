"""Unit tests for the dependency-light swarm helper classes."""

import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from swarm_msgs.msg import SwarmSensing
from swarm_tools.swarm_test import Control, Sense, SwarmDrone


def _identity_pose() -> Pose:
    pose = Pose()
    pose.orientation.w = 1.0
    return pose


def test_sense_callbacks_store_self_and_neighbor_measurements():
    sense = Sense("drone0")
    self_msg = PoseArray()
    self_msg.header.stamp.sec = 10
    self_msg.poses = [_identity_pose()]
    sense.sense_self_callback(self_msg)

    neighbor_msg = SwarmSensing()
    neighbor_msg.header.stamp.sec = 11
    neighbor_msg.neighbors = ["drone1"]
    neighbor_msg.poses = [_identity_pose()]
    sense.sense_adjacents_callback(neighbor_msg)

    assert len(sense.self_states) == 1
    assert sense.neighbors == ["drone1"]
    assert sense.neighbor_states["drone1"].orientation.w == 1.0
    assert sense.state_times["drone0"].sec == 10
    assert sense.state_times["drone1"].sec == 11


def test_control_returns_complete_placeholder_horizon():
    msg = Control().swarm_ctrl(np.zeros(7), np.empty((0, 7)))
    assert len(msg.poses) == 6
    assert all(pose.orientation.w == 1.0 for pose in msg.poses)


def test_rotation_estimate_compensates_observer_rotation():
    identity = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float64)
    quarter_turn = np.array(
        [0, 0, 0, 0, 0, np.sqrt(0.5), np.sqrt(0.5)],
        dtype=np.float64,
    )
    estimated = SwarmDrone.estimate_rotation(identity, quarter_turn, identity)
    expected = np.array(
        [0, 0, 0, 0, 0, -np.sqrt(0.5), np.sqrt(0.5)],
        dtype=np.float64,
    )
    np.testing.assert_allclose(estimated, expected, atol=1e-12)
