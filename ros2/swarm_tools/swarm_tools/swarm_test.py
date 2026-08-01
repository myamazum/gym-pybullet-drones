"""Experimental ROS 2 swarm sensing and control node."""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from swarm_msgs.msg import SwarmSensing

DEFAULT_DIM_STATE = 7
DEFAULT_LOG_SELF = 10
DEFAULT_SWARM_CONTROL_FREQ_HZ = 10
DEFAULT_WP_STEP = 5


def _stamp_to_seconds(stamp) -> float:
    """Convert a ROS time message to floating-point seconds."""

    return stamp.sec + stamp.nanosec / 1e9


class Sense:
    """Hold the latest self and adjacent-drone measurements."""

    def __init__(self, identifier: str, group: set[str] | None = None):
        """Initialize sensing state for one drone identifier."""

        self.identifier = identifier
        self.group = group if group is not None else set()
        self.neighbors: list[str] = []
        self.self_states: list[Pose] = []
        self.neighbor_states: dict[str, Pose] = {}
        self.state_times = {}

    def set_sensing_set(self, group: list[str] | None = None) -> None:
        """Replace the ordered set of sensed neighbors."""

        self.neighbors = group if group is not None else []

    def reset_measurements(self) -> None:
        """Discard measurements after a control update."""

        self.self_states = []
        self.neighbor_states = {}
        self.state_times = {}

    def sense_self_callback(self, msg: PoseArray) -> None:
        """Store the most recent self-state history message."""

        self.state_times[self.identifier] = msg.header.stamp
        self.self_states = list(msg.poses)

    def sense_adjacents_callback(self, msg: SwarmSensing) -> None:
        """Store the most recent measurement for each adjacent drone."""

        for neighbor, pose in zip(msg.neighbors, msg.poses):
            self.state_times[neighbor] = msg.header.stamp
            self.neighbor_states[neighbor] = pose
        self.neighbors = list(msg.neighbors)


class Control:
    """Produce placeholder waypoint messages for the swarm controller."""

    def swarm_ctrl(self, self_state: np.ndarray, neighbor_states: np.ndarray) -> PoseArray:
        """Return a zero waypoint horizon until a control law is selected."""

        del self_state, neighbor_states
        msg = PoseArray()
        msg.poses = [Pose() for _ in range(DEFAULT_WP_STEP + 1)]
        for pose in msg.poses:
            pose.orientation.w = 1.0
        return msg


class SwarmDrone(Node):
    """Align asynchronous swarm measurements and publish a control horizon."""

    def __init__(self, node_prefix: str = "drone", drone_id: str = "0"):
        """Create one experimental swarm-control node."""

        self.node_name = node_prefix + drone_id
        super().__init__(self.node_name)
        self.swarm_control = Control()
        self.swarm_sense = Sense(self.node_name)
        self.neighbors: list[str] = []

        self.self_states = np.empty((0, DEFAULT_DIM_STATE), dtype=np.float64)
        self.neighbor_states: dict[str, np.ndarray] = {}
        self.sensing_times: dict[str, float] = {}

        self.control_publisher = self.create_publisher(
            PoseArray,
            "swarm_control_topic",
            3,
        )
        self.self_subscription = self.create_subscription(
            PoseArray,
            "self_sensing_topic",
            self.swarm_sense.sense_self_callback,
            1,
        )
        self.neighbor_subscription = self.create_subscription(
            SwarmSensing,
            "swarm_sensing_topic",
            self.swarm_sense.sense_adjacents_callback,
            1,
        )

        timer_period = 1 / DEFAULT_SWARM_CONTROL_FREQ_HZ
        self.timer = self.create_timer(timer_period, self._timer_callback)

    def _timer_callback(self) -> None:
        if not self.swarm_sense.self_states:
            return
        if self.node_name not in self.swarm_sense.state_times:
            return

        self.neighbors = [
            neighbor
            for neighbor in self.swarm_sense.neighbors
            if neighbor in self.swarm_sense.neighbor_states
            and neighbor in self.swarm_sense.state_times
        ]
        self_states = np.array([
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            for pose in reversed(self.swarm_sense.self_states[-DEFAULT_LOG_SELF:])
        ])
        neighbor_states = np.array([
            [
                self.swarm_sense.neighbor_states[neighbor].position.x,
                self.swarm_sense.neighbor_states[neighbor].position.y,
                self.swarm_sense.neighbor_states[neighbor].position.z,
                self.swarm_sense.neighbor_states[neighbor].orientation.x,
                self.swarm_sense.neighbor_states[neighbor].orientation.y,
                self.swarm_sense.neighbor_states[neighbor].orientation.z,
                self.swarm_sense.neighbor_states[neighbor].orientation.w,
            ]
            for neighbor in self.neighbors
        ], dtype=np.float64).reshape((-1, DEFAULT_DIM_STATE))
        sensing_times = np.array([
            *[
                _stamp_to_seconds(self.swarm_sense.state_times[neighbor])
                for neighbor in self.neighbors
            ],
            _stamp_to_seconds(self.swarm_sense.state_times[self.node_name]),
        ])

        self.set_current_state(self_states, neighbor_states, sensing_times)
        control_msg = self.swarm_control.swarm_ctrl(
            self.self_states[0],
            np.array(
                list(self.neighbor_states.values()),
                dtype=np.float64,
            ).reshape((-1, DEFAULT_DIM_STATE)),
        )
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = "world"
        self.control_publisher.publish(control_msg)
        self.swarm_sense.reset_measurements()

    def set_current_state(
        self,
        self_states: np.ndarray,
        neighbor_states: np.ndarray,
        sensing_times: np.ndarray,
    ) -> None:
        """Align neighbor orientations to the nearest self-state timestamp."""

        self_times = sensing_times[-1] - (
            np.arange(len(self_states)) / DEFAULT_SWARM_CONTROL_FREQ_HZ
        )
        for index, neighbor in enumerate(self.neighbors):
            nearest = int(np.argmin(np.abs(self_times - sensing_times[index])))
            neighbor_states[index] = self.estimate_rotation(
                self_states[nearest],
                self_states[0],
                neighbor_states[index],
            )

        self.self_states = self_states
        self.neighbor_states = {
            neighbor: neighbor_states[index]
            for index, neighbor in enumerate(self.neighbors)
        }
        self.sensing_times = {
            neighbor: sensing_times[index]
            for index, neighbor in enumerate(self.neighbors)
        }
        self.sensing_times[self.node_name] = sensing_times[-1]

    @staticmethod
    def estimate_rotation(
        past_self_state: np.ndarray,
        latest_self_state: np.ndarray,
        past_neighbor_state: np.ndarray,
    ) -> np.ndarray:
        """Compensate a neighbor orientation for the observer's rotation."""

        past_self_rotation = Rotation.from_quat(past_self_state[3:7])
        latest_self_rotation = Rotation.from_quat(latest_self_state[3:7])
        relative_rotation = latest_self_rotation.inv() * past_self_rotation
        past_neighbor_rotation = Rotation.from_quat(past_neighbor_state[3:7])
        estimated_rotation = relative_rotation * past_neighbor_rotation

        return np.concatenate([
            past_neighbor_state[0:3],
            estimated_rotation.as_quat(),
        ])


def main(args=None) -> None:
    """Run a single experimental swarm-control node."""

    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = SwarmDrone()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
