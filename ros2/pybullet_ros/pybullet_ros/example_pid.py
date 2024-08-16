"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ ros2 run pybullet_ros test_pid

Notes
-----
The drones move, at different altitudes, along cicular trajectories 
in the X-Y plane, around point (0, -.3).

"""
import math
import numpy as np

import pybullet as p

import rclpy
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 4
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 36
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

class SimpleCopter(Node):
    def __init__(self):
        super().__init__('simple_copter_sim')

        # Declare and acquire `turtlename` parameter
        self.copterName = self.declare_parameter(
            "copter_name", "mc1").get_parameter_value().string_value
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1/DEFAULT_SIMULATION_FREQ_HZ #[s]
        self.timer = self.create_timer(timer_period,self._timer_callback)

        # simulator
        self.env = None
        self.action = None
        self.ctrl = None
        self.wp_counters = None
        self.logger = None
        self.num_drones = 0

        self.PYB_CLIENT = None
        self.INIT_XYZS = None
        self.INIT_RPYS = None
        self.TARGET_POS = None
        self.NUM_WP = 0

    def setEnv(self,
            drone=DEFAULT_DRONES,
            num_drones=DEFAULT_NUM_DRONES,
            physics=DEFAULT_PHYSICS,
            gui=DEFAULT_GUI,
            record_video=DEFAULT_RECORD_VISION,
            plot=DEFAULT_PLOT,
            user_debug_gui=DEFAULT_USER_DEBUG_GUI,
            obstacles=DEFAULT_OBSTACLES,
            simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
            control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
            duration_sec=DEFAULT_DURATION_SEC,
            output_folder=DEFAULT_OUTPUT_FOLDER,
            colab=DEFAULT_COLAB):

        #### Initialize the simulation #############################
        self.num_drones=num_drones
        H = .5
        H_STEP = .1
        R = .3
        self.INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(num_drones)])
        self.INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])

        #### Initialize a circular trajectory ######################
        PERIOD = 12
        NUM_WP = control_freq_hz*PERIOD
        self.NUM_WP = NUM_WP
        self.TARGET_POS = np.zeros((NUM_WP,3))
        for i in range(NUM_WP):
            self.TARGET_POS[i, :] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+self.INIT_XYZS[0, 0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+self.INIT_XYZS[0, 1], 0
        self.wp_counters = np.array([int((i*NUM_WP/6)%NUM_WP) for i in range(num_drones)])

        self.env = CtrlAviary(drone_model=drone,
                    num_drones=num_drones,
                    initial_xyzs=self.INIT_XYZS,
                    initial_rpys=self.INIT_RPYS,
                    physics=Physics.PYB_DW,
                    #physics=physics,
                    neighbourhood_radius=10,
                    pyb_freq=simulation_freq_hz,
                    ctrl_freq=control_freq_hz,
                    gui=gui,
                    record=record_video,
                    obstacles=obstacles,
                    user_debug_gui=user_debug_gui
                    )
        #### Obtain the PyBullet Client ID from the environment ####
        self.PYB_CLIENT = self.env.getPyBulletClient()

        #### Initialize the logger #################################
        self.logger = Logger(logging_freq_hz=control_freq_hz,
                        num_drones=num_drones,
                        output_folder=output_folder,
                        colab=colab
                        )
        self.action = np.zeros((num_drones,4))

        #### Initialize the controllers ############################
        if drone in [DroneModel.CF2X, DroneModel.CF2P]:
            self.ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    def _timer_callback(self):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = self.env.step(self.action)

        #### Compute control for the current way point #############
        for j in range(self.num_drones):
            self.action[j, :], _, _ = self.ctrl[j].computeControlFromState(
                                    control_timestep=self.env.CTRL_TIMESTEP,
                                    state=obs[j],
                                    target_pos=np.hstack([self.TARGET_POS[self.wp_counters[j], 0:2], self.INIT_XYZS[j, 2]]),
                                    target_rpy=self.INIT_RPYS[j, :]
                                    )
        #### Go to the next way point and loop #####################
        for j in range(self.num_drones):
            self.wp_counters[j] = self.wp_counters[j] + 1 if self.wp_counters[j] < (self.NUM_WP-1) else 0

        #### Printout ##############################################
        self.env.render()

        #### Sync the simulation ###################################
        pass

def main():
    try:
        rclpy.init()
        exec = SingleThreadedExecutor()

        minimal_node = SimpleCopter()
        minimal_node.setEnv()

        exec.add_node(minimal_node)
        exec.spin()
    except(KeyboardInterrupt,ExternalShutdownException):
        pass
    
    exec.shutdown()
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()