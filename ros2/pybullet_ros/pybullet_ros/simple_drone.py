"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ ros2 run pybullet_ros drone.py

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
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 3
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_GUIDANCE_FREQ_HZ = 6
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

class SimpleDrone(Node):
    def __init__(self,_node):
        super().__init__(_node)
        drone=DEFAULT_DRONES

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1/DEFAULT_GUIDANCE_FREQ_HZ #[s]
        self.timer = self.create_timer(timer_period,self._timer_callback)

        if drone in [DroneModel.CF2X, DroneModel.CF2P]:
            self.action = np.zeros(4)
            self.obs = np.zeros(20)

        self.power_on = False
        
        self.step_counter = 0
        self.wp_counters = 0

        '''
        # tf
        self._suffix = _node
        self.base_tf = TransformStamped()
        self.odom_tf = TransformStamped()
        '''

        self.PYB_CLIENT = None
        self.INIT_XYZS = None
        self.INIT_RPYS = None
        self.CURR_XYZS = None
        self.CURR_RPYS = None
        self.TARGET_POS = None
        self.NUM_WP = 0
        self.NUM_WP_ALL = 0
        pass
    
    def setEnv(self,i_num,R,H,H_STEP):
        #### Initialize the simulation #############################
        self.INIT_XYZS = np.array([R*np.cos((i_num/6.)*2*np.pi +np.pi/2), 
                                   R*np.sin((i_num/6.)*2*np.pi +np.pi/2)-R, 
                                   H+i_num*H_STEP])
        self.INIT_RPYS = np.array([0, 0, i_num*(np.pi/2)/DEFAULT_NUM_DRONES])
        print(self.INIT_XYZS)
        print(self.INIT_RPYS)

        '''
        ## set base_tf
        self.base_tf.header.stamp = self.get_clock().now()
        self.base_tf.header.frame_id = 'world'
        self.base_tf.child_frame_id = 'base_'+self._suffix
        '''

        #### Initialize a circular trajectory ######################
        PERIOD = 12
        HOLIZON = 1    # 3 step of velocity controller

        self.NUM_WP_ALL = DEFAULT_CONTROL_FREQ_HZ*PERIOD
        TRAJECTORY_ALL =    np.zeros((self.NUM_WP_ALL,3))
        self.TRAJECTORY_ALL = TRAJECTORY_ALL
        for i in range(self.NUM_WP_ALL):
            TRAJECTORY_ALL[i,:] =   R*np.cos((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)  +0, \
                                    R*np.sin((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)-R+0, \
                                    0
            '''
            TRAJECTORY_ALL[i,:] =   R*np.cos((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)  +self.INIT_XYZS[0], \
                                    R*np.sin((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)-R+self.INIT_XYZS[1], \
                                    0
            '''

        self.step_counter = int((i_num*self.NUM_WP_ALL/6)%self.NUM_WP_ALL)
        print(self.step_counter)

        self.NUM_WP = int(DEFAULT_CONTROL_FREQ_HZ/DEFAULT_GUIDANCE_FREQ_HZ)*HOLIZON
        self.TARGET_POS = np.zeros((self.NUM_WP,3))
        self.TARGET_POS = TRAJECTORY_ALL[self.step_counter:(self.NUM_WP+self.step_counter),:]

    def setOdom(self):
        t_stamped = TransformStamped()
        t_stamped.header.stamp = self.get_clock().now()
        t_stamped.header.frame_id = 'hoge'
        t_stamped.child_frame_id = 'fuga'


    def _timer_callback(self):
        if self.power_on==False:
            print("None")
            return
        #### Step the controller ###################################
        self.step_counter += self.wp_counters
        remaining_points = self.NUM_WP_ALL - self.step_counter

        if  remaining_points >= self.NUM_WP:
            self.TARGET_POS = self.TRAJECTORY_ALL[self.step_counter:(self.NUM_WP+self.step_counter),:]
        else:
            self.TARGET_POS[:remaining_points,:] = self.TRAJECTORY_ALL[self.step_counter:,:]
            self.TARGET_POS[remaining_points:,:] = self.TRAJECTORY_ALL[:(self.NUM_WP-remaining_points),:]
            self.step_counter = self.NUM_WP-remaining_points
        self.wp_counters = 0

class SimpleWorld(Node):
    def __init__(self):
        super().__init__('simple_copter_sim')
        
        timer_period = 1/DEFAULT_CONTROL_FREQ_HZ #[s]
        self.timer = self.create_timer(timer_period,self._timer_callback)

        # simulator
        self.env = None
        self.action = None
        self.ctrl = None
        self.step_counters = None
        self.logger = None
        self.num_drones = 0
        self.sim_step = 0
        self.time = 0

        self.PYB_CLIENT = None
        self.INIT_XYZS = []
        self.INIT_RPYS = []
        self.NUM_WP = 0

        # drone_ptr
        self.droneList = []
    
    def setDroneInWorld(self,_drone):
        if not isinstance(_drone, SimpleDrone):
            raise TypeError(f"Argument must be of type {SimpleDrone.__name__}, but got {type(_drone).__name__}.")
        self.droneList.append(_drone)
        self.num_drones = self.num_drones + 1

    def setEnv(self,
            drone=DEFAULT_DRONES,
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
        if self.num_drones == 0:
            print("error")
            return
        else:
            num_drones = self.num_drones
        self.INIT_XYZS = np.zeros((num_drones,3))
        self.INIT_RPYS = np.zeros((num_drones,3))

        for i in range(num_drones):
            self.INIT_XYZS[i,:] = self.droneList[i].INIT_XYZS
            self.INIT_RPYS[i,:] = self.droneList[i].INIT_RPYS

        self.env = CtrlAviary(drone_model=drone,
                    num_drones=num_drones,
                    initial_xyzs=self.INIT_XYZS,
                    initial_rpys=self.INIT_RPYS,
                    physics=physics,
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
        for i in range(num_drones):
            self.droneList[i].power_on=True

    def _timer_callback(self):
        #### Update Obs ###################################
        obs, reward, terminated, truncated, info = self.env.step(self.action)
        for j in range(self.num_drones):
            self.droneList[j].obs = obs[j,:]

        #### Compute control for the current way point #############
        for j in range(self.num_drones):
            self.action[j, :], _, _ = self.ctrl[j].computeControlFromState(
                                    control_timestep=self.env.CTRL_TIMESTEP,
                                    state=obs[j],
                                    target_pos=np.hstack([self.droneList[j].TARGET_POS[self.droneList[j].wp_counters, :2], self.INIT_XYZS[j, 2]]),
                                    target_rpy=self.INIT_RPYS[j, :]
                                    )
        #### Go to the next way point and loop #####################
        for j in range(self.num_drones):
            if self.droneList[j].wp_counters < self.droneList[j].NUM_WP-1:
                self.droneList[j].wp_counters = self.droneList[j].wp_counters + 1

        #### Printout ##############################################
        #### Log the simulation ####################################
        self.env.render()

        for j in range(self.num_drones):
            self.logger.log(drone=j,  
                        timestamp=self.time/self.env.CTRL_FREQ,
                        state=obs[j],
                        control=np.hstack([self.droneList[j].TARGET_POS[self.droneList[j].wp_counters, :2], self.INIT_XYZS[j, 2], self.INIT_RPYS[j, :], np.zeros(6)])
                        )

        #### Sync the simulation ###################################
        self.time += 1
        pass

    def plot(self):
        self.logger.plot()

def main():
    try:
        rclpy.init()
        exec = SingleThreadedExecutor()

        world_node = SimpleWorld()
        exec.add_node(world_node)

        for i in range(DEFAULT_NUM_DRONES):
            _d = SimpleDrone('drone_'+str(i))
            world_node.setDroneInWorld( _d )
            world_node.droneList[i].setEnv(i,.3,.1,.05)
            exec.add_node(world_node.droneList[i])
        world_node.setEnv()
        exec.spin()
    except(KeyboardInterrupt,ExternalShutdownException):
        pass

    exec.shutdown()
    world_node.plot()
    for i in range(DEFAULT_NUM_DRONES):
        world_node.droneList[i].destroy_node()
    world_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()