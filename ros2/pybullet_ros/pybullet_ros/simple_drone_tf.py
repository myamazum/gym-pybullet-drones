"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ ros2 run pybullet_ros drone_tf

Notes
-----
The drones move, at different altitudes, along circular trajectories
in the X-Y plane, around point (0, -.3).

"""
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.Logger import Logger

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 4
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = False
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = False
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
        
        self.timer_counter = 0
        self.step_counter = 0
        self.wp_counters = 0

        # tf
        self._suffix = _node
        self.baselink_tf = TransformStamped()

        self.baselink_tf.header.frame_id = "world"
        self.baselink_tf.child_frame_id = "baselink_" + self._suffix

        self.PYB_CLIENT = None
        self.INIT_XYZS = np.zeros(3)
        self.INIT_RPYS = np.zeros(3)
        self.CURR_XYZS = None
        self.CURR_RPYS = None
        self.TARGET_POS = None
        self.NUM_WP = 0
        self.NUM_WP_ALL = 0
    
    def setEnv(self, i_num, R, H, H_STEP):
        #### Initialize the simulation #############################
        self.INIT_XYZS = np.array([R*np.cos((i_num/6.)*2*np.pi +np.pi/2),
                                   R*np.sin((i_num/6.)*2*np.pi +np.pi/2)-R,
                                   H+i_num*H_STEP])
        self.INIT_RPYS = np.array([0, 0, i_num*(np.pi/2)/DEFAULT_NUM_DRONES])
        self.publish_transform(
            self.INIT_XYZS,
            quaternion_from_euler(*self.INIT_RPYS),
        )

        #### Initialize a circular trajectory ######################
        PERIOD = 12
        HOLIZON = 1    # 3 step of velocity controller

        self.NUM_WP_ALL = DEFAULT_CONTROL_FREQ_HZ*PERIOD
        TRAJECTORY_ALL =    np.zeros((self.NUM_WP_ALL,3))
        self.TRAJECTORY_ALL = TRAJECTORY_ALL
        for i in range(self.NUM_WP_ALL):
            '''
            TRAJECTORY_ALL[i,:] =   R*np.cos((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)  +0, \
                                    R*np.sin((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)-R+0, \
                                    0
            '''
            TRAJECTORY_ALL[i,:] =   R*np.cos((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)  +self.INIT_XYZS[0], \
                                    R*np.sin((i/self.NUM_WP_ALL)*(2*np.pi)+np.pi/2)-R+self.INIT_XYZS[1], \
                                    0
            ''''''

        self.step_counter = int((i_num*self.NUM_WP_ALL/6)%self.NUM_WP_ALL)
        print(self.step_counter)

        self.NUM_WP = int(DEFAULT_CONTROL_FREQ_HZ/DEFAULT_GUIDANCE_FREQ_HZ)*HOLIZON
        self.TARGET_POS = np.zeros((self.NUM_WP,3))
        self.TARGET_POS = TRAJECTORY_ALL[self.step_counter:(self.NUM_WP+self.step_counter),:]

    def publish_transform(self, position=None, quaternion=None):
        """Publish the simulator-owned ``world -> baselink`` transform."""

        if position is None:
            position = self.obs[0:3]
        if quaternion is None:
            quaternion = self.obs[3:7]
        self.baselink_tf.header.stamp = self.get_clock().now().to_msg()
        self.baselink_tf.transform.translation.x = float(position[0])
        self.baselink_tf.transform.translation.y = float(position[1])
        self.baselink_tf.transform.translation.z = float(position[2])
        self.baselink_tf.transform.rotation.x = float(quaternion[0])
        self.baselink_tf.transform.rotation.y = float(quaternion[1])
        self.baselink_tf.transform.rotation.z = float(quaternion[2])
        self.baselink_tf.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(self.baselink_tf)


    def _timer_callback(self):
        if not self.power_on:
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
        self.declare_parameter('gui', DEFAULT_GUI)
        self.declare_parameter('plot', DEFAULT_PLOT)
        
        timer_period = 1/DEFAULT_CONTROL_FREQ_HZ #[s]
        self.timer = self.create_timer(timer_period,self._timer_callback)

        # simulator
        self.env = None
        self.action = None
        self.ctrl = None
        self.step_counters = None
        self.logger = None
        self.plot_enabled = DEFAULT_PLOT
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
            gui=None,
            record_video=DEFAULT_RECORD_VISION,
            plot=None,
            user_debug_gui=DEFAULT_USER_DEBUG_GUI,
            obstacles=DEFAULT_OBSTACLES,
            simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
            control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
            duration_sec=DEFAULT_DURATION_SEC,
            output_folder=DEFAULT_OUTPUT_FOLDER,
            colab=DEFAULT_COLAB):

        #### Resolve launch/runtime parameters #####################
        if gui is None:
            gui = self.get_parameter('gui').get_parameter_value().bool_value
        if plot is None:
            plot = self.get_parameter('plot').get_parameter_value().bool_value
        self.plot_enabled = plot

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
        obs, *_ = self.env.step(self.action)
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
        #self.env.render()

        for j in range(self.num_drones):
            self.logger.log(drone=j,  
                        timestamp=self.time/self.env.CTRL_FREQ,
                        state=obs[j],
                        control=np.hstack([self.droneList[j].TARGET_POS[self.droneList[j].wp_counters, :2], self.INIT_XYZS[j, 2], self.INIT_RPYS[j, :], np.zeros(6)])
                        )
            self.droneList[j].publish_transform()

        #### Sync the simulation ###################################
        self.time += 1

    def plot(self):
        self.logger.plot()

    def close(self):
        if self.env is not None:
            self.env.close()


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    world_node = SimpleWorld()
    name_list = ['a','b','c','d']
    try:
        executor.add_node(world_node)

        for i in range(DEFAULT_NUM_DRONES):
            if i < len(name_list):
                _name = name_list[i]
            else:
                _name = 'drone_'+str(i)
            _d = SimpleDrone(_name)
            world_node.setDroneInWorld( _d )
            world_node.droneList[i].setEnv(i,.3,.1,.05)
            executor.add_node(world_node.droneList[i])
        world_node.setEnv()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may forward another SIGINT while cleanup is in progress.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        world_node.close()
        if world_node.logger is not None and world_node.plot_enabled:
            world_node.plot()
        for drone in world_node.droneList:
            drone.destroy_node()
        world_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
