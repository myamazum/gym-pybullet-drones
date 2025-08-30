import math
import numpy as np
import copy
from typing     import List
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg  import Pose, PoseArray
from swarm_msgs.msg     import SwarmSensing

DEFAULT_DIM_STATE   = 7
DEFAULT_LOG_SELF    = 10

DEFAULT_NUM_DRONES  = 8

DEFAULT_SWARM_CONTROL_FREQ_HZ   = 10
DEFAULT_WP_STEP     = 5

### subscribe 
class Sense():
    """
    @ brief 群の計測を表すクラス

    Attributes:
        __id (str)              :   個体の識別子
        n (set)                 :   群れの識別子の集合
        ni (list)               :   隣接の識別子の集合
        state_i     (Pose[])    :   個体の状態量
        states_j    (dict)      :   隣接の状態量 ( 識別子:Pose )
    """

    def __init__(self,_id :str,_group:set = None):
        """
        @ brief コンストラクタ

        @param  _id     個体の識別子
        @param  _group  群れの識別子の集合
        """
        self.__id = _id
        self.n = _group if _group is not None else set()
        self.ni = []
        self.state_i = List[Pose]
        self.states_j= dict()
        self.states_time= dict()
   
    def setSensingSet(self,_group:list = None):
        """
        @ brief 観測集合の更新
        """
        self.ni = _group if _group is not None else []

    def resetSensing(self):
        self.state_i = List[Pose]
        self.states_j= dict()
        self.states_time= dict()

    ### 最新の情報しか保持しない
    def sense_self_callback(self,_msg : PoseArray):
        self.states_time[self.__id]  = _msg.header.stamp
        self.state_i = _msg.poses
    
    ### 最新の情報しか保持しない
    def sense_adjacents_callback(self,_msg : SwarmSensing):
        """
        @brief SwarmSensing.msg から 状態量を取得
        """
        for _j in range(len(_msg.neighbors)):
            self.states_time[_msg.neighbors[_j]]    = _msg.header.stamp
            self.states_j[_msg.neighbors[_j]]       = _msg.poses[_j]
        self.ni = _msg.neighbors

### 
class Control():
    def __init__(self):
        self.ctrl_msg = PoseArray()

    def swarm_ctrl(self,_xii,xNi):
        _msg = Pose()
        self.ctrl_msg = PoseArray()
        self.ctrl_msg.header
        self.ctrl_msg.poses.append(_msg)
        for _t in range(DEFAULT_WP_STEP):
            self.ctrl_msg.poses.append(_msg)

class SwarmDrone(Node):
    def __init__(self,_node :str ='drone',_id : str='0'):
        self.__name = _node + _id
        super().__init__(self.__name)
        self.sw_ctrl    = Control(self.__name)
        self.sw_sense   = Sense(self.__name)
        self.__neighbors = set()

        # 状態量を numpy の変数で取得
        self.xii = np.empty((DEFAULT_DIM_STATE,DEFAULT_LOG_SELF),dtype=np.float64)
        self.xNi = dict()
        self.sns_time = dict()

        self.ctrl_swarm_pub     =   self.create_publisher(
            PoseArray,
            'swarm_control_topic',
            3
        )

        self.sense_self_sub     =   self.create_subscription(
            PoseArray,
            'self_sensing_topic',
            self.sw_sense.sense_self_callback,
            1
            )
        self.sense_adjacents_sub=   self.create_subscription(
            SwarmSensing,
            'swarm_sensing_topic',
            self.sense_adjacents_callback,
            1
            )

        ### timer
        timer_period = 1/DEFAULT_SWARM_CONTROL_FREQ_HZ #[s]
        self.timer = self.create_timer(timer_period,self._timer_callback)

    def _timer_callback(self):
        ### 状態変数を取得する
        self.__neighbors = copy.deepcopy(self.sw_sense.ni)
        xii = np.empty((DEFAULT_LOG_SELF,DEFAULT_DIM_STATE),dtype=np.float64)
        xNi = np.empty((len(self.__neighbors),DEFAULT_DIM_STATE),dtype=np.float64)
        sns_time = np.empty(1+len(self.__neighbors),dtype=np.float64)

        ### wait Sensing (データが空の場合は sleepなどで確保を待つようにする)
        ### timer-loopにいるとき、subscribe の結果って反映されるか？未確認

        ### xii は最新の順番から降順で状態量を保存
        xii = np.array([
            [pose.position.x,pose.position.y,pose.position.z,
             pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w] 
             for pose in reversed(self.sw_sense.state_i[:DEFAULT_LOG_SELF])
             ])
        sns_time[-1] = self.sw_sense.states_time[ self.__name ].sec + self.sw_sense.states_time[ self.__name ].nanosec/10e9
        
        for _j in range(len(self.__neighbors)):
            pose = self.sw_sense.states_j[ self.__neighbors[_j] ]
            xNi[_j,:] = np.array(
                [pose.position.x,pose.position.y,pose.position.z,
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
            sns_time[_j] = self.sw_sense.states_time[ self.__neighbors[_j] ].sec + self.sw_sense.states_time[ self.__neighbors[_j] ].nanosec/10e9
        self.sw_sense.resetSensing()
        self.setCurrentState()

        ### 制御の演算を行う
        self.sw_ctrl.swarm_ctrl(xii[0,:],xNi)
        self.ctrl_swarm_pub(self.sw_ctrl.ctrl_msg)
    
    def setCurrentState(self,_xii,_xNi,_sns_time):

        _scale_time = _sns_time[-1] - range(len(_xii))/DEFAULT_SWARM_CONTROL_FREQ_HZ
        ## 各 xNi[_j,:] の取得時刻から 最も近い xii の時刻を抽出して indices として記録する
        _indices = np.searchsorted(reversed(_scale_time),_sns_time)
        for _j in range(len(self.__neighbors)):
            _xNi[_j] = self.estimate_rotation(_xii[_indices[_j]],_xii[0],_xNi[_j])
            _sns_time[_j]=_sns_time[-1]

    ### 実装、未検証
    def estimate_rotation(self,_xii_j,_xii_latest,_xNi_past):
        """
        @ brief 相対観測した対象について移動体の回転による影響を補正

        Args:
            _xii_past (np.ndarray): 過去の移動体の状態 [位置、クォータニオン]。
            _xii_latest (np.ndarray): 最新の移動体の状態 [位置、クォータニオン]。
            _xNi_past (np.ndarray): 過去の観測対象の状態 [位置、クォータニオン]。
        Returns:
            np.ndarray: 最新の観測対象の状態 [位置、クォータニオン]。
        """

        # クォータニオンの差（回転の差）を計算
        q_j = Rotation.from_quat(_xii_j[3:7])  # jステップ目のクォータニオン
        q_latest = Rotation.from_quat(_xii_latest[3:7])  # 最新のクォータニオン
        q_relative = q_latest.inv() * q_j  # 相対的な回転を求める

        # 最新の xNi[-1] に相対姿勢を適用
        q_relative_latest = Rotation.from_quat(_xNi_past[3:7])
        estimated_rotation = q_relative * q_relative_latest  # 回転の適用

        return np.concatenate([_xNi_past[0:3], estimated_rotation.as_quat()])

def main():
    try:
        rclpy.init()
        exec = SingleThreadedExecutor()
        node = SwarmDrone(Node)
        exec.add_node(node)
        exec.spin()
    except(KeyboardInterrupt,ExternalShutdownException):
        pass
    exec.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


