from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from states.fsm import FSM
from controller.security import Security
from controller.buttons import Buttons
from common.remote_controller import RemoteController, KeyMap
from common.rotation_helper import get_gravity_orientation, transform_imu_data
import time
import torch
import numpy as np



class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked

    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

    JOINT_LIMITS = {
        0:  (-2.5307,       2.8798),       # left_hip_pitch_joint
        1:  (-0.5236,       2.9671),       # left_hip_roll_joint
        2:  (-2.7576,       2.7576),       # left_hip_yaw_joint
        3:  (-0.087267,     2.8798),       # left_knee_joint
        4:  (-0.87267,      0.5236),       # left_ankle_pitch_joint
        5:  (-0.2618,       0.2618),       # left_ankle_roll_joint

        6:  (-2.5307,       2.8798),       # right_hip_pitch_joint
        7:  (-2.9671,       0.5236),       # right_hip_roll_joint
        8:  (-2.7576,       2.7576),       # right_hip_yaw_joint
        9:  (-0.087267,     2.8798),       # right_knee_joint
        10: (-0.87267,      0.5236),       # right_ankle_pitch_joint
        11: (-0.2618,       0.2618),       # right_ankle_roll_joint

        12: (-2.618,        2.618),        # waist_yaw_joint
        13: (-0.52,         0.52),         # waist_roll_joint
        14: (-0.52,         0.52),         # waist_pitch_joint

        15: (-3.0892,       2.6704),       # left_shoulder_pitch_joint
        16: (-1.5882,       2.2515),       # left_shoulder_roll_joint
        17: (-2.618,        2.618),        # left_shoulder_yaw_joint
        18: (-1.0472,       2.0944),       # left_elbow_joint
        19: (-1.972222054,  1.972222054),  # left_wrist_roll_joint
        20: (-1.614429558,  1.614429558),  # left_wrist_pitch_joint
        21: (-1.614429558,  1.614429558),  # left_wrist_yaw_joint

        22: (-3.0892,       2.6704),       # right_shoulder_pitch_joint
        23: (-2.2515,       1.5882),       # right_shoulder_roll_joint
        24: (-2.618,        2.618),        # right_shoulder_yaw_joint
        25: (-1.0472,       2.0944),       # right_elbow_joint
        26: (-1.972222054,  1.972222054),  # right_wrist_roll_joint
        27: (-1.614429558,  1.614429558),  # right_wrist_pitch_joint
        28: (-1.614429558,  1.614429558),  # right_wrist_yaw_joint,
    }
class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Control:
    def __init__(self):
        self.joint_info = G1JointIndex()
        self.time = 0
        self.time_2 = 0
        self.fin = 0
        self.stop = 0
        self.control_dt_ = 0.003  # [1 ms]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.fin = 0
        self.mode_machine_ = 0
        self.update_mode_machine_ = False
        self.target_dof_pos = [0] * 29 # Commande de moteurs nulle au début

        self.G1_NUM_MOTOR = 29
        self.Kp = [0]*29
        self.Kd = [0]*29
        

    def Init(self):

        print("ATTENTION: Le robot va bouger, éloignez vous de lui")
        input("Appuyer sur Entrée pour continuer ...")

        # In SIM TO SIM MODE
        ChannelFactoryInitialize(0, "lo")

        # In SIM TO REAL MODE
        """ 
        ChannelFactoryInitialize(0, "enp0s31f6")

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        """ 

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        # Thread that runs the policy
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.Run, name="Run_control"
        )

        # Create remote
        self.remote_controller = RemoteController() 

        # Créer une fsm pour gérer les états et initialise l'état à passive
        self.fsm = FSM(self)          
        self.fsm.set_state("passive") 

        # Créer Security pour couper la simulation en cas de danger
        self.security = Security(self)

        # Créer Buttons pour gérer les buttons de la manette
        self.buttons = Buttons(self)

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
    
    def LowCmdWrite(self, motor_cmd_q, Kp, Kd) :
        for i in range(self.G1_NUM_MOTOR):
            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
            self.low_cmd.motor_cmd[i].tau = 0. 
            self.low_cmd.motor_cmd[i].q = motor_cmd_q[i]
            self.low_cmd.motor_cmd[i].dq = 0. 
            self.low_cmd.motor_cmd[i].kp = Kp[i] 
            self.low_cmd.motor_cmd[i].kd = Kd[i]
                
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

    def Start(self):
        print("-> En attente de connection avec le robot")
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            print("-> Robot G1 connecté")
            self.lowCmdWriteThreadPtr.Start()


    def Run(self):
        self.buttons.update_states_with_buttons()
        self.fsm.step()
        """
        if (self.security.check(self.target_dof_pos, )=="stop"):
            print("Arrete urgence")
            self.fsm.set_state("EMERGENCY STOP")
        """
        self.LowCmdWrite(self.target_dof_pos,self.Kp,self.Kd)
