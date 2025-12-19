import time
import torch
import torch.nn as nn
import sys
import numpy as np
import keyboard
from collections import deque
from pathlib import Path  
import matplotlib.pyplot as plt

# Get the current folder (the one where this file is located)
current_dir = Path(__file__).resolve()
project_root = current_dir.parents[1]  

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from common.remote_controller import RemoteController, KeyMap
from common.rotation_helper import get_gravity_orientation, transform_imu_data


G1_NUM_MOTOR = 29
Kp = [
    100.0,  # left_hip_pitch_joint
    100.0,  # left_hip_roll_joint
    100.0,  # left_hip_yaw_joint
    150.0,  # left_knee_joint
    40.0,   # left_ankle_pitch_joint
    40.0,   # left_ankle_roll_joint

    100.0,  # right_hip_pitch_joint
    100.0,  # right_hip_roll_joint
    100.0,  # right_hip_yaw_joint
    150.0,  # right_knee_joint
    40.0,   # right_ankle_pitch_joint
    40.0,   # right_ankle_roll_joint

    200.0,  # waist_yaw_joint
    40.0,   # waist_roll_joint
    40.0,   # waist_pitch_joint

    40.0,   # left_shoulder_pitch_joint
    40.0,   # left_shoulder_roll_joint
    40.0,   # left_shoulder_yaw_joint
    40.0,   # left_elbow_joint
    40.0,   # left_wrist_roll_joint
    40.0,   # left_wrist_pitch_joint
    40.0,   # left_wrist_yaw_joint

    40.0,   # right_shoulder_pitch_joint
    40.0,   # right_shoulder_roll_joint
    40.0,   # right_shoulder_yaw_joint
    40.0,   # right_elbow_joint
    40.0,   # right_wrist_roll_joint
    40.0,   # right_wrist_pitch_joint
    40.0,   # right_wrist_yaw_joint
]
Kd = [
    2.0,  # left_hip_pitch_joint
    2.0,  # left_hip_roll_joint
    2.0,  # left_hip_yaw_joint
    4.0,  # left_knee_joint
    2.0,  # left_ankle_pitch_joint
    2.0,  # left_ankle_roll_joint

    2.0,  # right_hip_pitch_joint
    2.0,  # right_hip_roll_joint
    2.0,  # right_hip_yaw_joint
    4.0,  # right_knee_joint
    2.0,  # right_ankle_pitch_joint
    2.0,  # right_ankle_roll_joint

    5.0,  # waist_yaw_joint
    5.0,  # waist_roll_joint
    5.0,  # waist_pitch_joint

    1.0,  # left_shoulder_pitch_joint
    1.0,  # left_shoulder_roll_joint
    1.0,  # left_shoulder_yaw_joint
    1.0,  # left_elbow_joint
    1.0,  # left_wrist_roll_joint
    1.0,  # left_wrist_pitch_joint
    1.0,  # left_wrist_yaw_joint

    1.0,  # right_shoulder_pitch_joint
    1.0,  # right_shoulder_roll_joint
    1.0,  # right_shoulder_yaw_joint
    1.0,  # right_elbow_joint
    1.0,  # right_wrist_roll_joint
    1.0,  # right_wrist_pitch_joint
    1.0,  # right_wrist_yaw_joint
]
default_motor_policy=[
    -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3,
  0.3, -0.2, -0.2, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97, 0.15, -0.15, 0.0,
  0.0, 0.0, 0.0]
default_motor_sdk = [
    -0.1,  # left_hip_pitch_joint       0
    0.0,   # left_hip_roll_joint        1
    0.0,   # left_hip_yaw_joint         2
    0.3,   # left_knee_joint            3
    -0.2,  # left_ankle_pitch_joint     4 
    0.0,   # left_ankle_roll_joint      5

    -0.1,  # right_hip_pitch_joint      6
    0.0,   # right_hip_roll_joint       7
    0.0,  # right_hip_yaw_joint         8
    0.3,   # right_knee_joint           9
    -0.2,  # right_ankle_pitch_joint    10
    0.0,   # right_ankle_roll_joint     11

    0.0,   # waist_yaw_joint            12
    0.0,   # waist_roll_joint           13
    0.0,   # waist_pitch_joint          14

    0.3,   # left_shoulder_pitch_joint  15
    0.25,  # left_shoulder_roll_joint   16
    0.0,   # left_shoulder_yaw_joint    17
    0.97,  # left_elbow_joint           18
    0.15,  # left_wrist_roll_joint      19
    0.0,   # left_wrist_pitch_joint     20
    0.0,   # left_wrist_yaw_joint       21

    0.3,   # right_shoulder_pitch_joint 22
    -0.25, # right_shoulder_roll_joint  23
    0.0,   # right_shoulder_yaw_joint   24
    0.97,  # right_elbow_joint          25
    -0.15, # right_wrist_roll_joint     26
    0.0,   # right_wrist_pitch_joint    27
    0.0,   # right_wrist_yaw_joint      28
]
sdk_to_policy = [
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11,
  17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
policy_to_sdk = [
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28
]
offset = [
    -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.3, -0.2,
      -0.2, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97, 0.15, -0.15, 0.0, 0.0, 0.0,
      0.0
      ]

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




class Custom:
    def __init__(self):
        self.joint_info = G1JointIndex()
        self.time = 0
        self.time_2 = 0
        self.fin=0
        self.stop = 0
        self.control_dt_ = 0.001  # [1 ms]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()

        self.obs_base_ang_vel = deque(maxlen=3*5)      # Chaque élément de observation pour 1 obs_frame (96,)
        self.obs_projected_gravity = deque(maxlen=3*5) #
        self.obs_velocity_commands = deque(maxlen=3*5) #
        self.obs_joint_pos_rel = deque(maxlen=29*5)    #
        self.obs_joint_vel_rel = deque(maxlen=29*5)    #
        self.obs_last_action = deque(maxlen=29*5)      #
        self.vx_filtered = 0 # Commandes de vitesse adoucies
        self.vy_filtered = 0 #
        self.wz_filtered = 0 #
        self.remote_controller = RemoteController() # Initialization of the controller
        for k in range(3*5):
            self.obs_base_ang_vel.append(0)
        for k in range(3*5):
            self.obs_projected_gravity.append(0)
        for k in range(3*5):
            self.obs_velocity_commands.append(0)
        for k in range(29*5):
            self.obs_joint_pos_rel.append(0)
        for k in range(29*5):
            self.obs_joint_vel_rel.append(0)
        for k in range(29*5):
            self.obs_last_action.append(0)
        self.action_rl = np.zeros(29, dtype=np.float32) #  dernière action rl 

        self.L_temps = []
        self.L_temps_2 = []

        self.L_target_pos_1 = []
        self.L_target_pos_2 = []
        self.L_target_pos_3 = []

        self.L_motor_state_1 = []
        self.L_motor_state_2 = []
        self.L_motor_state_3 = []




    def Init(self):
        """
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
	    """
        policy_path = project_root / "deploy_python" / "pre_train" / "policy.pt"
        self.policy = torch.jit.load(policy_path) 

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        # Thread that runs the policy
        self.timer = 0
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.Run, name="Run_control"
        )

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

    def LowCmdWrite(self,motor_cmd_q):
        for i in range(G1_NUM_MOTOR):
            q_min, q_max = self.joint_info.JOINT_LIMITS[i]
            if (motor_cmd_q[i]<q_min*3 or motor_cmd_q[i]>q_max*3):
                print("Arrete urgence")
                self.stop=1
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
            self.Position_depart() # Initial position
            self.lowCmdWriteThreadPtr.Start()

    def Position_depart(self): # Cette fonction déplace le robot vers sa position de départ et le maintient comme ca jusqu'à ce qu'on l'arrête
        self.init_motor_state = self.low_state.motor_state
        temps = 0
        duree_mouvement = 2
        print("-> Déplacement vers Position de Départ")
        while (temps < duree_mouvement):
            # Set robot to zero posture
            motor_cmd_q = []
            ratio = np.clip(temps / duree_mouvement, 0.0, 1.0)
            for i in range(G1_NUM_MOTOR):
                motor_cmd_q.append((1.0 - ratio) * self.init_motor_state[i].q + ratio * default_motor_sdk[i])

            self.LowCmdWrite(motor_cmd_q)
            temps += self.control_dt_    
            time.sleep(0.001)
        print("-> Le Robot se maintient en default pose")
        print("             ┌───────┐")
        print("APPUYER SUR  │ START │  POUR LANCER LE MODELE")
        print("             └───────┘")
        print(" ")
        while self.remote_controller.button[KeyMap.start] != 1:
            self.LowCmdWrite(default_motor_sdk)
            time.sleep(0.02)
        print("-> Le modèle tourne sur le robot")
        print("            ╔════════╗")
        print("APPUYER SUR ║ SELECT ║ POUR ARRETER LE MODELE")
        print("            ╚════════╝")
        print(" ")

    def Arret_soft(self): # Cette fonction déplace le robot vers sa position de départ et le maintient comme ca jusqu'à ce qu'on l'arrête
        self.init_motor_state = self.low_state.motor_state
        temps = 0
        duree_mouvement = 2
        print("-> Déplacement vers Position de Départ")
        while (temps < duree_mouvement):
            # Set robot to zero posture
            motor_cmd_q = []
            ratio = np.clip(temps / duree_mouvement, 0.0, 1.0)
            for i in range(G1_NUM_MOTOR):
                motor_cmd_q.append((1.0 - ratio) * self.init_motor_state[i].q + ratio * default_motor_sdk[i])

            self.LowCmdWrite(motor_cmd_q)
            temps += self.control_dt_    
            time.sleep(0.001)

        print("-> Le Robot se maintient en default pose.")
        print("            ╔════════╗")
        print("APPUYER SUR ║ SELECT ║ POUR RELACHER LE ROBOT")
        print("            ╚════════╝")
        print(" ")
        while self.remote_controller.button[KeyMap.select] != 1:
            self.LowCmdWrite(default_motor_sdk)
            time.sleep(0.02)
        self.fin=1
        print("-> Fin de l'expérience") 

    def get_tilt_angle(self,gravity_orientation):
        # gravity_orientation : np.array([gx, gy, gz])
        g = gravity_orientation.astype(np.float32)

        # Normalisation (par sécurité, au cas où le vecteur ne soit pas parfaitement normé)
        norm = np.linalg.norm(g) + 1e-8
        g /= norm

        gz = g[2]

        # cos(theta) = -gz  (comme dans bad_orientation côté C++)
        cos_theta = np.clip(-gz, -1.0, 1.0)
        angle_rad = np.arccos(cos_theta)
        angle_deg = np.degrees(angle_rad)

        return angle_rad, angle_deg

    def Run(self):

        if (self.remote_controller.button[KeyMap.select] == 1 and self.stop==0):
            self.stop = 1
            self.Arret_soft()
        
        if(self.timer==0 and self.stop!=1):
            ##################################### OBSERVATIONS ###################################################
            # BASE ANGLE VELOCITY
            base_ang_vel = np.array(self.low_state.imu_state.gyroscope, dtype=np.float32)  # [wx, wy, wz]
            base_ang_vel *= 0.2 # scale
            for k in range(len(base_ang_vel)):
                self.obs_base_ang_vel.append(base_ang_vel[k])

            # PROJECTED GRAVITY
            quat = self.low_state.imu_state.quaternion  # (w, x, y, z) ou autre ordre suivant SDK
            gravity_orientation = get_gravity_orientation(quat)  # vector g dans le repère base, dim=3
            projected_gravity = gravity_orientation.astype(np.float32)
            for k in range(len(projected_gravity)):
                self.obs_projected_gravity.append(projected_gravity[k])
            
            # VELOCITY COMMAND
            vx_cmd =  self.remote_controller.ly  
            vy_cmd = -self.remote_controller.lx 
            wz_cmd = -self.remote_controller.rx 

            self.vx_filtered = self.vx_filtered + 0.03 * (vx_cmd - self.vx_filtered)
            self.vy_filtered = self.vy_filtered + 0.03 * (vy_cmd - self.vy_filtered)
            self.wz_filtered = self.wz_filtered + 0.03 * (wz_cmd - self.wz_filtered)

            velocity_commands = np.array([self.vx_filtered, self.vy_filtered, self.wz_filtered], dtype=np.float32)
            for k in range(len(velocity_commands)):
                self.obs_velocity_commands.append(velocity_commands[k])

            # JOINT POS RELATIVE
            q_sdk = np.array([self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)], dtype=np.float32)
            q_rl = q_sdk[sdk_to_policy]  
            joint_pos_rel = q_rl - default_motor_policy
            for k in range(len(joint_pos_rel)):
                self.obs_joint_pos_rel.append(joint_pos_rel[k])

            # JOINT VEL RELATIVE
            dq_sdk = np.array([self.low_state.motor_state[i].dq for i in range(G1_NUM_MOTOR)], dtype=np.float32)
            dq_rl = dq_sdk[sdk_to_policy]
            joint_vel_rel = dq_rl * 0.05  # scale
            for k in range(len(joint_vel_rel)):
                self.obs_joint_vel_rel.append(joint_vel_rel[k])

            # LAST ACTION
            last_action = self.action_rl.copy().astype(np.float32)
            for k in range(len(last_action)):
                self.obs_last_action.append(last_action[k])

            # A chaque step, on creer observations qui prend dans l'ordre des observations les 5 dernieres:
            observations = []
            for k in range(len(self.obs_base_ang_vel)):
                observations.append(self.obs_base_ang_vel[k])

            for k in range(len(self.obs_projected_gravity)):
                observations.append(self.obs_projected_gravity[k])
            
            for k in range(len(self.obs_velocity_commands)):
                observations.append(self.obs_velocity_commands[k])
            
            for k in range(len(self.obs_joint_pos_rel)):
                observations.append(self.obs_joint_pos_rel[k])
            
            for k in range(len(self.obs_joint_vel_rel)):
                observations.append(self.obs_joint_vel_rel[k])
            
            for k in range(len(self.obs_last_action)):
                observations.append(self.obs_last_action[k])
            
            observations = np.asarray(observations, dtype=np.float32)
            obs_tensor = torch.from_numpy(observations).unsqueeze(0)  # (1, 480)
            self.action_rl = self.policy(obs_tensor).detach().numpy().squeeze()  # (29,)

            # Transform action to target_dof_pos                                   
            self.target_dof_pos = self.action_rl * 0.25 + offset

            self.L_temps_2.append(self.time_2)
            self.time_2 += 0.02
            self.L_target_pos_1.append(self.target_dof_pos[policy_to_sdk][0])
            self.L_target_pos_2.append(self.target_dof_pos[1])
            self.L_target_pos_3.append(self.target_dof_pos[2])

        if (self.stop!=1):
            self.LowCmdWrite(self.target_dof_pos[policy_to_sdk])


        quat = self.low_state.imu_state.quaternion
        gravity_orientation = get_gravity_orientation(quat)
        angle_rad, angle_deg = self.get_tilt_angle(gravity_orientation)
        print(angle_deg)

        
        self.L_temps.append(self.time)
        self.time += 0.001
        self.L_motor_state_1.append(self.low_state.motor_state[0].q)
        self.L_motor_state_2.append(self.low_state.motor_state[1].q)
        self.L_motor_state_3.append(self.low_state.motor_state[2].q)

        # On envoit les commandes moteurs toutes les 1 ms mais la maj des targets n'est que effectuée toutes les 20 ms
        self.timer+=0.001
        if (self.timer>=0.02):
            self.timer=0
        



    
if __name__ == '__main__':

    print("ATTENTION: Le robot va bouger, éloignez vous de lui")
    input("Appuyer sur Entrée pour continuer ...")
	
    #ChannelFactoryInitialize(0, "enp0s31f6")
    ChannelFactoryInitialize(0, "lo")

    print("\n")

    custom = Custom()
    custom.Init()
    custom.Start()

    while (custom.fin==0):        
            time.sleep(1)
    
    custom.lowCmdWriteThreadPtr._RecurrentThread__quit = True
    print("-> Courbes")
    plt.plot(custom.L_temps,custom.L_motor_state_1)
    plt.plot(custom.L_temps_2,custom.L_target_pos_1)
    plt.show()



