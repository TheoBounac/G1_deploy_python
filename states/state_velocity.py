from controller.fsm import State
import numpy as np
import torch
from collections import deque
from common.rotation_helper import get_gravity_orientation, transform_imu_data
from pathlib import Path 

# Get the current folder (the one where this file is located)
current_dir = Path(__file__).resolve()
project_root = current_dir.parents[2]  



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




class VelocityState(State):
    def __init__(self, fsm):
        super().__init__(fsm, "velocity")
        self.ctrl = self.fsm.controller # Lien vers le controlleur des moteurs

        self.obs_base_ang_vel = deque(maxlen=3*5)      # Chaque élément de observation pour 1 obs_frame (96*5 = 480 floats)
        self.obs_projected_gravity = deque(maxlen=3*5) #
        self.obs_velocity_commands = deque(maxlen=3*5) #
        self.obs_joint_pos_rel = deque(maxlen=29*5)    #
        self.obs_joint_vel_rel = deque(maxlen=29*5)    #
        self.obs_last_action = deque(maxlen=29*5)      #
        self.vx_filtered = 0 # Commandes de vitesse adoucies
        self.vy_filtered = 0 #
        self.wz_filtered = 0 #
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

        # Load policy
        policy_path = project_root / "deploy_python" / "pre_train" / "policy.pt"
        self.policy = torch.jit.load(policy_path) 


    def enter(self):
        print("[FSM] Enter VELOCITY")
        self.timer = 0


    def step(self):

        if self.timer == 0:
            ####################################### OBSERVATIONS ###################################################
            # BASE ANGLE VELOCITY                                                                                 ##
            base_ang_vel = np.array(self.ctrl.low_state.imu_state.gyroscope, dtype=np.float32)  # [wx, wy, wz]    ##
            base_ang_vel *= 0.2  # scale                                                                          ##
            for v in base_ang_vel:                                                                                ##
                self.obs_base_ang_vel.append(v)                                                                   ##
                                                                                                                  ##
            # PROJECTED GRAVITY                                                                                   ##
            quat = self.ctrl.low_state.imu_state.quaternion  # (w, x, y, z)                                       ##
            gravity_orientation = get_gravity_orientation(quat).astype(np.float32)  # vector g base frame         ##
            for v in gravity_orientation:                                                                         ##
                self.obs_projected_gravity.append(v)                                                              ##
                                                                                                                  ##
            # VELOCITY COMMAND (filtrée)                                                                          ##
            vx_cmd = self.ctrl.remote_controller.ly                                                               ##
            vy_cmd = -self.ctrl.remote_controller.lx                                                              ##
            wz_cmd = -self.ctrl.remote_controller.rx                                                              ##
                                                                                                                  ##
            self.vx_filtered += 0.03 * (vx_cmd - self.vx_filtered)                                                ##
            self.vy_filtered += 0.03 * (vy_cmd - self.vy_filtered)                                                ##
            self.wz_filtered += 0.03 * (wz_cmd - self.wz_filtered)                                                ##
                                                                                                                  ##
            velocity_commands = np.array([self.vx_filtered, self.vy_filtered, self.wz_filtered], dtype=np.float32)##
            for v in velocity_commands:                                                                           ##
                self.obs_velocity_commands.append(v)                                                              ##
                                                                                                                  ##
            # JOINT POS RELATIVE                                                                                  ##
            q_sdk = np.array([self.ctrl.low_state.motor_state[i].q                                                ##
                            for i in range(G1_NUM_MOTOR)], dtype=np.float32)                                      ##
            q_rl = q_sdk[sdk_to_policy]                                                                           ##
            joint_pos_rel = q_rl - default_motor_policy                                                           ##
            for v in joint_pos_rel:                                                                               ##
                self.obs_joint_pos_rel.append(v)                                                                  ##
                                                                                                                  ##
            # JOINT VEL RELATIVE                                                                                  ##
            dq_sdk = np.array([self.ctrl.low_state.motor_state[i].dq                                              ##
                            for i in range(G1_NUM_MOTOR)], dtype=np.float32)                                      ##
            dq_rl = dq_sdk[sdk_to_policy]                                                                         ##
            joint_vel_rel = dq_rl * 0.05                                                                          ##
            for v in joint_vel_rel:                                                                               ##
                self.obs_joint_vel_rel.append(v)                                                                  ##
                                                                                                                  ##
            # LAST ACTION                                                                                         ##
            last_action = self.action_rl.copy().astype(np.float32)                                                ##
            for v in last_action:                                                                                 ##
                self.obs_last_action.append(v)                                                                    ##
            ########################################################################################################


            ############################ BUILD OBS VECTOR WITH HISTORY LENGTH = 5 #####################################
            # A chaque step, on creer observations qui prend dans l'ordre les 5 dernieres observations               ##
            # rangés par groupe d'observations (base_ang_vel, gravity, commands, joints pos, joints vel, last_action)##
            observations = []                                                                                        ##
            for elem in self.obs_base_ang_vel:                                                                       ##
                observations.append(elem)                                                                            ##
                                                                                                                     ##
            for elem in self.obs_projected_gravity:                                                                  ##
                observations.append(elem)                                                                            ##
                                                                                                                     ##
            for elem in self.obs_velocity_commands:                                                                  ##
                observations.append(elem)                                                                            ##
                                                                                                                     ##
            for elem in self.obs_joint_pos_rel:                                                                      ##
                observations.append(elem)                                                                            ##
                                                                                                                     ##
            for elem in self.obs_joint_vel_rel:                                                                      ##
                observations.append(elem)                                                                            ##
                                                                                                                     ##
            for elem in self.obs_last_action:                                                                        ##
                observations.append(elem)                                                                            ##
                                                                                                                     ##
            observations = np.asarray(observations, dtype=np.float32)                                                ##
            ###########################################################################################################


            obs_tensor = torch.from_numpy(observations).unsqueeze(0)  # (1, 480)
            self.action_rl = self.policy(obs_tensor).detach().numpy().squeeze()  # (29,)
            target_dof_pos = self.action_rl * 0.25 + offset
            
            ########### Consigne pour le controller ##################
            self.ctrl.target_dof_pos = target_dof_pos[policy_to_sdk] #
            self.ctrl.Kp = Kp                                        #
            self.ctrl.Kd = Kd                                        #
            ##########################################################

        """||step() est appelé toute les 1ms || Mais la prédiction du réseau n'est faîte que toute les 20ms ||"""
        self.timer += 0.001
        if self.timer >= 0.02:
            self.timer = 0.0

    def exit(self):
        print("[FSM] Exit VELOCITY")