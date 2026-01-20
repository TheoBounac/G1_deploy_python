from states.fsm import State
import numpy as np
import torch
from collections import deque
from pathlib import Path 
import numpy as np
import redis
import torch
from collections import deque
import json
from common.rot_utils import quatToEuler
from common.params import DEFAULT_MIMIC_OBS

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


class MocapState(State):
    def __init__(self, fsm):
        super().__init__(fsm, "mocap")

        ## Lien vers le controlleur des moteurs ##
        self.ctrl = self.fsm.controller          #
        self.control_dt = self.ctrl.control_dt_  #
        ##########################################

        ############################## Config ###################################
        self.num_actions = 23                                                   #
        self.sim_dt = 0.001                                                     #
        self.sim_decimation = 20                                                #
        xml_file = "assets/g1/g1_sim2sim_with_wrist_roll.xml"                   #
        policy_path = "policies/mimic_policies/twist_general_motion_tracker.pt" #
        device = 'cuda'                                                         #
        #########################################################################

        ####################### Connexion Redis ################################
        self.redis_client = None                                               #
        try:                                                                   #
            self.redis_client = redis.Redis(host='localhost',                  #
                                            port=6379, db=0)                   #
        except Exception as e:                                                 #
            print(f"Error connecting to Redis: {e}")                           #
        ########################################################################

        ############################ Load policy ###############################
        self.policy = torch.jit.load(policy_path, map_location='cuda')         #
        print(f"Policy loaded from {policy_path}")                             #
        ######################################################################## 

        ############################ Default Values #############################
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)         #
        self.default_dof_pos = np.array([                                       #
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # left leg (6)                 #
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # right leg (6)                #
                0.0, 0.0, 0.0, # torso (1)                                      #
                0.0, 0.4, 0.0, 1.2,                                             #
                0.0, -0.4, 0.0, 1.2,                                            #
            ])                                                                  #
        self.default_mimic_obs = DEFAULT_MIMIC_OBS["g1"]                        #
        self.mujoco_default_dof_pos = np.concatenate([                          #
            np.array([0, 0, 0.793]),                                            #
            np.array([0, 0, 0, 1]),                                             #
            np.array([-0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # left leg (6)           #
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # right leg (6)                #
                0.0, 0.0, 0.0, # torso (1)                                      #
                0.0, 0.2, 0.0, 1.2, 0.0, # left arm (4)                         #
                0.0, -0.2, 0.0, 1.2, 0.0, # right arm (4)                       #
                ])                                                              #
        ])                                                                      #
        self.stiffness = np.array([                                             #
        100, 100, 100, 150, 40, 40,                                             #
        100, 100, 100, 150, 40, 40,                                             #
        150, 150, 150,                                                          #
        40, 40, 40, 40, 20, 20, 20,                                             #
        40, 40, 40, 40, 20, 20, 20                                              #
            ])                                                                  #
        self.damping = np.array([                                               #
                2, 2, 2, 4, 2, 2,                                               #
                2, 2, 2, 4, 2, 2,                                               #
                4, 4, 4,                                                        #
                5, 5, 5, 5, 1,1,1,                                              #
                5, 5, 5, 5, 1,1,1                                               # 
            ])                                                                  #
        self.torque_limits = np.array([                                         #
                88, 139, 88, 139, 50, 50,                                       #
                88, 139, 88, 139, 50, 50,                                       #
                88, 50, 50,                                                     #
                25, 25, 25, 25, 25,                                             #
                25, 25, 25, 25, 25,                                             #
            ])                                                                  #
        self.action_scale = 0.5                                                 #
        self.ankle_idx = [4, 5, 10, 11]                                         #
        self.pd_target = []                                                     #
        #########################################################################


    def enter(self):
        print("[FSM] Enter MOCAP")
        self.STEPS = 0       

        ########################### For multi-step history #######################   
        self.n_mimic_obs = 31                                                    #
        self.n_proprio = self.n_mimic_obs + 3 + 2 + 3*self.num_actions           #
        self.proprio_history_buf = deque(maxlen=10)                              #
        for _ in range(10):                                                      #
            self.proprio_history_buf.append(np.zeros(self.n_proprio))            #
        ##########################################################################

    
    ########################### Mimic / Data extraction ##########################
    def extract_mimic_obs_to_body_and_wrist(self, mimic_obs):                    #
        total_degrees = 33                                                       #
        wrist_ids = [27, 32]                                                     #
        other_ids = [f for f in range(total_degrees) if f not in wrist_ids]      #
        policy_target = mimic_obs[other_ids]                                     #
        wrist_dof_pos = mimic_obs[wrist_ids]                                     #
                                                                                 #
        return policy_target, wrist_dof_pos                                      #
                                                                                 #
    def aggregate_wrist_dof_pos(self, body_dof_pos, wrist_dof_pos):              #
        total_degrees = 25                                                       #
        wrist_ids = [19, 24]                                                     #
        other_ids = [f for f in range(total_degrees) if f not in wrist_ids]      #
        whole_body_pd_target = np.zeros(total_degrees)                           #
        whole_body_pd_target[other_ids] = body_dof_pos                           #
        whole_body_pd_target[wrist_ids] = wrist_dof_pos                          #
                                                                                 #
        return whole_body_pd_target                                              #
                                                                                 #
    import numpy as np                                                           #
                                                                                 #
    def L_motor_mujoco_policy(self, L_motor: np.ndarray) -> np.ndarray:          #
        idx = np.array([                                                         #
            0, 1, 2, 3, 4, 5,                                                    #
            6, 7, 8, 9, 10, 11,                                                  #
            12, 13, 14,                                                          #
            15, 16, 17, 18, 19,                                                  #
            22, 23, 24, 25, 26                                                   #
        ])                                                                       #
        return L_motor[idx]                                                      #
                                                                                 #
    def L_motor_policy_mujoco(self,L_motor_policy: np.ndarray) -> np.ndarray:    #
        idx = np.array([                                                         #
            0, 1, 2, 3, 4, 5,                                                    #
            6, 7, 8, 9, 10, 11,                                                  #
            12, 13, 14,                                                          #
            15, 16, 17, 18, 19,                                                  #
            22, 23, 24, 25, 26                                                   #
        ])                                                                       #
                                                                                 #
        L_motor_mujoco = np.zeros(29, dtype=L_motor_policy.dtype)                #
        L_motor_mujoco[idx] = L_motor_policy                                     #
        return L_motor_mujoco                                                    #
                                                                                 #
    def extract_data(self):                                                      #
        body_ids = [0,1,2,3,4,5,                                                 #
                    6,7,8,9,10,11,                                               #
                    12,13,14,                                                    #
                    15,16,17,18,# 19                                             #
                    20,21,22,23, # 24                                            #
                    ]                                                            #
        wrist_ids = [19, 24]                                                     #
                                                                                 #
        whole_whole_body_dof = np.array([self.ctrl.low_state.motor_state[i].q for i in range(29)], dtype=np.float32)      # 29 #  Les données moteurs au format Mujoco g1_29_dof
        whole_whole_body_dof_vel = np.array([self.ctrl.low_state.motor_state[i].dq for i in range(29)], dtype=np.float32) # 29 #
                                                                                    
        whole_body_dof = self.L_motor_mujoco_policy(whole_whole_body_dof)        # 25 # Les données moteurs au format Mujoco g1_25_dof (sans les poignets  aux indices 20,21,27,28 du g1_29_dof)
        whole_body_dof_vel = self.L_motor_mujoco_policy(whole_whole_body_dof_vel)# 25 #
                                                                                 #
        body_dof_pos = whole_body_dof[[f for f in body_ids]]                     # 23 # Les données moteurs au format policy, comme g1_25_dof mais sans les poignets aux indices 19 et 24
        body_dof_vel = whole_body_dof_vel[[f for f in body_ids]]                 # 23 #
                                                                                 #
        wrist_dof_pos = whole_whole_body_dof[[f for f in wrist_ids]]             # 2 #
        wrist_dof_vel = whole_body_dof_vel[[f for f in wrist_ids]]               # 2 #
                                                                                 ###########################################
        quat = self.ctrl.low_state.imu_state.quaternion                                                               # 4  #
        ang_vel = np.array(self.ctrl.low_state.imu_state.gyroscope, dtype=np.float32)                                 # 3  #
                                                                                                                           #
        return whole_body_dof, whole_body_dof_vel, body_dof_pos, body_dof_vel, wrist_dof_pos, wrist_dof_vel, quat, ang_vel #
    ########################################################################################################################

    def send_start_pose(self):
        whole_body_dof, whole_body_dof_vel, body_dof_pos, body_dof_vel, wrist_dof_pos, wrist_dof_vel, quat, ang_vel = self.extract_data() 
        self.redis_client.set("pose_depart", json.dumps(whole_body_dof.tolist()))


    def step(self):

        ######################## send initial proprio to redis ###################
        proprio_json = json.dumps(self.proprio_history_buf[0].tolist())          #
        self.redis_client.set("state_body_g1", proprio_json)                     #
        self.redis_client.set("state_hand_g1", json.dumps(np.zeros(14).tolist()))#
        ##########################################################################

        whole_body_dof, whole_body_dof_vel, body_dof_pos, body_dof_vel, wrist_dof_pos, wrist_dof_vel, quat, ang_vel = self.extract_data()  # Ce que renvoit Mujoco

        if self.STEPS % self.sim_decimation == 0:

            ############ Build a "proprio" vector for your policy #########
            rpy = quatToEuler(quat)                                       #
            obs_body_dof_vel = body_dof_vel.copy()                        #
            obs_body_dof_vel[self.ankle_idx] = 0.                         #
            obs_proprio = np.concatenate([                                #
                ang_vel * 0.25,                                           #
                rpy[:2],                                                  #
                (body_dof_pos - self.default_dof_pos),                    #
                obs_body_dof_vel * 0.05,                                  #
                self.last_action                                          #
            ])                                                            #
            ###############################################################

            ########################### send proprio to redis ##########################
            self.redis_client.set("state_body_g1", json.dumps(obs_proprio.tolist()))   #
            self.redis_client.set("state_hand_g1", json.dumps(np.zeros(14).tolist()))  #
            ############################################################################

            ########## Try to get the latest mimic obs from Redis ########################################
            try:                                                                                         #
                action_mimic_json = self.redis_client.get("action_mimic_g1")                             #
                if action_mimic_json is not None:                                                        #
                    action_mimic_list = json.loads(action_mimic_json)                                    #
                    action_mimic = np.array(action_mimic_list, dtype=np.float32)                         #
                    action_mimic, wrist_dof_pos = self.extract_mimic_obs_to_body_and_wrist(action_mimic) #
                                                                                                         #
                else:                                                                                    #
                    print("cannot get action mimic from redis")                                          #
            except:                                                                                      #
                print("cannot get action mimic from redis")                                              #
            ##############################################################################################   

            ########################## Policy inference ###################################
            obs_full = np.concatenate([action_mimic, obs_proprio])                       ##
            obs_hist = np.array(self.proprio_history_buf).flatten()                      ##
            obs_buf = np.concatenate([obs_full, obs_hist])                               ##
            self.proprio_history_buf.append(obs_full)                                    ##
                                                                                         ##
            obs_tensor = torch.from_numpy(obs_buf).float().unsqueeze(0).to('cuda')       ##
            with torch.no_grad():                                                        ##
                raw_action = self.policy(obs_tensor).cpu().numpy().squeeze()             ##
                                                                                         ##
            self.last_action = raw_action                                                ##
            raw_action = np.clip(raw_action, -10., 10.)                                  ##
            scaled_actions = raw_action * self.action_scale                              ##
            self.pd_target = scaled_actions + self.default_dof_pos                       ##
            self.pd_target = self.aggregate_wrist_dof_pos(self.pd_target, wrist_dof_pos) ##
            ###############################################################################

        ##################### Controller Target ###############################
        self.ctrl.target_dof_pos = self.L_motor_policy_mujoco(self.pd_target) #
        self.ctrl.Kp = self.stiffness                                         #
        self.ctrl.Kd = self.damping                                           #
        #######################################################################

        self.STEPS += 1

    def exit(self):
        print("[FSM] Exit MOCAP")