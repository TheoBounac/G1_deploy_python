from states.fsm import State
import numpy as np

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

""" Déplace le robot vers sa position de départ et le maintient comme ca """
class DefaultStaticState(State): 
    def __init__(self, fsm):
        super().__init__(fsm, "default_static")
        self.ctrl = self.fsm.controller # Lien vers le controlleur des moteurs
        self.control_dt = self.ctrl.control_dt_
        self.G1_NUM_MOTOR = self.ctrl.G1_NUM_MOTOR

    def enter(self):
        print("[FSM] Enter DEFAULT_STATIC")
        self.init_motor_state = self.ctrl.low_state.motor_state
        self.temps = 0

    def step(self):
        duree_mouvement = 3
        motor_cmd_q = []

        if (self.temps < duree_mouvement):
            """Déplacement linéaire du robot vers sa default pose"""
            ratio = np.clip(self.temps / duree_mouvement, 0.0, 1.0)
            for i in range(self.G1_NUM_MOTOR):
                motor_cmd_q.append((1.0 - ratio) * self.init_motor_state[i].q + ratio * default_motor_sdk[i])

            ########### Consigne pour le controller ##################
            self.ctrl.target_dof_pos = motor_cmd_q                   #
            self.ctrl.Kp = Kp                                        #
            self.ctrl.Kd = Kd                                        #
            ##########################################################
            self.temps += self.control_dt
            
        
        else :
            """Maintient du robot dans sa default pose"""
            ########### Consigne pour le controller ##################
            self.ctrl.target_dof_pos = default_motor_sdk             #
            self.ctrl.Kp = Kp                                        #
            self.ctrl.Kd =Kd                                         #
            ##########################################################
            

    def exit(self):
        print("[FSM] Exit DEFAULT_STATIC")


