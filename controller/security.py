import numpy as np
from common.rotation_helper import get_gravity_orientation

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
G1_NUM_MOTOR = 29

class Security:
    def __init__(self,controller):
        self.ctrl = controller


    def get_tilt_angle(self):
        # PROJECTED GRAVITY                                                                                   ##
        quat = self.ctrl.low_state.imu_state.quaternion  # (w, x, y, z)                                       ##
        gravity_orientation = get_gravity_orientation(quat).astype(np.float32)  # vector g base frame         ##

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

        return angle_deg
    

    def check(self, target_dof_pos):
        for i in range(G1_NUM_MOTOR):
            q_min, q_max = JOINT_LIMITS[i]
            if (target_dof_pos[i]<q_min*3 or target_dof_pos[i]>q_max*3):
                return "stop"
            
        if (self.get_tilt_angle()>50):
            return "stop"
        
        else :
            return "go"
        
            