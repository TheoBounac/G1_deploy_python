from states.fsm import State
import numpy as np
from common.rotation_helper import get_gravity_orientation
import onnx
import onnxruntime
import torch

onnx_path = "dance_0605.onnx"



kp = [100, 100, 100, 200, 20, 20,
            100, 100, 100, 200, 20, 20,
            400, 400, 400,
            90,   60,  20, 60, 20, 20, 20,
            90,   60,  20, 60, 20, 20, 20
            ]
kd = [2.5, 2.5, 2.5, 5.0, 0.2, 0.1,
            2.5, 2.5, 2.5, 5.0, 0.2, 0.1,
            5.0, 5.0, 5.0,
            2.0, 1.0, 0.4, 1.0, 0.4, 0.4, 0.4,
            2.0, 1.0, 0.4, 1.0, 0.4, 0.4, 0.4
            ]
tau_limit = [88, 88, 88, 139, 50, 50,
            88, 88, 88, 139, 50, 50,
            88, 50, 50,
            25, 25, 25, 25, 5, 5, 5,
            25, 25, 25, 25, 5, 5, 5
           ]
default_angles = [-0.1,  0.0,  0.0,  0.3, -0.2, 0.0, 
                 -0.1,  0.0,  0.0,  0.3, -0.2, 0.0,
                  0.0, 0.0, 0.0, 
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                  ]
dof23_index = [ 0, 1, 2, 3, 4, 5,
               6, 7, 8 ,9 ,10, 11,
               12, 13, 14,
               15, 16, 17, 18,
               22, 23, 24, 25
             ]

ang_vel_scale = 0.25
dof_pos_scale = 1.0
dof_vel_scale = 0.05
action_scale = 0.25
history_length = 4
num_actions = 23
num_obs = 380
motion_length = 18.0



class Dance(State):
    """ONNX skill: Dance (23 actions -> 29 dof targets)"""

    def __init__(self, fsm):
        super().__init__(fsm, "dance")
        self.ctrl = self.fsm.controller

        # -------------------- Datas -----------------------------
        self.motion_phase = 0
        self.counter_step = 0
        self.ref_motion_phase = 0
        self.kps = np.array(kp, dtype=np.float32)
        self.kds = np.array(kd, dtype=np.float32)
        self.default_angles =  np.array(default_angles, dtype=np.float32)
        self.dof23_index =  np.array(dof23_index, dtype=np.int32)
        self.tau_limit =  np.array(tau_limit, dtype=np.float32)
        self.num_actions = num_actions
        self.num_obs = num_obs
        self.ang_vel_scale = ang_vel_scale
        self.dof_pos_scale = dof_pos_scale
        self.dof_vel_scale = dof_vel_scale
        self.action_scale = action_scale
        self.history_length = history_length
        self.motion_length = motion_length
        self.timer = 0

        # -------------------- Buffers -----------------------------
        self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
        self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
        self.obs = np.zeros(self.num_obs)
        self.action = np.zeros(self.num_actions)
        self.obs_history = np.zeros((self.history_length, self.num_obs), dtype=np.float32)
        
        self.ang_vel_buf = np.zeros(3 * self.history_length, dtype=np.float32)
        self.proj_g_buf = np.zeros(3 * self.history_length, dtype=np.float32)
        self.dof_pos_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.dof_vel_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.action_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.ref_motion_phase_buf = np.zeros(1 * self.history_length, dtype=np.float32)

        # -------------------- Policy ----------------
        self.onnx_model = onnx.load(onnx_path)
        self.ort_session = onnxruntime.InferenceSession(onnx_path)
        self.input_name = self.ort_session.get_inputs()[0].name
        for _ in range(50):
                obs_tensor = torch.from_numpy(self.obs).unsqueeze(0).cpu().numpy()
                obs_tensor = obs_tensor.astype(np.float32)
                self.ort_session.run(None, {self.input_name: obs_tensor})[0]
    
    def progress_bar(self, current, total, length=50):
        percent = current / total
        filled = int(length * percent)
        bar = "█" * filled + "-" * (length - filled)
        return f"\r|{bar}| {percent:.1%} [{current:.3f}s/{total:.3f}s]"



    def enter(self):
        self.action = np.zeros(23, dtype=np.float32)
        self.action_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.ref_motion_phase = 0.
        self.ref_motion_phase_buf = np.zeros(1 * self.history_length, dtype=np.float32)
        self.motion_time = 0
        self.counter_step = 0
        
        self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
        self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
        self.obs = np.zeros(self.num_obs)
        self.action = np.zeros(self.num_actions)
        self.obs_history = np.zeros((self.history_length, self.num_obs), dtype=np.float32)
        
        self.ang_vel_buf = np.zeros(3 * self.history_length, dtype=np.float32)
        self.proj_g_buf = np.zeros(3 * self.history_length, dtype=np.float32)
        self.dof_pos_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.dof_vel_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.action_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.ref_motion_phase_buf = np.zeros(1 * self.history_length, dtype=np.float32)


    def step(self):
        if self.counter_step == 0:
            quat = self.ctrl.low_state.imu_state.quaternion  
            gravity_orientation = get_gravity_orientation(quat).astype(np.float32)
            gravity_orientation = np.asarray(gravity_orientation, dtype=np.float32).reshape(-1)

            qj = []
            dqj =[]
            for i in range(29):
                qj.append(self.ctrl.low_state.motor_state[i].q)
                dqj.append(self.ctrl.low_state.motor_state[i].dq)
            qj = np.asarray(qj, dtype=np.float32).reshape(-1)
            dqj = np.asarray(dqj, dtype=np.float32).reshape(-1)

            ang_vel = self.ctrl.low_state.imu_state.gyroscope
            ang_vel = np.asarray(ang_vel, dtype=np.float32).reshape(-1)


            qj_23dof = qj[self.dof23_index].copy()
            dqj_23dof = dqj[self.dof23_index].copy()
            default_angles_23dof = self.default_angles[self.dof23_index].copy()
            qj_23dof = (qj_23dof - default_angles_23dof) * self.dof_pos_scale
            dqj_23dof = dqj_23dof * self.dof_vel_scale
            ang_vel = ang_vel * self.ang_vel_scale

            self.ang_vel_buf = np.concatenate((ang_vel, self.ang_vel_buf[:-3]), axis=-1, dtype=np.float32)
            self.proj_g_buf = np.concatenate((gravity_orientation, self.proj_g_buf[:-3] ), axis=-1, dtype=np.float32)
            self.dof_pos_buf = np.concatenate((qj_23dof, self.dof_pos_buf[:-23] ), axis=-1, dtype=np.float32)
            self.dof_vel_buf = np.concatenate((dqj_23dof, self.dof_vel_buf[:-23] ), axis=-1, dtype=np.float32)
            self.action_buf = np.concatenate((self.action, self.action_buf[:-23] ), axis=-1, dtype=np.float32)
            self.ref_motion_phase_buf = np.concatenate((np.array([min(self.ref_motion_phase,1.0)]), self.ref_motion_phase_buf[:-1] ), axis=-1, dtype=np.float32)
            

            mimic_history_obs_buf = np.concatenate((self.action_buf, 
                                                    self.ang_vel_buf, 
                                                    self.dof_pos_buf, 
                                                    self.dof_vel_buf, 
                                                    self.proj_g_buf, 
                                                    self.ref_motion_phase_buf
                                                    ), 
                                                    axis=-1, dtype=np.float32)
            
            mimic_obs_buf = np.concatenate((self.action,
                                            ang_vel,
                                            qj_23dof,
                                            dqj_23dof,
                                            mimic_history_obs_buf,
                                            gravity_orientation,
                                            np.array([min(self.ref_motion_phase,1.0)])
                                            ),
                                            axis=-1, dtype=np.float32)
            
            mimic_obs_tensor = torch.from_numpy(mimic_obs_buf).unsqueeze(0).cpu().numpy()
            self.action = np.squeeze(self.ort_session.run(None, {self.input_name: mimic_obs_tensor})[0])
            target_dof_pos = np.zeros(29)
            target_dof_pos[:15] = self.action[:15] * self.action_scale + self.default_angles[:15]
            target_dof_pos[15:19] = self.action[15:19] * self.action_scale + self.default_angles[15:19]
            target_dof_pos[22:26] = self.action[19:] * self.action_scale + self.default_angles[22:26]
            
            target_dof_pos[19:22] = self.default_angles[19:22]
            target_dof_pos[26:29] = self.default_angles[26:29]
            
            self.ctrl.target_dof_pos = target_dof_pos
            self.Kp = self.kps
            self.Kd = self.kds


            # update motion phase
            """self.counter_step += 1
            motion_time = self.counter_step * 0.02
            self.ref_motion_phase = motion_time / self.motion_length
            motion_time = min(motion_time, self.motion_length)
            print(self.progress_bar(motion_time, self.motion_length), end="", flush=True)"""

        """||step() est appelé toute les 1ms || Mais la prédiction du réseau n'est faîte que toute les 20ms ||"""
        self.timer += 0.003
        if self.timer >= 0.02:
            self.timer = 0.0

        self.counter_step += 1
        if self.counter_step == 6:
            self.counter_step = 0


    def exit(self):
        # Optional: return to default pose, or just keep last command
        pass
