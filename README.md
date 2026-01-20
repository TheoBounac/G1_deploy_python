 <p align="center">
  <img src="doc/im2.png" width="1000">
  <br>
 </p>
 
# <h2 align="center">G1 RL Deploy Python</h2>

**Python deploy code for Unitree G1 based on a Finite State Machine (FSM) architecture, only using Python. It allows SIM-to-SIM (Mujoco) and SIM-to-REAL (Unitree G1 robot) reinforcement learning models deployment.**

**This aims at deploying Reinforcement Learning models trained with : [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) for Locomotion, and [TWIST](https://github.com/YanjieZe/TWIST) for Teleoperated Whole-Body Control.**


<table align="center" style="border-collapse:collapse;">
<th style="width:50%; text-align:center;">
  <div style="display:inline-block; width:200px;">Deploy on Mujoco</div>
</th>

  <tr>
    <td style="width:50%; text-align:center;">
      <img src="doc/gif3.gif" style="width:100%; display:block; margin:auto;">
    </td>

  </tr>
</table>

---
## Project overview

This project implements a complete **Sim-to-Real** pipeline:

 - 🎮 How to train Reinforcement Learning (RL) policies in **IsaacLab Simulation**

 - 🤖 How to **deploy models on the real Go2 robot** via the Unitree SDK

 - 🔄 How to use **ROS 2 Communication** and **Kalman filter (Inria Paris)** for real-time control and sensor/command integration

The project combines **Python + ROS 2 + IsaacLab + Kalman filter (Inria Paris)**, enabling training, testing, and transferring an RL policy to the real robot.


---
## 📁 Architecture

```
G1_deploy_python/
├── main.py
├── controller/
│   ├── buttons.py
│   ├── controller.py
│   ├── security.py
│   ├── server_high_level_motion_lib.py
│   └── UI.py
├── states/
│   ├── fsm.py
│   ├── base_state.py
│   ├── state_passive.py
│   ├── state_default_static.py
│   ├── state_velocity.py
│   ├── state_mocap.py
│   └── emergency.py
├── policies/
│   └── locomotion_policies/
│   └── mimici_policies/
├── doc/
├── common/
└── assets/
```

---
## ⚙️ System Requirements

- Python >= 3.8 and env from  [Isaaclab](https://github.com/isaac-sim/IsaacLab)
  
---

Run

From the repository root:
```bash
python3 -m controller.main
```

or:

```bash
python3 main.py
```

---

Notes

- DDS must be initialized before any RPC client (MotionSwitcher), if SIMTOREAL -> uncomment __Init__() section in controller.py
- The FSM manages transitions between passive, default static and velocity control states.
- An emergency stop state is implemented and must be used with caution.

---

Author

Theo Bounaceur (CNRS-LORIA)
