 <p align="center">
  <img src="doc/im2.png" width="1000">
  <br>
 </p>
 
# <h2 align="center">G1 RL Deploy Python</h2>
# G1_deploy_python

**Python deploy code for Unitree G1 based on a Finite State Machine (FSM) architecture, only using Python.

This aims at deploying Reinforcement Learning models trained with : [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) for Locomotion, and [TWIST](https://github.com/YanjieZe/TWIST) for Teleoperated Whole-Body Control.

It allows SIM-to-SIM (Mujoco) and SIM-to-REAL (Unitree G1 robot) reinforcement learning models deployment.**


---
## 📁 Architecture

```

G1_deploy_python/
├── main.py
├── controller/
│   ├── controller.py
│   ├── fsm.py
│   ├── deploy_fsm.py
│   └── security.py
├── states/
│   ├── base_state.py
│   ├── state_passive.py
│   ├── state_default_static.py
│   ├── state_velocity.py
│   └── emergency.py
└── common/
    ├── command_helper.py
    ├── remote_controller.py
    └── rotation_helper.py
```

---
## ⚙️ System Requirements

- Python >= 3.8 and env from  [Isaaclab](https://github.com/isaac-sim/IsaacLab)
- unitree_sdk2_python available in the environment

Recommended installation:
pip install -e unitree_sdk2_python

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
