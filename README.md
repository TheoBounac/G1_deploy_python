# G1_deploy_python

Python deploy code for Unitree G1 based on a Finite State Machine (FSM) architecture.

WARNING
This code sends commands to a real robot.
Make sure the robot is secured before running.

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

Requirements

- Python >= 3.8
- unitree_sdk2_python available in the environment
- Network interface connected to the robot (Ethernet or WiFi)

Recommended installation:
pip install -e unitree_sdk2_python

--------------------------------------------------

Run

From the repository root:
python3 -m controller.main

or:
python3 main.py

--------------------------------------------------

Notes

- DDS must be initialized before any RPC client (MotionSwitcher).
- The FSM manages transitions between passive, default static and velocity control states.
- An emergency stop state is implemented and must be used with caution.

--------------------------------------------------

Author

Theo Bounac
