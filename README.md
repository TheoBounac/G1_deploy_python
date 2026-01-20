 <p align="center">
  <img src="doc/im2.png" width="1000">
  <br>
 </p>
 
# <h2 align="center">G1 RL Deploy Python</h2>

**This repository provides a Python deployment framework for the Unitree G1 humanoid robot, designed to run reinforcement-learning policies both in simulation and on real hardware.**
**It supports SIM-to-SIM deployment in MuJoCo as well as SIM-to-REAL execution on the physical G1 robot, with a focus on UI control and safety during deployment.**

**It deploys RL policies trained for locomotion and teleoperated whole-body control.**


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

This project implements a complete deployment guide:

 - 🎮 How to deploy Reinforcement Learning (RL) policies in **Mujoco**

 - 🤖 How to deploy models on the **real G1 robot** via  Unitree SDK

Reinforcement Learning models are trained with : [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) for Locomotion, and [TWIST](https://github.com/YanjieZe/TWIST) for Teleoperated Whole-Body Control. 

The project combines **Python + Unitree SDK + Mujoco + IsaacLab + Unitree_rl_lab + Twist**, enabling the deployment of RL policies to the real robot with an Interactive **UI** and **Safety measures**.

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

|  Component |  Recommended Version |
|--------------|------------------------|
| Go2 robot (Unitree) | Edu version with feet sensors |
| **Ubuntu** | 22.04 LTS |
| **Python** | 3.10+ |
| **ROS 2** | Humble |
| **Isaac Sim / Isaac Lab** | 4.0.0+ |
| **CUDA** | 11.8+ |
---

---
<h2 align="center">🔧 Installation Guides🔧</h2> 
As mentioned earlier, this project is divided into two main parts :

  1. Training a reinforcement learning (RL) model in IsaacLab
  3. Deploying the trained model on the real Unitree Go2 robot
     
 I strongly suggest you to follow **Part 1** then **Part 2**. If you only want to deploy an RL model, you can skip the IsaacLab training tutorial. However, make sure to use the provided model, as it follows a specific observation structure required for deployment.

**Part 1** : [📘 How to train Reinforcement Learning (RL) policies on **IsaacLab Simulation**](doc/Isaaclab.md)

**Part 2** : [📘 How to **deploy models on the real Go2 robot** via the Unitree SDK](doc/Deploy.md)

Full tutorial for kalman filter from Inria Paris :

[📘 How to use  **Kalman filter (Inria Paris)** for real-time control and sensor/command integration](doc/Deploy_with_Kalman_filter.md)


---

##  Links

These are the repositories I used for my project :

| 🔗 Resources | 📍 Link |
|--------------|---------|
|  **IsaacLab (NVIDIA)** | [https://github.com/isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) |
|  **Unitree SDK2 Python** | [https://github.com/unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
|  **unitree_rl_lab** | [https://github.com/unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) |
|  **TWIST** | [https://github.com/YanjieZe/TWIST](https://github.com/YanjieZe/TWIST) |




---

## 👥 Author & Contributors

**Author:**  
Théo Bounaceur  
Laboratory **LORIA** (CNRS / University of Lorraine), Nancy, France  
🧬 Field: Reinforcement Learning · Unitree robots · IsaacLab · IsaacGym · ROS 2 · Unitree SDK2  
📫 Contact: theo.bounaceur@loria.fr  

**Supervisors / Advisors:**  
- Adrien Guenard  
- Cyril Regan  
