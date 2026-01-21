 <p align="center">
  <img src="doc/im3.png" width="1000">
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

The project combines **Python + Unitree SDK + Mujoco + IsaacLab + Unitree_rl_lab + Twist**, enabling the deployment of RL policies on the real robot with an Interactive **UI** and **Safety measures**.

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
├── mocap_files/
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

## 1️⃣ 🐍 Create & prepare the Conda environment

Create your workspace :
```bash
mkdir -p ~/G1
```

Create the env conda :
```bash
conda create -n env_g1_deploy python=3.11
conda activate env_g1_deploy
```

Install libraries :
```bash
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
pip install rich
pip install scipy
pip insatll redis
```

## 2️⃣ 🤖 Install Unitree SDK2 Python

Install [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python) in the G1 folder. Tutorial :

```bash
cd ~/G1
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
```

you must now download cyclonedds :

```bash
cd ~/G1
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```
Then add to path :

```bash
cd ~/G1/unitree_sdk2_python
export CYCLONEDDS_HOME="~/G1/cyclonedds/install"  # MODIFY THE PATH IF YOU DOWNLOADED CYCLONEDDS IN ANOTHER PLACE
pip3 install -e .
```

## 3️⃣ 🚀 Run the code

```bash
cd ~/G1/G1_deploy_python
python3 main.py
```
You should see :

 <p align="center">
  <img src="doc/im4.png" width="600">
  <br>
 </p>
 

---
## 4️⃣ 🏗️ Launch the Mujoco simulation

Now you should install [Unitree Mujoco](https://github.com/unitreerobotics/unitree_mujoco) in the G1 folder. Tutorial :

```bash
cd ~/G1
pip3 install mujoco
pip3 install pygame
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```
You must replace the config files in unitree_mujoco/simulate_python by those in config_mujoco like this :

 <p align="center">
  <img src="doc/im7.png" width="600">
  <br>
 </p>

Now connect the remote to your PC and run the simulation with :
```bash
cd unitree_mujoco/simulate_python
python3 unitree_mujoco.py
```

You should see :
 <p align="center">
  <img src="doc/im5.png" width="700">
  <br>
 </p>
 
Press `9` to deactivate the elastic band and `7` / `8` to raise / lower the robot.
Use the remote to navigate in the UI.



---
## 5️⃣ 🤖 Run it on Real robot G1

1. **Connect the robot with ethernet**
   
   Turn on the robot and connect it to your PC using an Ethernet cable.
   Go to Settings/Network and then IPv4. Fill the gaps as follows :
   
   <p align="center">
    <img src="doc/ipv4.png" width="700">
    <br>
   </p>
2. **Collect the network interface**

   Type in the terminal, and collect the adress :

   ```bash
   ifconfig
   ```
   <p align="center">
    <img src="doc/net.png" width="700">
    <br>
   </p>
3. **Switch the code in real mode**
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
