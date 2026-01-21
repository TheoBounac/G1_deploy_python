# UI.py
from dataclasses import dataclass
from typing import List, Optional
import os
import threading
import subprocess

from rich.panel import Panel
from rich.table import Table
from rich.text import Text
import time


@dataclass(frozen=True)
class Transition:
    src: str
    dst: str
    key: str
    label: str


# ------------------------------ Transitions ------------------------------
TRANSITIONS: List[Transition] = [
    Transition("passive", "default_static", "RIGHT", "Stand up to default pose"),
    Transition("default_static", "velocity", "RIGHT", "RL velocity control: put the robot on the ground before !"),
    Transition("default_static", "passive", "LEFT", "Go passive"),
    Transition("velocity", "default_static", "LEFT", "Stop RL and go default pose"),
    Transition("velocity", "mocap", "RIGHT", "Mocap control enabled"),
    Transition("mocap", "velocity", "LEFT", "Back to RL velocity control"),
    Transition("passive", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
    Transition("default_static", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
    Transition("velocity", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
    Transition("mocap", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
]

ALL_STATES = ["passive", "default_static", "velocity", "mocap"]


# ------------------------------ Mocap menu config ------------------------------
MOCAP_ROOT = "/home/theo/G1/TWIST/twist_motion_dataset/"

# Tu modifies ici les 3 mouvements
MOCAP_MOVES = [
    {"name": "Crouch", "file": "mocap_files/00043_simple_crouch.pkl"},
    {"name": "Gestures",   "file": "mocap_files/S1_Gestures_3.pkl"},
    {"name": "Kick",   "file": "mocap_files/S1_Box_3.pkl"},
]


class UI:
    def __init__(self):
        # sélection dans la liste (haut/bas)
        self.mocap_sel_idx: int = 0
        # lequel est “armé/actif” (en vert) après appui sur A
        self.mocap_active_idx: Optional[int] = None

        # thread/process pour lancer le script HL
        self._hl_thread: Optional[threading.Thread] = None
        self._hl_proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

        self.start_check = 0

    # ------------------------------ FSM helpers ------------------------------
    def get_cur_name(self, fsm) -> str:
        cs = getattr(fsm, "current_state", None)
        if cs is None:
            return "<?>"
        if hasattr(cs, "name"):
            return str(cs.name)
        return str(cs)

    def _find_transition(self, cur: str, key: str) -> Optional[Transition]:
        for t in TRANSITIONS:
            if t.src == cur and t.key == key:
                return t
        return None

    def _state_description(self, cur: str) -> str:
        if cur == "passive":
            return "Robot is passive (no active control)."
        if cur == "default_static":
            return "Robot holds the default standing posture. Put the robot on its feet before running velocity."
        if cur == "velocity":
            return "RL velocity control enabled (walking controller)."
        if cur == "mocap":
            return "Mocap mode active: use UP/DOWN to choose a motion, press A to play it."
        if cur == "EMERGENCY STOP":
            return "Emergency stop engaged (robot may fall)."
        return "Unknown state."

    # ------------------------------ Mocap control (called by Buttons) ------------------------------
    def mocap_move_up(self):
        self.mocap_sel_idx = (self.mocap_sel_idx - 1) % len(MOCAP_MOVES)

    def mocap_move_down(self):
        self.mocap_sel_idx = (self.mocap_sel_idx + 1) % len(MOCAP_MOVES)

    def mocap_confirm(self):
        """Appui sur START en mode mocap."""
        idx = self.mocap_sel_idx
        self.mocap_active_idx = idx
        motion_file = MOCAP_MOVES[idx]["file"]
        self._start_high_level_motion(motion_file)

    

    #################################################################################################################
    # [MOTION] Start high-level motion server in separate thread (env=twist) — NO FILE LOGS                         #
    #################################################################################################################
    def _start_high_level_motion(self, motion_file: str):                                                           #
        """Lance server_high_level_motion_lib.py dans un thread séparé (env=twist)."""                              #
                                                                                                                    #
        def _runner():                                                                                              #
            # Commande: exécuter dans l'env twist                                                                   #
            cmd = [                                                                                                 #
                "conda", "run", "-n", "twist",                                                                      #
                "python", "controller/server_high_level_motion_lib.py",                                             #
                "--motion_file", motion_file,                                                                       #
            ]                                                                                                       #
                                                                                                                    #
            with self._lock:                                                                                        #
                # Stop l'ancien HL si besoin                                                                        #
                if self._hl_proc is not None and self._hl_proc.poll() is None:                                      #
                    try:                                                                                            #
                        self._hl_proc.terminate()                                                                   #
                        self._hl_proc.wait(timeout=1.0)                                                             #
                    except Exception:                                                                               #
                        try:                                                                                        #
                            self._hl_proc.kill()                                                                    #
                        except Exception:                                                                           #
                            pass                                                                                    #
                    self._hl_proc = None                                                                            #
                                                                                                                    #
                try:                                                                                                #
                    # IMPORTANT: redirect stdout/stderr vers /dev/null pour ne pas casser rich.Live                 #
                    self._hl_proc = subprocess.Popen(                                                               #
                        cmd,                                                                                        #
                        cwd=os.getcwd(),                                                                            #
                        stdout=subprocess.DEVNULL,                                                                  #
                        stderr=subprocess.DEVNULL,                                                                  #
                        start_new_session=True,                                                                     #
                    )                                                                                               #
                except Exception as e:                                                                              #
                    self.mocap_active_idx = None                                                                    #
                    print(f"[UI] Failed to start high-level motion: {e}")                                           #
                    return                                                                                          #
                                                                                                                    #
            # Surveille la fin du process (sans bloquer l'UI)                                                       #
            while True:                                                                                             #
                with self._lock:                                                                                    #
                    p = self._hl_proc                                                                               #
                if p is None:                                                                                       #
                    break                                                                                           #
                if p.poll() is not None:                                                                            #
                    break                                                                                           #
                time.sleep(0.05)                                                                                    #
                                                                                                                    #
            # Quand HL fini, on "désactive" le vert                                                                 #
            with self._lock:                                                                                        #
                self.mocap_active_idx = None                                                                        #
                                                                                                                    #
        self.start_check += 1                                                                                       #
        t = threading.Thread(target=_runner, daemon=True)                                                           #
        self._hl_thread = t                                                                                         #
        t.start()                                                                                                   #
    #################################################################################################################


    # ------------------------------ Render ------------------------------
    def render_ui(self, fsm) -> Panel:
        cur = self.get_cur_name(fsm)

        left_t = self._find_transition(cur, "LEFT")
        right_t = self._find_transition(cur, "RIGHT")
        sel_t = self._find_transition(cur, "SELECT")

        left_hint = Text("← " + left_t.dst, style="bold") if left_t else Text("←", style="dim")
        right_hint = Text(right_t.dst + " →", style="bold") if right_t else Text("→", style="dim")

        states_line = Text()
        for i, s in enumerate(ALL_STATES):
            if i:
                states_line.append("       ")
            if s == cur:
                states_line.append(f"[ {s} ]", style="bold black on bright_green")
            else:
                states_line.append(f"  {s}  ", style="dim")

        if cur == "EMERGENCY STOP":
            states_line = Text(f"[ {cur} ]", style="bold black on bright_red")

        desc_line = Text(self._state_description(cur), style="dim")

        top_row = Table.grid(expand=True)
        top_row.add_column(justify="left", ratio=1)
        top_row.add_column(justify="center", ratio=2)
        top_row.add_column(justify="right", ratio=1)
        top_row.add_row(left_hint, states_line, right_hint)

        content = Table.grid(expand=True)
        content.add_column(justify="center")
        content.add_row(top_row)
        content.add_row(Text(""))
        content.add_row(desc_line)

        # ---- Mocap menu ----
        if cur == "mocap":
            content.add_row(Text(""))
            menu = Table.grid(padding=(0, 2))
            menu.add_column(justify="left")

            menu.add_row(Text("MOCAP MOVES", style="bold bright_cyan"))
            for i, m in enumerate(MOCAP_MOVES):
                name = m["name"]
                is_sel = (i == self.mocap_sel_idx)
                is_active = (self.mocap_active_idx == i)

                line = Text()
                prefix = "▶ " if is_sel else "  "
                line.append(prefix)

                if is_active:
                    line.append(name, style="bold bright_green")
                elif is_sel:
                    line.append(name, style="bold black on white")
                else:
                    line.append(name, style="dim")

                menu.add_row(line)

            menu.add_row(Text(""))
            menu.add_row(Text("UP/DOWN: choose mocap    START: play", style="bold"))
            content.add_row(menu)

        if cur != "EMERGENCY STOP" and sel_t is not None:
            content.add_row(Text(""))
            em_line = Text("SELECT ", style="bold")
            em_line.append(sel_t.label, style="bold black on bright_red")
            content.add_row(em_line)

        return Panel(content, title="G1 Deploy FSM", border_style="bright_blue")
