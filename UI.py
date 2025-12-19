from dataclasses import dataclass
from typing import List, Optional

from rich.panel import Panel
from rich.table import Table
from rich.text import Text


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
    Transition("passive", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
    Transition("default_static", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
    Transition("velocity", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!!"),
]

ALL_STATES = ["passive", "default_static", "velocity"]


class UI:
    def __init__(self):
        pass

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
        if cur == "EMERGENCY STOP":
            return "Emergency stop engaged (robot may fall)."
        return "Unknown state."

    def render_ui(self, fsm) -> Panel:
        cur = self.get_cur_name(fsm)

        left_t = self._find_transition(cur, "LEFT")
        right_t = self._find_transition(cur, "RIGHT")
        sel_t = self._find_transition(cur, "SELECT")

        # --- Side arrows ---
        left_hint = (
            Text("← " + left_t.dst, style="bold") if left_t else Text("←", style="dim")
        )
        right_hint = (
            Text(right_t.dst + " →", style="bold") if right_t else Text("→", style="dim")
        )

        # --- States row ---
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

        # --- Layout ---
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

        # SELECT line only if NOT in emergency
        if cur != "EMERGENCY STOP" and sel_t is not None:
            content.add_row(Text(""))
            em_line = Text("SELECT ", style="bold")
            em_line.append(sel_t.label, style="bold black on bright_red")
            content.add_row(em_line)

        return Panel(content, title="G1 Deploy FSM", border_style="bright_blue")
