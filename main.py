import time
from dataclasses import dataclass
from typing import List

from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from controller.controller import Control


ctrl = Control()
ctrl.Init()
ctrl.Start()


@dataclass(frozen=True)
class Transition:
    src: str
    dst: str
    key: str
    label: str


TRANSITIONS: List[Transition] = [
    Transition("passive", "default_static", "A", "Stand up to default pose"),
    Transition("default_static", "velocity", "START", "Enable RL velocity control --> Put th e robot on the ground before !"),
    Transition("default_static", "passive", "B", "Go passive"),
    Transition("velocity", "default_static", "B", "Stop RL and go default pose"),
    Transition("passive", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!! Be carefull : robot will fall"),
    Transition("default_static", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!! Be carefull : robot will fall"),
    Transition("velocity", "EMERGENCY STOP", "SELECT", "EMERGENCY STOP !!! Be carefull : robot will fall")
]

ALL_STATES = ["passive", "default_static", "velocity","EMERGENCY STOP"]


def get_cur_name(fsm) -> str:
    cs = getattr(fsm, "current_state", None)
    if cs is None:
        return "<?>"
    if hasattr(cs, "name"):
        return str(cs.name)
    return str(cs)


def render_ui(fsm) -> Panel:
    cur = get_cur_name(fsm)

    # --- States row ---
    states_line = Text()
    for i, s in enumerate(ALL_STATES):
        if i:
            states_line.append("   ")
        if s == cur:
            if (s == "EMERGENCY STOP"):
                states_line.append(f"[ {s} ]", style="bold black on bright_red")
            else :
                states_line.append(f"[ {s} ]", style="bold black on bright_green")
        else:
            states_line.append(f"  {s}  ", style="dim")

    # --- Transitions table ---
    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Key", width=6)
    table.add_column("From", width=16)
    table.add_column("To", width=16)
    table.add_column("Description")

    avail = [t for t in TRANSITIONS if t.src == cur]
    if avail:
        for t in avail:
            table.add_row(t.key, t.src, t.dst, t.label)
    else:
        table.add_row("-", cur, "-", "No transitions from this state")

    layout = Table.grid(expand=True)
    layout.add_row(Panel(states_line, title="FSM State", border_style="green"))
    layout.add_row(table)

    return Panel(layout, title="G1 Deploy FSM", border_style="bright_blue")


console = Console()
console.clear()
refresh_dt = 1.0 / 20.0

try:
    with Live(render_ui(ctrl.fsm), console=console, refresh_per_second=20, screen=True) as live:
        while ctrl.fin == 0:
            live.update(render_ui(ctrl.fsm), refresh=True)
            time.sleep(refresh_dt)
finally:
    ctrl.lowCmdWriteThreadPtr._RecurrentThread__quit = True

console.print("\n[bold]Exit[/bold]")
