import time

from rich.console import Console
from rich.live import Live

from controller.controller import Control
from controller.UI import UI

console = Console()
console.clear()

ui = UI()
refresh_dt = 1.0 / 20.0

ctrl = Control(ui)
ctrl.Init()
ctrl.Start()

try:
    with Live(ui.render_ui(ctrl.fsm), console=console, refresh_per_second=20, screen=True) as live:
        while True:
            live.update(ui.render_ui(ctrl.fsm), refresh=True)
            time.sleep(refresh_dt)
finally:
    ctrl.lowCmdWriteThreadPtr._RecurrentThread__quit = True

while True:
    time.sleep(refresh_dt)
    
console.print("\n[bold]Exit[/bold]")
