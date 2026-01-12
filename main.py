import time

from rich.console import Console
from rich.live import Live

from controller.controller import Control
from UI import UI


ctrl = Control()
ctrl.Init()
ctrl.Start()


console = Console()
console.clear()

ui = UI()
refresh_dt = 1.0 / 20.0
try:
    with Live(ui.render_ui(ctrl.fsm), console=console, refresh_per_second=20, screen=True) as live:
        while ctrl.fin == 0:
            live.update(ui.render_ui(ctrl.fsm), refresh=True)
            time.sleep(refresh_dt)
finally:
    ctrl.lowCmdWriteThreadPtr._RecurrentThread__quit = True

while ctrl.fin == 0:
    time.sleep(refresh_dt)
    
console.print("\n[bold]Exit[/bold]")
