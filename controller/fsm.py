from states.base_state import State   # Interface de State
from states.state_passive import PassiveState              # FSM va initialiser les 3 states crées, donc on les importe ici
from states.state_default_static import DefaultStaticState # On rappel qu'un state (passive,default_static,velocity) est une 
from states.state_velocity import VelocityState            # sous classe de STATE qui redéfinie ses méthodes.
from states.emergency import Emergency                     #

class FSM:
    def __init__(self, controller):
        self.controller = controller # Le controller, celui qui vient de créer cette fsm
        self.states: dict[str, State] = {               # Le dictionnaire des états disponibles
            "passive" : PassiveState(self),             #
            "default_static": DefaultStaticState(self), #
            "velocity": VelocityState(self),            #
            "EMERGENCY STOP": Emergency(self)           #
        }                                               #
        self.current_state: State | None = None # On initialise current_state à None

    def set_state(self, state_name: str):            # Pour switcher d'état
        """Transition vers un autre état."""         #
        if self.current_state is not None:           #
            self.current_state.exit()                #
                                                     #
        self.current_state = self.states[state_name] #
        self.current_state.enter()                   #

    def step(self):
        """Appelé à chaque cycle de contrôle (par le thread RecurrentThread)."""
        if self.current_state is not None:
            self.current_state.step()
