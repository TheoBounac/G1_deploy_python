from states.base_state import State   # Interface of State
from states.state_passive import PassiveState              # FSM va initialiser les 3 states crées, donc on les importe ici.
from states.state_default_static import DefaultStaticState # On rappel qu'un state (passive,default_static,velocity) est une 
from states.state_velocity import VelocityState            # sous classe de STATE qui redéfinie ses méthodes.
from states.emergency import Emergency                     #
from states.state_mocap import MocapState                  #

class FSM:
    def __init__(self, controller):
        self.controller = controller # <----------------- The controller, the one which created this FSM
        self.states: dict[str, State] = {               # The dictionary of available FSM states: this is where it is created
            "passive" : PassiveState(self),             #
            "default_static": DefaultStaticState(self), #
            "velocity": VelocityState(self),            #
            "EMERGENCY STOP": Emergency(self),          #
            "mocap": MocapState(self)                   #
        }                                               #
        self.current_state: State | None = None # <------ Initialize current_state at None

    ################ To switch state #################
    def set_state(self, state_name: str):            # 
        """Transition vers un autre état."""         #
        if self.current_state is not None:           #
            self.current_state.exit()                #
                                                     #
        self.current_state = self.states[state_name] #
        self.current_state.enter()                   #
    ##################################################

    ## Called at each control_dt step by RecurrentThread) ##
    def step(self):                                        #
        if self.current_state is not None:                 #
            self.current_state.step()                      #
    ########################################################

    ### Envoyer la position de départ pour mimic ###
    def send_start_pose(self):                     #
        if self.current_state is not None:         #
            self.current_state.send_start_pose()   #
    ################################################
