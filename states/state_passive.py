from controller.fsm import State 

class PassiveState(State):
    def __init__(self, fsm):
        super().__init__(fsm, "passive")
        self.ctrl = self.fsm.controller # Lien vers le controlleur des moteurs

    def enter(self):
        print("[FSM] Enter PASSIVE")

    def step(self):
        """Commande de moteurs nulle """
        ########### Consigne pour le controller ##################
        self.ctrl.target_dof_pos = [0] * 29                      #
        self.ctrl.Kp = [0] * 29                                  #
        self.ctrl.Kd = [0] * 29                                  #
        ##########################################################

    def exit(self):
        print("[FSM] Exit PASSIVE")