from common.remote_controller import RemoteController, KeyMap

class Buttons:
    def __init__(self,controller):
        self.ctrl = controller
        self.prev_select = 0 # Etat boutons
        self.prev_A = 0      #
        self.prev_B = 0      #
        self.temps = 0

    def update_states_with_buttons(self):
        if (self.temps == 0) :
            A = self.ctrl.remote_controller.button[KeyMap.right]
            B = self.ctrl.remote_controller.button[KeyMap.left]
            select = self.ctrl.remote_controller.button[KeyMap.select]

            if A and not self.prev_A:
                if (self.ctrl.fsm.current_state.name=="passive"):
                    self.ctrl.fsm.set_state("default_static")
                elif (self.ctrl.fsm.current_state.name=="default_static"):
                    self.ctrl.fsm.set_state("velocity")

            if B and not self.prev_B:
                if (self.ctrl.fsm.current_state.name=="velocity"):
                    self.ctrl.fsm.set_state("default_static")
                elif (self.ctrl.fsm.current_state.name=="default_static"):
                    self.ctrl.fsm.set_state("passive")
            
            if select and not self.prev_select:
                if (self.ctrl.fsm.current_state.name=="velocity"):
                    self.ctrl.fsm.set_state("EMERGENCY STOP")
                elif (self.ctrl.fsm.current_state.name=="default_static"):
                    self.ctrl.fsm.set_state("EMERGENCY STOP")
                elif (self.ctrl.fsm.current_state.name=="passive"):
                    self.ctrl.fsm.set_state("EMERGENCY STOP")

            self.prev_A = A
            self.prev_B = B

            self.prev_select = select
        
        self.temps += 0.001
        if (self.temps >= 0.2):
            self.temps = 0

