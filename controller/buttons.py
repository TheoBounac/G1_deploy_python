from common.remote_controller import KeyMap

class Buttons:
    def __init__(self, controller, ui):
        self.ctrl = controller
        self.ui = ui

        self.prev_select = 0
        self.prev_A = 0
        self.prev_B = 0
        self.prev_up = 0
        self.prev_down = 0

        self.temps = 0

    def update_states_with_buttons(self):
        
        self.temps += 0.001
        if self.temps >= 0.2:
            self.temps = 0

        if self.temps == 0:
            A = self.ctrl.remote_controller.button[KeyMap.right]
            B = self.ctrl.remote_controller.button[KeyMap.left]
            select = self.ctrl.remote_controller.button[KeyMap.select]
            start = self.ctrl.remote_controller.button[KeyMap.start]

            up = self.ctrl.remote_controller.button[KeyMap.up]
            down = self.ctrl.remote_controller.button[KeyMap.down]

            cur = self.ctrl.fsm.current_state.name

            # ------------------ Mocap menu controls ------------------
            if cur == "mocap":
                if up and not self.prev_up:
                    self.ui.mocap_move_up()

                if down and not self.prev_down:
                    self.ui.mocap_move_down()

                if start and not self.prev_start:
                    self.ui.mocap_confirm()

                # B reste “retour velocity”
                if B and not self.prev_B:
                    self.ctrl.fsm.set_state("velocity")

            # ------------------ Normal FSM navigation ------------------
            else:
                if A and not self.prev_A:
                    if cur == "passive":
                        self.ctrl.fsm.set_state("default_static")
                    elif cur == "default_static":
                        self.ctrl.fsm.set_state("velocity")
                    elif cur == "velocity":
                        self.ctrl.fsm.set_state("mocap")

                if B and not self.prev_B:
                    if cur == "velocity":
                        self.ctrl.fsm.set_state("default_static")
                    elif cur == "default_static":
                        self.ctrl.fsm.set_state("passive")

            # Emergency stop partout
            if select and not self.prev_select:
                self.ctrl.fsm.set_state("EMERGENCY STOP")

            self.prev_A = A
            self.prev_B = B
            self.prev_select = select
            self.prev_start = start
            self.prev_up = up
            self.prev_down = down
