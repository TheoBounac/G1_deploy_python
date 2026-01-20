"""La classe State est l'interface pour tous les états du robot"""
class State:
    def __init__(self, fsm, name: str): 
        """Un State est définie par :"""
        self.fsm = fsm          # Sa fsm à laquelle il est lié, ce qui permet aussi de le lier à son controller
        self.name = name        # Son nom

    def enter(self):
        """Appelé quand on ENTRE dans cet état."""
        raise NotImplementedError("enter() function must be implement!")

    def step(self):
        """Appelé à CHAQUE tick (chaque appel de Run)."""
        raise NotImplementedError("step() function must be implement!")

    def exit(self):
        """Appelé quand on QUITTE cet état."""
        raise NotImplementedError("exit() function must be implement!")
    
    def send_start_pose(self):
        raise NotImplementedError("send_start_pose() function must be implement!")