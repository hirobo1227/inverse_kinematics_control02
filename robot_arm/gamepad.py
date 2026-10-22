"""Raw PS5/SDL inputs; no motor commands or kinematics."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GamepadState:
    connected: bool
    axes: tuple
    buttons: tuple
    hats: tuple


def apply_deadzone(value, deadzone=.12):
    if not math.isfinite(value) or not -1 <= value <= 1 or not 0 <= deadzone < 1:
        raise ValueError('Invalid stick value/deadzone')
    return 0.0 if abs(value) <= deadzone else math.copysign((abs(value)-deadzone)/(1-deadzone),value)


class PS5Controller:
    def __init__(self, index=0):
        import pygame
        self.pygame = pygame
        pygame.display.init()
        pygame.joystick.init()
        if index < 0 or index >= pygame.joystick.get_count():
            raise RuntimeError('Controller not connected')
        self.joy = pygame.joystick.Joystick(index)
        self.joy.init()
        self.name = self.joy.get_name()
        self.instance_id = self.joy.get_instance_id()
        self.connected = True

    def read_controller(self):
        """Pump SDL events and return raw arrays; inspect indices on the target OS."""
        if not self.connected:
            return GamepadState(False,(),(),())
        for event in self.pygame.event.get():
            if event.type == self.pygame.JOYDEVICEREMOVED and event.instance_id == self.instance_id:
                self.connected = False
        if not self.connected or not self.joy.get_init():
            self.connected = False
            return GamepadState(False,(),(),())
        return GamepadState(True,
            tuple(self.joy.get_axis(i) for i in range(self.joy.get_numaxes())),
            tuple(bool(self.joy.get_button(i)) for i in range(self.joy.get_numbuttons())),
            tuple(self.joy.get_hat(i) for i in range(self.joy.get_numhats())))

    def close(self):
        self.connected = False
        self.joy.quit()

    def __enter__(self):
        return self

    def __exit__(self,*args):
        self.close()