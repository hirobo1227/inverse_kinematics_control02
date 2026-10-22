"""Low-level motor access; all angles are native driver coordinates, radians."""
import math
import time
from .protocol import pack_mit, decode_feedback

def computeTaskSpaceForce(
        targetPosition,
        currentPosition,
        targetVelocity,
        currentVelocity,
        kp,
        kd,
):
    positionError = targetPosition - currentPosition
    velocityError = targetVelocity - currentVelocity
    return kp * positionError + kd * velocityError


class MotorController:
    """Caller supplies motion limits and handles kinematics, trajectories and stopping."""
    def __init__(self, transport, models_by_id, limits_by_id):
        self.transport = transport
        self.models = models_by_id
        self.limits = limits_by_id
        if set(models_by_id) != set(limits_by_id):
            raise ValueError('Limits required for every motor')
        for motor_id, limits in limits_by_id.items():
            if not isinstance(motor_id, int) or not 1 <= motor_id <= 255:
                raise ValueError('CAN ID must be 1..255')
            for name in ('position_rad','velocity_rad_s','torque_nm'):
                lo, hi = limits[name]
                model_lo, model_hi = models_by_id[motor_id][name]
                if not all(math.isfinite(x) for x in (lo,hi)) or not model_lo <= lo < hi <= model_hi:
                    raise ValueError('Invalid limit: ' + name)

    def command_motor(self, can_id, position_rad, velocity_rad_s=0.0,
                      kp=0.0, kd=0.0, torque_nm=0.0):
        """Send a V3 MIT command. kp/kd defaults produce no position holding."""
        model = self.models[can_id]
        limits = self.limits[can_id]
        for name, value in (('position_rad',position_rad),('velocity_rad_s',velocity_rad_s),('torque_nm',torque_nm)):
            lo, hi = limits[name]
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'{name}: {value} outside [{lo}, {hi}]')
        data = pack_mit(model,position_rad,velocity_rad_s,kp,kd,torque_nm)
        self.transport.send((8 << 8) | can_id,data)

    def move_motor(self, can_id, angle_deg, *, kp, kd):
        """Position command in degrees; caller must ramp targets to limit motion."""
        if kp <= 0:
            raise ValueError('Position control requires positive kp')
        self.command_motor(can_id,math.radians(angle_deg),kp=kp,kd=kd)

    def set_motor_velocity(self, can_id, velocity_rad_s, *, kd):
        if kd <= 0:
            raise ValueError('Velocity control requires positive kd')
        self.command_motor(can_id,0.0,velocity_rad_s=velocity_rad_s,kd=kd)

    def set_motor_torque(self, can_id, torque_nm):
        self.command_motor(can_id,0.0,torque_nm=torque_nm)

    def read_motor(self, can_id, timeout_s=.1):
        """Read the next queued status; no command or origin change is sent."""
        if can_id not in self.models or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Invalid CAN ID/timeout')
        return self.transport.read_motor(can_id,timeout_s)

    def close(self):
        self.transport.close()

    def disable_all(self):
        """Best-effort V3 disable (function ID 15, DLC 0); not a brake."""
        errors = []
        for canId in self.models:
            try:
                self.transport.send((15 << 8) | canId, b'')
            except Exception as error:
                errors.append((canId, error))
        return errors


class CanTransport:
    def __init__(self, channel='can0'):
        import can
        self.can = can
        self.bus = can.Bus(interface='socketcan',channel=channel)
        self.pendingStates = {}

    def read_motor(self, can_id, timeout_s):
        """Consume a pending status, or receive one while retaining other IDs.

        Only the latest encountered status per other ID is retained. Its
        received_at timestamp is preserved; pending data may be old.
        """
        if can_id in self.pendingStates:
            return self.pendingStates.pop(can_id)
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            msg = self.bus.recv(max(0,deadline-time.monotonic()))
            if msg is None:
                break
            if msg.is_extended_id and not msg.is_error_frame and not msg.is_remote_frame:
                state = decode_feedback(msg.arbitration_id,msg.data,time.monotonic())
                if state:
                    if state.can_id == can_id:
                        return state
                    self.pendingStates[state.can_id] = state
        raise TimeoutError(f'No feedback from CAN ID {can_id}')

    def send(self, arbitration_id, data):
        self.bus.send(self.can.Message(arbitration_id=arbitration_id,data=data,
                                      is_extended_id=True),timeout=.02)

    def close(self):
        # Closing CAN is not a motor stop command.
        self.bus.shutdown()