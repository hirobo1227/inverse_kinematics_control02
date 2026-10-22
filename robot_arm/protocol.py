"""CubeMars V3 manual V1.0.1 §§2.2, 2.3.1. All positions in radians."""
import math
import struct
from dataclasses import dataclass


def quantize(value, bounds, bits):
    lo, hi = bounds
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{value} outside [{lo}, {hi}]')
    return int(((value - lo) / (hi - lo)) * ((1 << bits) - 1))


def pack_mit(model, position, velocity=0.0, kp=0.0, kd=0.0, torque=0.0):
    """V3 layout: KP, KD, position, velocity, torque (not legacy MIT)."""
    p = quantize(position, model['position_rad'], 16)
    v = quantize(velocity, model['velocity_rad_s'], 12)
    k = quantize(kp, model['kp'], 12)
    d = quantize(kd, model['kd'], 12)
    t = quantize(torque, model['torque_nm'], 12)
    return bytes((k >> 4, (k & 15) << 4 | d >> 8, d & 255,
                  p >> 8, p & 255, v >> 4, (v & 15) << 4 | t >> 8, t & 255))


@dataclass(frozen=True)
class Feedback:
    can_id: int
    position_rad: float
    speed_erpm: float
    current_a: float
    temperature_c: int
    fault: int
    received_at: float


def decode_feedback(arbitration_id, data, received_at):
    if arbitration_id >> 8 != 0x29 or len(data) != 8:
        return None
    p, v, i, temp, fault = struct.unpack('>hhh b B', data)
    return Feedback(arbitration_id & 255, math.radians(p * .1),
                    v * 10.0, i * .01, temp, fault, received_at)