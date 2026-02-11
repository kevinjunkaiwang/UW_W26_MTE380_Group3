"""
End-to-end Python-only simulation:
- Convert raw signals to X1 (speed %) and X2 (line-length %)
- Run fuzzy scheduler to get speed cap + PID gains
- Apply PID on a given IR lateral error to produce wheel commands
"""
from dataclasses import dataclass
from typing import Tuple, List
import math

from paper_fuzzy import FuzzyScheduler


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class PIDState:
    integ: float = 0.0
    prev_e: float = 0.0


class Simulator:
    def __init__(self):
        self.sched = FuzzyScheduler()
        self.state = PIDState()

    def step(
        self,
        speed_cmd_norm: float,
        line_frac: float,
        ir_error_m: float,
        dt: float,
    ) -> dict:
        """
        Args:
            speed_cmd_norm: desired forward speed in [0,1]
            line_frac: fraction of max visible line length in [0,1]
            ir_error_m: lateral error from IR array (meters)
            dt: timestep (seconds)
        Returns:
            dict with X1, X2, X*, label, pid, base_v, u, left, right
        """
        # raw -> normalized fuzzy inputs
        x1 = clamp(speed_cmd_norm, 0, 1) * 100.0
        x2 = clamp(line_frac, 0, 1) * 100.0

        x_star, label, pid_tuple = self.sched.evaluate(x1, x2)
        v_cap, kp, ki, kd = pid_tuple

        base_v = min(speed_cmd_norm, v_cap)  # cap forward speed

        # PID on lateral error
        self.state.integ += ir_error_m * dt
        de = (ir_error_m - self.state.prev_e) / dt if dt > 0 else 0.0
        self.state.prev_e = ir_error_m
        u = kp * ir_error_m + ki * self.state.integ + kd * de

        left = clamp(base_v - u, -1.0, 1.0)
        right = clamp(base_v + u, -1.0, 1.0)

        return {
            "X1": x1,
            "X2": x2,
            "X*": x_star,
            "label": label,
            "pid": pid_tuple,
            "base_v": base_v,
            "u": u,
            "left": left,
            "right": right,
        }


def demo():
    sim = Simulator()
    scenarios: List[Tuple[float, float, float]] = [
        (0.6, 0.8, 0.00),   # fast, long line, centered
        (0.6, 0.3, 0.02),   # fast cmd, short line, slight right error
        (0.4, 0.2, -0.03),  # slower, very short line, left error
        (0.8, 0.9, 0.00),   # very fast, long line
        (0.3, 0.5, 0.00),   # moderate
    ]
    dt = 0.02  # 50 Hz control

    header = (
        f"{'speed':>5} {'line':>5} {'err(m)':>7}  {'X*':>6}  {'lbl':>3}  "
        f"{'base':>5} {'u':>7} {'left':>6} {'right':>6}   pid(vmax,kp,ki,kd)"
    )
    sep = "-" * len(header)
    print(header)
    print(sep)
    for speed_cmd, line_frac, err in scenarios:
        out = sim.step(speed_cmd, line_frac, err, dt)
        print(
            f"{speed_cmd:5.2f} {line_frac:5.2f} {err:+7.3f}  {out['X*']:6.2f}  "
            f"{out['label']:>3s}  {out['base_v']:5.2f} {out['u']:+7.3f} "
            f"{out['left']:6.2f} {out['right']:6.2f}   {out['pid']}"
        )


if __name__ == "__main__":
    demo()
