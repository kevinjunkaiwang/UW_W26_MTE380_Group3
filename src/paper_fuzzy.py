"""
Reference implementation of the paper's fuzzy rule-based scheduler.

Inputs:
- X1: current motor speed percentage (0..100)
- X2: detected line length percentage (0..100)

Outputs:
- X*: defuzzified scalar output in [1, 100]
- label: winning fuzzy mode (LC, LF, MC, MF, HC, HF)
- pid_tuple: (v_cap, Kp, Ki, Kd) mapped from label

Notes:
- Membership breakpoints and PID sets are implementation-level tunables.
- Rule base follows Table I structure from the paper.
"""
from dataclasses import dataclass
from typing import Dict, Tuple, List, Optional
import numpy as np


def tri(x: float, a: float, b: float, c: float) -> float:
    """
    Triangular/shoulder membership function.

    Supports:
    - left shoulder  (a == b < c)
    - right shoulder (a < b == c)
    - standard triangle (a < b < c)
    - degenerate singleton (a == b == c)
    """
    # Left shoulder (a == b): full membership at and left of b, then falls to c.
    if a == b and b < c:
        if x <= b:
            return 1.0
        if x >= c:
            return 0.0
        return (c - x) / (c - b)

    # Right shoulder (b == c): rises from a to b, then full membership at and right of b.
    if a < b and b == c:
        if x <= a:
            return 0.0
        if x >= b:
            return 1.0
        return (x - a) / (b - a)

    # Degenerate singleton.
    if a == b == c:
        return 1.0 if x == a else 0.0

    # Standard triangular set.
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


@dataclass
class FuzzySet:
    """Simple fuzzy set wrapper around membership-function parameters."""
    name: str
    a: float
    b: float
    c: float

    def mu(self, x: float) -> float:
        return tri(x, self.a, self.b, self.c)


# Input membership definitions (heuristic, as paper does not provide exact numbers)
X1_SETS: List[FuzzySet] = [
    FuzzySet("Low", 0, 0, 40),
    FuzzySet("Medium", 20, 50, 80),
    FuzzySet("High", 60, 100, 100),
]

X2_SETS: List[FuzzySet] = [
    FuzzySet("Close", 0, 0, 50),
    FuzzySet("Far", 30, 100, 100),
]

# Output membership definitions (LC, LF, MC, MF, HC, HF)
XOUT_SETS: Dict[str, FuzzySet] = {
    "LC": FuzzySet("LC", 0, 10, 25),
    "LF": FuzzySet("LF", 15, 30, 45),
    "MC": FuzzySet("MC", 35, 50, 65),
    "MF": FuzzySet("MF", 55, 70, 85),
    "HC": FuzzySet("HC", 70, 85, 95),
    "HF": FuzzySet("HF", 85, 100, 100),
}

# Rule base (Table I)
RULES: Dict[Tuple[str, str], str] = {
    ("Low", "Close"): "LC",
    ("Low", "Far"): "LF",
    ("Medium", "Close"): "MC",
    ("Medium", "Far"): "MF",
    ("High", "Close"): "HC",
    ("High", "Far"): "HF",
}

# Example PID sets (speed cap, kp, ki, kd) per output label
PID_SETS: Dict[str, Tuple[float, float, float, float]] = {
    "LC": (0.30, 0.80, 0.00, 0.10),
    "LF": (0.35, 0.70, 0.00, 0.10),
    "MC": (0.45, 0.65, 0.00, 0.12),
    "MF": (0.55, 0.55, 0.00, 0.14),
    "HC": (0.65, 0.45, 0.00, 0.16),
    "HF": (0.75, 0.40, 0.00, 0.18),
}


class FuzzyScheduler:
    """Mamdani fuzzy scheduler with centroid defuzzification."""

    def __init__(
        self,
        blend_gains: bool = True,
        pid_smoothing: float = 0.8,
        sample_step: float = 0.25,
        x1_alpha: float = 1.0,
        x2_alpha: float = 0.85,
        x_star_alpha: float = 0.7,
    ):
        self.x1_sets = X1_SETS
        self.x2_sets = X2_SETS
        self.out_sets = XOUT_SETS
        self.blend_gains = bool(blend_gains)
        self.pid_smoothing = float(np.clip(pid_smoothing, 0.0, 1.0))
        self.sample_step = max(0.1, float(sample_step))
        self.x1_alpha = float(np.clip(x1_alpha, 0.0, 1.0))
        self.x2_alpha = float(np.clip(x2_alpha, 0.0, 1.0))
        self.x_star_alpha = float(np.clip(x_star_alpha, 0.0, 1.0))
        self.output_order = list(XOUT_SETS.keys())
        self._x1_f: Optional[float] = None
        self._x2_f: Optional[float] = None
        self._x_star_f: Optional[float] = None
        self._pid_smoothed: Optional[Tuple[float, float, float, float]] = None

    @staticmethod
    def _ema(prev: Optional[float], current: float, alpha: float) -> float:
        if prev is None:
            return current
        return float((1.0 - alpha) * prev + alpha * current)

    def _fuse_inputs(self, x1: float, x2: float) -> Tuple[float, float]:
        """Low-pass filter X1/X2 to reduce label jitter from noisy sensing."""
        self._x1_f = self._ema(self._x1_f, x1, self.x1_alpha)
        self._x2_f = self._ema(self._x2_f, x2, self.x2_alpha)
        return self._x1_f, self._x2_f

    def _fuse_x_star(self, x_star: float) -> float:
        """Low-pass filter the defuzzified output before quantization."""
        self._x_star_f = self._ema(self._x_star_f, x_star, self.x_star_alpha)
        return self._x_star_f

    def choose_label(self, x_star_q: float, activations: Optional[Dict[str, float]] = None) -> str:
        """
        Pick the mode whose output-set peak is closest to quantized x*.

        If activations are provided, limit candidates to labels that are currently active.
        This prevents selecting labels that have zero rule support (e.g., LF at x2=0).
        """
        if activations is None:
            candidates = self.output_order
        else:
            candidates = [label for label in self.output_order if activations.get(label, 0.0) > 0.0]
            if not candidates:
                candidates = self.output_order
        return min(candidates, key=lambda label: abs(x_star_q - XOUT_SETS[label].b))

    def blend_pid(self, activations: Dict[str, float]) -> Tuple[float, float, float, float]:
        """
        Blend PID tuples by output activation strength for smoother transitions.
        """
        num_v = num_kp = num_ki = num_kd = 0.0
        den = 0.0
        for label, mu_rule in activations.items():
            if mu_rule <= 0.0:
                continue
            v_cap, kp, ki, kd = PID_SETS[label]
            num_v += v_cap * mu_rule
            num_kp += kp * mu_rule
            num_ki += ki * mu_rule
            num_kd += kd * mu_rule
            den += mu_rule
        if den <= 1e-9:
            return PID_SETS["LC"]
        inv = 1.0 / den
        return (num_v * inv, num_kp * inv, num_ki * inv, num_kd * inv)

    def infer_label(self, x1: float, x2: float) -> Dict[str, float]:
        """
        Compute output-label activations via Mamdani MIN/MAX inference.

        - AND operator: min(mu_x1, mu_x2)
        - OR aggregation per output label: max(...)
        """
        activations = {name: 0.0 for name in self.out_sets}
        for s1 in self.x1_sets:
            mu1 = s1.mu(x1)
            if mu1 == 0:
                continue
            for s2 in self.x2_sets:
                mu2 = s2.mu(x2)
                if mu2 == 0:
                    continue
                out_label = RULES[(s1.name, s2.name)]
                mu = min(mu1, mu2)  # AND = MIN
                activations[out_label] = max(activations[out_label], mu)  # OR = MAX
        return activations

    def defuzzify(self, activations: Dict[str, float], samples: np.ndarray) -> float:
        """Centroid defuzzification over sampled output universe."""
        num = 0.0
        den = 0.0
        for x in samples:
            mu_x = 0.0
            for label, mu_rule in activations.items():
                if mu_rule == 0:
                    continue
                mu_set = self.out_sets[label].mu(x)
                mu_x = max(mu_x, min(mu_rule, mu_set))
            num += mu_x * x
            den += mu_x
        return num / den if den > 1e-6 else 0.0

    def evaluate(self, x1: float, x2: float) -> Tuple[float, str, Tuple[float, float, float, float]]:
        """
        Evaluate scheduler end-to-end.

        Returns:
        - x_star_q: defuzzified and filtered quantized output in [1, 100]
        - best_label: output mode from nearest label peak to x_star_q
        - pid: smoothed PID tuple, optionally activation blended
        """
        x1 = float(np.clip(x1, 0.0, 100.0))
        x2 = float(np.clip(x2, 0.0, 100.0))
        x1_f, x2_f = self._fuse_inputs(x1, x2)
        acts = self.infer_label(x1_f, x2_f)
        samples = np.arange(0.0, 100.0 + self.sample_step, self.sample_step)
        x_star = self.defuzzify(acts, samples)
        x_star_f = self._fuse_x_star(x_star)

        x_star_q = float(max(1.0, min(100.0, round(x_star_f, 2))))
        best_label = self.choose_label(x_star_q, acts)
        raw_pid = self.blend_pid(acts) if self.blend_gains else PID_SETS[best_label]
        if self.pid_smoothing <= 0.0:
            return x_star_q, best_label, raw_pid

        if self._pid_smoothed is None:
            self._pid_smoothed = raw_pid
        else:
            prev_v, prev_kp, prev_ki, prev_kd = self._pid_smoothed
            raw_v, raw_kp, raw_ki, raw_kd = raw_pid
            self._pid_smoothed = (
                float(self.pid_smoothing * prev_v + (1.0 - self.pid_smoothing) * raw_v),
                float(self.pid_smoothing * prev_kp + (1.0 - self.pid_smoothing) * raw_kp),
                float(self.pid_smoothing * prev_ki + (1.0 - self.pid_smoothing) * raw_ki),
                float(self.pid_smoothing * prev_kd + (1.0 - self.pid_smoothing) * raw_kd),
            )
        return x_star_q, best_label, self._pid_smoothed

