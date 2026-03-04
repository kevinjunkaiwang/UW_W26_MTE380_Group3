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
from typing import Dict, Tuple, List
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

    def __init__(self):
        self.x1_sets = X1_SETS
        self.x2_sets = X2_SETS
        self.out_sets = XOUT_SETS

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
        - x_star_q: defuzzified and quantized output in [1, 100]
        - best_label: max-activation output label
        - pid: (v_cap, Kp, Ki, Kd) mapped from best_label
        """
        x1 = float(np.clip(x1, 0, 100))
        x2 = float(np.clip(x2, 0, 100))
        acts = self.infer_label(x1, x2)
        samples = np.linspace(0, 100, 101)
        x_star = self.defuzzify(acts, samples)

        # Pick label with highest activation for mapping PID set.
        best_label = max(acts.items(), key=lambda kv: kv[1])[0]
        pid = PID_SETS[best_label]
        # Quantize/clamp to 1..100 as in paper-style output range.
        x_star_q = max(1.0, min(100.0, round(x_star, 2)))
        return x_star_q, best_label, pid

