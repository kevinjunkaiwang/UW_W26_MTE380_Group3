"""
Reference implementation of the paper's fuzzy rule-based scheduler.
Inputs:
  X1: current motor speed percentage (0-100)
  X2: detected line length percentage (0-100)
Output:
  X*: defuzzified scheduler output in [1, 100], plus label and PID tuple.
"""
from dataclasses import dataclass
from typing import Dict, Tuple, List
import numpy as np


def tri(x: float, a: float, b: float, c: float) -> float:
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


@dataclass
class FuzzySet:
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
    def __init__(self):
        self.x1_sets = X1_SETS
        self.x2_sets = X2_SETS
        self.out_sets = XOUT_SETS

    def infer_label(self, x1: float, x2: float) -> Dict[str, float]:
        """Return activation (mu) for each output label after MIN+MAX composition."""
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
        """Return (X*, winning_label, pid_tuple)."""
        x1 = float(np.clip(x1, 0, 100))
        x2 = float(np.clip(x2, 0, 100))
        acts = self.infer_label(x1, x2)
        samples = np.linspace(0, 100, 101)
        x_star = self.defuzzify(acts, samples)

        # pick label with highest activation for mapping PID set
        best_label = max(acts.items(), key=lambda kv: kv[1])[0]
        pid = PID_SETS[best_label]
        # quantize to 1..100 as in the paper
        x_star_q = max(1.0, min(100.0, round(x_star, 2)))
        return x_star_q, best_label, pid

