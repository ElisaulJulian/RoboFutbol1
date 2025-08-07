import math

def normalize_angle(angle: float) -> float:
    """Normaliza para [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))
