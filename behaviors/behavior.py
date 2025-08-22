from abc import ABC, abstractmethod
from core.types import Robot, Ball

class Behavior(ABC):
    """
    Interface básica para qualquer comportamento.
    """
    @abstractmethod
    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        """Devolve (left_speed, right_speed)."""

    
    

    