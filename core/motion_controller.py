from .types import Pose, Vector2
from .math_utils import normalize_angle
import math

class DifferentialController:
    """
    Calcula velocidades de roda para um robô com tração diferencial.
    """
    def __init__(
        self,
        k_forward: float = 50.0,
        k_turn: float = 10.0,
        stop_distance: float = 0.10,
        angle_threshold: float = 0.10,
    ):
        self.k_forward = k_forward
        #qué tan fuerte acelera hacia el objetivo (control proporcional).

        self.k_turn = k_turn
        #cuánto gira

        self.stop_distance = stop_distance
        #si el robot está más cerca que esto del objetivo, frena.

        self.angle_threshold = angle_threshold
        #si el error angular es mayor a esto, no avanza (solo gira para alinearse).

    def goto_point(self, robot: Pose, target: Vector2) -> tuple[float, float]:
        dx, dy = target.x - robot.position.x, target.y - robot.position.y
        #diferencia entre posición actual y objetivo.

        distance = math.hypot(dx, dy)
        #distancia euclidiana al objetivo.

        angle_to_target = math.atan2(dy, dx)
        #dirección donde está el objetivo.

        angle_err = normalize_angle(angle_to_target - robot.theta)
        #cuánto tiene que girar el robot para mirar hacia el objetivo. (normalizado)

        # ZONA MUERTA: no corregimos si el error angular es mínimo
        if abs(angle_err) < 0.05:
            angle_err = 0.0

        # FRENADO PROGRESIVO: si está muy cerca, frena solo
        if distance < (2*self.stop_distance):
            forward = 20.0
        else:
            forward = self.k_forward * distance  # velocidad depende de distancia

        # Giro proporcional pero limitado
        turn = self.k_turn * angle_err
        max_turn = 10.0
        
        if angle_err < 0.6:
            turn = max(-max_turn, min(max_turn, turn))
        else:
            forward = -25.0

        return forward - turn, forward + turn  # (left_speed, right_speed)
