from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball, Vector2
import math



class Defensa(Behavior):
    def __init__(self, controller: DifferentialController | None = None):
        self.controller = controller or DifferentialController()
        self.last_ball_pos = None
        self.last_time = None

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        import time
        if ball is None:
            return 0.0, 0.0

        # Calcular velocidad de la pelota
        current_time = time.time()
        if self.last_ball_pos is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                ball_vel = Vector2(
                    (ball.position.x - self.last_ball_pos.x) / dt,
                    (ball.position.y - self.last_ball_pos.y) / dt
                )
            else:
                ball_vel = Vector2(0, 0)
        else:
            ball_vel = Vector2(0, 0)

        self.last_ball_pos = Vector2(ball.position.x, ball.position.y)
        self.last_time = current_time

        # Centro del arco propio (ajusta según tu campo)
        arco_centro = Vector2(0.65, 0.0)  # ejemplo: arco en x=-0.65, centro en y=0

        # Distancia deseada desde el arco (ajusta según tu estrategia)
        distancia = 0.26

        # Calcula el target sobre la recta arco-pelota
        target = punto_sobre_recta_arco_pelota(ball.position, arco_centro, distancia)

        # Limitar target a la zona de defensa (por ejemplo, x <= 0)
        target.x = max(target.x, 0.0)  # Cambia 0.0 por el límite de tu zona si es necesario

        return self.controller.goto_point_lim(robot.pose, target)


def punto_sobre_recta_arco_pelota(ball_pos: Vector2, arco_pos: Vector2, distancia: float) -> Vector2:
    """
    Devuelve el punto sobre la recta arco-pelota a 'distancia' del arco.
    """
    dx = ball_pos.x - arco_pos.x
    dy = ball_pos.y - arco_pos.y
    norm = math.hypot(dx, dy)
    if norm == 0:
        return Vector2(arco_pos.x, arco_pos.y)
    # Vector unitario de arco a pelota
    ux = dx / norm
    uy = dy / norm
    # Punto a 'distancia' del arco sobre la recta
    x = arco_pos.x + ux * distancia
    y = arco_pos.y + uy * distancia
    return Vector2(x, y)


