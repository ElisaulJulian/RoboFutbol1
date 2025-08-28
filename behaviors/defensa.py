from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball, Vector2
import math

class Defensa(Behavior):
    def __init__(self, controller: DifferentialController | None = None):
        self.controller = controller or DifferentialController()
        self.last_ball_pos = None
        self.last_time = None
        self.modo_ataque = False

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

        # Parámetros de la cancha (ajusta según tu campo)
        arco_centro = Vector2(0.65, 0.0)  # Arco propio (ahora en el lado derecho)
        arco_rival = Vector2(-0.65, 0.0)  # Arco rival (lado izquierdo)
        distancia_defensa = 0.26
        distancia_ataque = 0.16  # Distancia para cambiar a modo ataque

        # Distancia del robot a la pelota
        dx = ball.position.x - robot.pose.position.x
        dy = ball.position.y - robot.pose.position.y
        dist_robot_ball = math.hypot(dx, dy)

        # Cambiar de modo según la distancia a la pelota
        if dist_robot_ball < distancia_ataque:
            self.modo_ataque = True
        elif dist_robot_ball > distancia_ataque * 1.5:
            self.modo_ataque = False

        if self.modo_ataque:
            # Target: predicción de intersección con la pelota
            v_robot = 1.0  # velocidad máxima del robot (ajusta según tu robot)
            target = predecir_interseccion_pelota(
                robot.pose.position, ball.position, ball_vel, v_robot
            )
        else:
            # Target: sobre la línea entre pelota y arco propio
            target = punto_sobre_recta_arco_pelota(ball.position, arco_centro, distancia_defensa)
            # Limitar target a la zona de defensa (por ejemplo, x >= 0)
            target.x = max(target.x, 0.0)

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

def predecir_interseccion_pelota(robot_pos: Vector2, ball_pos: Vector2, ball_vel: Vector2, v_robot: float) -> Vector2:
    """
    Calcula el punto donde el robot puede interceptar la pelota,
    considerando la velocidad de ambos.
    """
    dx = ball_pos.x - robot_pos.x
    dy = ball_pos.y - robot_pos.y
    a = ball_vel.x
    b = ball_vel.y
    A = (a**2 + b**2) - v_robot**2
    B = 2 * (dx * a + dy * b)
    C = dx**2 + dy**2
    discriminante = B**2 - 4*A*C
    if discriminante < 0 or abs(A) < 1e-6:
        return ball_pos
    t1 = (-B + math.sqrt(discriminante)) / (2*A)
    t2 = (-B - math.sqrt(discriminante)) / (2*A)
    t = min([ti for ti in [t1, t2] if ti > 0], default=None)
    if t is None:
        return ball_pos
    x = ball_pos.x + a * t
    y = ball_pos.y + b * t
    return Vector2(x, y)


