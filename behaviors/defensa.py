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

        # Velocidad máxima del robot (ajusta según tu robot)
        v_robot = 1.0  # metros/segundo, por ejemplo

        # Calcular punto de intersección
        target = predecir_interseccion_pelota(
            robot.pose.position, ball.position, ball_vel, v_robot
        )

         # Limitar target a la zona de defensa (por ejemplo, x <= 0)
        target.x = max(target.x, 0.0)  # Cambia 0.0 por el límite de tu zona si es necesario

        return self.controller.goto_point_lim(robot.pose, target)


def predecir_interseccion_pelota(robot_pos: Vector2, ball_pos: Vector2, ball_vel: Vector2, v_robot: float) -> Vector2:
    """
    Calcula el punto donde el robot puede interceptar la pelota, 
    considerando la velocidad de ambos.
    """
    # Diferencias iniciales
    dx = ball_pos.x - robot_pos.x
    dy = ball_pos.y - robot_pos.y

    # Velocidad relativa
    a = ball_vel.x
    b = ball_vel.y

    # Coeficientes para la ecuación cuadrática At^2 + Bt + C = 0
    A = (a**2 + b**2) - v_robot**2
    B = 2 * (dx * a + dy * b)
    C = dx**2 + dy**2

    # Resolver la cuadrática
    discriminante = B**2 - 4*A*C
    if discriminante < 0 or abs(A) < 1e-6:
        # No hay solución real o A ≈ 0 (evitar división por cero)
        return ball_pos

    t1 = (-B + math.sqrt(discriminante)) / (2*A)
    t2 = (-B - math.sqrt(discriminante)) / (2*A)

    # Tomar el menor t positivo
    t = min([ti for ti in [t1, t2] if ti > 0], default=None)
    if t is None:
        return ball_pos

    # Punto de intersección
    x = ball_pos.x + a * t
    y = ball_pos.y + b * t
    return Vector2(x, y)
    
    
    