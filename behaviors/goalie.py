import time
from .behavior import Behavior
from core.types import Vector2, Robot, Ball
from core.math_utils import normalize_angle
import math

class Goalie(Behavior):
    def __init__(
        self,
        goalie_orientation: float = math.pi/2,
        k_turn: float = 10.0,
        k_move: float = 90.0,
        angle_threshold: float = 0.2,
        stop_tolerance: float = 0.02,
        zone_min_x: float = 0.5,
        zone_max_x: float = 0.8,
        zone_min_y: float = -0.4,
        zone_max_y: float = 0.4,
        prediction_time: float = 0.3   # segundos a futuro para anticipar
    ):
        self.goalie_orientation = goalie_orientation
        self.k_turn = k_turn
        self.k_move = k_move
        self.angle_threshold = angle_threshold
        self.stop_tolerance = stop_tolerance
        self.zone_min_x = zone_min_x
        self.zone_max_x = zone_max_x
        self.zone_min_y = zone_min_y
        self.zone_max_y = zone_max_y
        self.prediction_time = prediction_time

        # Para calcular velocidad de la pelota
        self.last_ball_pos = None
        self.last_time = None

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        pos = robot.pose.position

        # Si no hay pelota: centro de la zona
        if ball is None:
            target = Vector2(
                (self.zone_min_x + self.zone_max_x) / 2,
                (self.zone_min_y + self.zone_max_y) / 2
            )
        else:
            # Tiempo y velocidad de la pelota
            current_time = time.time()
            ball_vel = Vector2(0, 0)
            if self.last_ball_pos is not None and self.last_time is not None:
                dt = current_time - self.last_time
                if dt > 0:
                    ball_vel = Vector2(
                        (ball.position.x - self.last_ball_pos.x) / dt,
                        (ball.position.y - self.last_ball_pos.y) / dt
                    )
            self.last_ball_pos = Vector2(ball.position.x, ball.position.y)
            self.last_time = current_time

            # Predicción: dónde estará la pelota en "prediction_time"
            predicted_pos = Vector2(
                ball.position.x + ball_vel.x * self.prediction_time,
                ball.position.y + ball_vel.y * self.prediction_time
            )

            # Si la pelota viene hacia el arco: usar predicción, si no, usar posición actual
            raw = predicted_pos if ball_vel.x < 0 else ball.position

            # Clamp a la zona del arquero
            target = Vector2(
                max(self.zone_min_x, min(self.zone_max_x, raw.x)),
                max(self.zone_min_y, min(self.zone_max_y, raw.y)),
            )

        # Cálculo de movimiento hacia el target
        dx = target.x - pos.x
        dy = target.y - pos.y
        dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        err_ang = normalize_angle(target_angle - robot.pose.theta)

        forward = self.k_move * math.cos(err_ang) * dist
        turn = self.k_turn * err_ang

        # Si la pelota está muy cerca, desacelerar
        dx_ball = ball.position.x - robot.pose.position.x if ball else 0
        dy_ball = ball.position.y - robot.pose.position.y if ball else 0
        if ball and (dx_ball**2 + dy_ball**2) ** 0.5 < 0.2:
            forward = 30

        # Velocidades ruedas
        left = forward - turn
        right = forward + turn

        return left, right, False #El false está como devolución del should_kick que acá aún no se usa
