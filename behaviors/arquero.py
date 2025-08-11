from .behavior import Behavior
from core.types import Vector2, Robot, Ball
from core.math_utils import normalize_angle
import math, time

class Arquero(Behavior):
    def __init__(
        self,
        goalie_orientation: float = math.pi/2,
        k_turn: float = 10.0, 
        k_move: float = 90.0,  
        angle_threshold: float = 0.2,
        stop_tolerance: float = 0.02,
        zone_min_x: float =  0.5,
        zone_max_x: float =  0.51,
        zone_min_y: float = -0.4,
        zone_max_y: float =  0.4,
        prediction_time: float = 0.3
    ):
        self.goalie_orientation = goalie_orientation
        self.k_turn            = k_turn
        self.k_move            = k_move
        self.angle_threshold   = angle_threshold
        self.stop_tolerance    = stop_tolerance
        self.zone_min_x        = zone_min_x
        self.zone_max_x        = zone_max_x
        self.zone_min_y        = zone_min_y
        self.zone_max_y        = zone_max_y
        self.prediction_time   = prediction_time

        # Para calcular velocidad de la pelota
        self.last_ball_pos = None 
        self.last_time = None

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        pos = robot.pose.position

        # Si no hay pelota, volver al centro de la portería
        if ball is None:
            target = Vector2(
                (self.zone_min_x + self.zone_max_x) / 2,
                (self.zone_min_y + self.zone_max_y) / 2
            )
        else:
            # Calcular velocidad de la pelota
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

            # Posición predicha
            predicted_pos = Vector2(
                ball.position.x + ball_vel.x * self.prediction_time,
                ball.position.y + ball_vel.y * self.prediction_time
            )

            # Si la pelota viene hacia el arco, usamos predicción
            raw = predicted_pos if ball_vel.x < 0 else ball.position

            # Clamp: restringimos SOLO EN Y (no movemos al arquero en X)
            target = Vector2(
                (self.zone_min_x + self.zone_max_x) / 2,  # X fija: centro de la zona
                max(self.zone_min_y, min(self.zone_max_y, raw.y))
            )

            # Ajustar k_move dinámicamente según la rapidez de la pelota
            ball_speed = math.hypot(ball_vel.x, ball_vel.y)
            dynamic_k_move = self.k_move + min(ball_speed * 50, 150)  # gana velocidad extra

        # Calcular diferencias
        dx = target.x - pos.x
        dy = target.y - pos.y
        dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        err_ang = normalize_angle(target_angle - robot.pose.theta)

        # Si no hay pelota, usar velocidad base. Si hay, usar dinámica.
        forward_gain = dynamic_k_move if ball else self.k_move
        turn_gain = self.k_turn

        # Movimiento
        forward = forward_gain * math.cos(err_ang) * dist
        turn = turn_gain * err_ang

        # Ajuste si la pelota está muy cerca
        if ball and math.hypot(ball.position.x - pos.x, ball.position.y - pos.y) < 0.2:
            forward = 30

        # Velocidades diferenciales
        left = forward - turn
        right = forward + turn

        return left, right
