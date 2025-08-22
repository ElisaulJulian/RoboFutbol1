from .behavior import Behavior
from core.types import Vector2, Robot, Ball
from core.math_utils import normalize_angle
from core.motion_controller import DifferentialController
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
        self.controller        = DifferentialController(k_move, k_turn, -10, angle_threshold, angle_threshold*2)

        # Para calcular velocidad de la pelota
        self.last_ball_pos = None 
        self.last_time = None

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        # Si no hay pelota, volver al centro de la portería
        if ball is None:
            target = Vector2(
                (self.zone_min_x + self.zone_max_x) / 2,
                (self.zone_min_y + self.zone_max_y) / 2
            )
        else:
            # Calcular velocidad de la pelota
            ball_vel, self.last_time = self.controller.ball_velocity(ball, self.last_ball_pos, self.last_time)
            self.last_ball_pos = Vector2(ball.position.x, ball.position.y)

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

        return self.controller.goto_point_goalie(robot.pose, target, self.last_ball_pos)
    #c