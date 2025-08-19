from .types import Pose, Vector2
from .math_utils import normalize_angle
import math, time

class DifferentialController:
    """
    Calcula velocidades de roda para um robô com tração diferencial.
    """
    def __init__(
        self,
        k_forward: float = 50.0,
        k_turn: float = 10.0,
        stop_distance: float = -10.0,
        angle_threshold: float = 0.1,
        angle_threshold2: float = 0.2,
    ):
        self.k_forward = k_forward
        self.k_turn = k_turn
        self.stop_distance = stop_distance
        self.angle_threshold = angle_threshold
        self.angle_threshold2 = angle_threshold2

    def goto_point(self, robot: Pose, target: Vector2) -> tuple[float, float]:
        dx, dy = target.x - robot.position.x, target.y - robot.position.y
        distance = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - robot.theta)

        if distance > self.stop_distance:
            
            fwd = 0.0 if abs(angle_err) > self.angle_threshold else self.k_forward * distance
            turn = self.k_turn * angle_err

        return fwd + 10 - turn, fwd + 10 + turn  # (left_speed, right_speed)
    
    def constrain_target(self, target: Vector2) -> Vector2:
        forbidden_zones = [
            ##((0.45, 0.44), (0.65, -0.44)),
            ((-0.65, 0.44), (-0.55, -0.44))
        ]

        def is_inside_zone(pos: Vector2, x1, y1, x2, y2):
            return min(x1, x2) <= pos.x <= max(x1, x2) and min(y1, y2) <= pos.y <= max(y1, y2)

        for (x1, y1), (x2, y2) in forbidden_zones:
            if is_inside_zone(target, x1, y1, x2, y2):
                constrained_x = (
                    max(min(target.x, min(x1, x2)), max(x1, x2))
                    if x1 < x2
                    else min(max(target.x, max(x1, x2)), min(x1, x2))
                )
                return Vector2(constrained_x, target.y)
            target.y = max(min(target.y, 0.574), -0.574)
            
            target.x = max(min(target.x, 0.66), -0.65)

        return target


    def goto_point_lim(self, robot: Pose, target: Vector2) -> tuple[float, float]:
        target = self.constrain_target(target)

        dx, dy = target.x - robot.position.x, target.y - robot.position.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - robot.theta)

        if distance > self.stop_distance:
            fwd = 0.0 if abs(angle_err) > self.angle_threshold2 else self.k_forward * distance
            turn = self.k_turn * angle_err 
            return (fwd + 10 - turn), (fwd + 10 + turn) 

        return 0.0, 0.0



    def goto_point_HS(self, robot: Pose, target: Vector2) -> tuple[float, float]:
        dx, dy = target.x - robot.position.x, target.y - robot.position.y
        distance = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - robot.theta)

        if distance > self.stop_distance:
            
            fwd = 0.0 if abs(angle_err) > self.angle_threshold else self.k_forward * distance * 5
            turn = self.k_turn * angle_err

        return (fwd + 10 - turn), (fwd + 10 + turn)  # (left_speed, right_speed)
    
    def ball_velocity(self, ball, last_ball_pos, last_time):
        current_time = time.time()
        ball_vel = Vector2(0, 0)
        if last_ball_pos is not None and last_time is not None:
            dt = current_time - last_time
            if dt > 0:
                ball_vel = Vector2(
                    (ball.position.x - last_ball_pos.x) / dt,
                    (ball.position.y - last_ball_pos.y) / dt
                    )
        return ball_vel, current_time

    def turn_180(self, turn_speed: float = 10.0) -> tuple[float, float]:
        # Rueda izquierda hacia adelante, rueda derecha hacia atrás para girar en sitio
        left_speed = turn_speed
        right_speed = -turn_speed
        return left_speed, right_speed
    
    def goto_point_goalie(self, robot: Pose, target: Vector2) -> tuple[float, float]:
        # Queremos que solo se mueva en Y, no en X
        dx = target.x - robot.position.x
        dy = target.y - robot.position.y

        # Queremos que el robot se oriente para avanzar hacia target.y
        # Si la diferencia en Y es pequeña, para
        y_threshold = 0.02
        if abs(dy) < y_threshold:
            return 0.0, 0.0

        # Definimos hacia dónde debería estar mirando el robot para avanzar hacia target en Y
        # Si dy > 0, debería mirar 90 grados (pi/2), si dy < 0, -90 grados (-pi/2)
        desired_theta = math.pi / 2 if dy > 0 else -math.pi / 2

        angle_err = normalize_angle(desired_theta - robot.theta)
        
        # Inversamente proporcional a dx
        min_dx = 0.01
        inv_dx = 1 / (max(abs(dx), min_dx)*2)
        velocity_y = max(abs(dy), 0.2)

        speed = self.k_forward * velocity_y * inv_dx

        # Si está mirando en la dirección contraria (ángulo cercano a +-pi), gira 180
        if abs(angle_err) > 1.5:  # casi pi rad
            if abs(dy) < 0.4 or abs(dx) < 0.75:
                return -speed, -speed
            

        # Si la orientación está dentro de un umbral pequeño, avanza hacia Y
        if abs(angle_err) < 0.1:
            return speed, speed

        # Si no está bien orientado pero no está para girar 180, corrige girando suavemente
        turn = self.k_turn * angle_err
        return (5 - turn, 5 + turn)
