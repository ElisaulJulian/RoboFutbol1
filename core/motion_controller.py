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
    