"""
Exemplo de strategy: decidir quando chutar.
Futura expansão: avaliar linha de chute, força, etc.
"""
from core.types import Robot, Ball, Pose, Vector2

class SimpleKickWhenClose:
    def __init__(self, distance_threshold: float = 0.12):
        self.distance_threshold = distance_threshold

    def should_kick(self, robot: Robot, ball: Ball | None) -> bool:
        if ball is None:
            return False
        dx = ball.position.x - robot.pose.position.x
        dy = ball.position.y - robot.pose.position.y
        return (dx**2 + dy**2) ** 0.5 < self.distance_threshold
