from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball, Vector2

class Defensa(Behavior):
    """
    Aproxime o robô até parar a `stop_distance` da bola.
    """

    def __init__(self, controller: DifferentialController | None = None):
        self.controller = controller or DifferentialController()

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        tarX = 0.4
        target = Vector2(tarX, ball.position.y)

        if ball is None:
            return 0.0, 0.0
        return self.controller.goto_point_lim(robot.pose, target)

