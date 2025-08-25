from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball, Vector2
import math, random



class Goleador(Behavior):
    """
    Goleador: se posiciona detrás de la pelota y empuja hacia el arco.
    """

    def __init__(self, controller: DifferentialController | None = None):
        self.controller = controller or DifferentialController()
        self.offset = 0.062  # Distancia detrás de la pelota
        self.angle_tolerance = 0.18  # radianes
        self.pos_tolerance = 0.01  # metros
    
    def update_position(self, robot: Robot):
        self.last_shoot_pos = robot.pose.position  # se actualiza en cada paso

    def goaler_pos(self) -> Vector2:
        return self.last_shoot_pos

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        if ball is None:
            return 0.0, 0.0
        self.update_position(robot)
        # Posiciones clave

        goal_x = -1.1
        goal_y_min = -0.5
        goal_y_max = 0.5
        goal_y = random.uniform(goal_y_min, goal_y_max)  # Aleatorio
        goal_position = Vector2(goal_x, goal_y)
        ball_pos = ball.position
        robot_pos = robot.pose.position

        # Si el robot está más adelantado que la pelota, retroceder
        if robot_pos.x <= ball_pos.x:
            if robot_pos.y < ball_pos.y:
                retreat_target = Vector2(ball_pos.x + 0.4, ball_pos.y - 0.07)
            else:
                retreat_target = Vector2(ball_pos.x + 0.04, ball_pos.y + 0.07)
            return self.controller.goto_point_lim(robot.pose, retreat_target)

        # Vector de dirección pelota → arco
        dx = goal_position.x - ball_pos.x 
        dy = goal_position.y - ball_pos.y 
        norm = math.hypot(dx, dy)
        norm = norm * 1.0
        if norm == 0:
            return 0.0, 0.0
        vec_dir = Vector2(dx / norm, dy / norm)

        # Punto objetivo detrás de la pelota
        target = Vector2(
            ball_pos.x - vec_dir.x * self.offset,
            ball_pos.y - vec_dir.y * self.offset
        )

        # Orientación deseada hacia la pelota
        angle_to_ball = math.atan2(ball_pos.y - robot_pos.y, ball_pos.x - robot_pos.x)
        angle_error = abs(angle_to_ball - robot.pose.theta)

        # Distancia a punto objetivo
        dist_to_target = math.hypot(target.x - robot_pos.x, target.y - robot_pos.y)

        # Lógica de movimiento
        if dist_to_target <= self.pos_tolerance and angle_error < self.angle_tolerance:
            # Empujar la pelota hacia el arco
            push_target = Vector2(
                ball_pos.x + vec_dir.x * 1,
                ball_pos.y + vec_dir.y * 1
                )
            return self.controller.goto_point_lim(robot.pose, goal_position)
        else:
            # Posicionarse detrás de la pelota
            return self.controller.goto_point_lim(robot.pose, target)
        
    @staticmethod
    def ball_control(robot: Robot, ball: Ball | None, area_radius: float = 0.35) -> bool:
        if ball is None:
            return False
        robot_pos = robot.pose.position
        ball_pos = ball.position
        distance = math.hypot(ball_pos.x - robot_pos.x, ball_pos.y - robot_pos.y)
        return distance <= area_radius
    
    @staticmethod
    def goler_pos(robot: Robot) -> Vector2:
        return robot.pose.position