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
        dx = target.x - robot.position.x
        dy = target.y - robot.position.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - robot.theta)
        

        if abs(angle_err) > 1.5:
            turn = self.k_turn * angle_err
            return -25,-25
            

        # Si está muy cerca, ignora el giro y avanza fuerte para dar el tiro
        if distance < 0.07:
            fwd = self.k_forward * 0.7  # velocidad fuerte para tiro
            turn = 0.0                  # no gira más, solo avanza
        else:
            # Permite avanzar embora o ângulo não seja perfeito, mas reduz a velocidade se estiver muito desalinhado
            angle_factor = max(0.2, 1 - abs(angle_err) / math.pi)  # Entre 0.2 e 1 segundo o erro angular
            fwd = self.k_forward * distance * angle_factor

            # Garante uma velocidade mínima se estiver longe
            if distance > 0.05 and fwd < 10:
                fwd = 20

            # Controle de giro suave
            turn = self.k_turn * angle_err

        # Limita as velocidades máximas
        max_speed = 100
        left = max(-max_speed, min(max_speed, fwd - turn))
        right = max(-max_speed, min(max_speed, fwd + turn))

        return left, right



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
    
    def goto_point_goalie(self, robot: Pose, target: Vector2, ball_pos) -> tuple[float, float]:
        # Queremos que solo se mueva en Y, no en X 
        dx = target.x - robot.position.x
        dy = target.y - robot.position.y

        # Queremos que el robot se oriente para avanzar hacia target.y
        # Si la diferencia en Y es pequeña, para
        y_threshold = 0.02
        if abs(dy) < y_threshold and abs(dx) < 0.4:
            return 0.0, 0.0

        # Definimos hacia dónde debería estar mirando el robot para avanzar hacia target en Y
        # Si dy > 0, debería mirar 90 grados (pi/2), si dy < 0, -90 grados (-pi/2)
        desired_theta = math.pi / 2 if dy > 0 else -math.pi / 2

        angle_err = normalize_angle(desired_theta - robot.theta)

        #robot arriba ,pelota abajo -180
        #robot abajo pelota arriba 180 
        
        # Inversamente proporcional a dx
        min_dx = 0.01
        inv_dx = 1 / (max(abs(dx), min_dx)*2)
        velocity_y = max(abs(dy), 0.2)

        speed = self.k_forward * velocity_y * inv_dx
        speed = abs(speed)

        # Si está mirando en la dirección contraria (ángulo cercano a +-pi), gira 180
        if abs(angle_err) > math.pi / 2:  # casi pi rad
            if ball_pos is None:
                return 0,0
            if ball_pos.x > 0:
                return -speed, -speed
            return self.turn_180(turn_speed=30.0)

        # Si la orientación está dentro de un umbral pequeño, avanza hacia Y
        if abs(angle_err) < 0.1:
            return speed, speed

        # Si no está bien orientado pero no está para girar 180, corrige girando suavemente
        turn = self.k_turn * angle_err
        return (1 - turn, 1 + turn)

    def reorientate(self, robot, angle_range:tuple[float, float]):
        """
        Reorienta al robot hasta que su ángulo esté dentro de un rango dado. 
        angle_range: (min_angle, max_angle) en radianes
        """
        if not isinstance(robot, Pose):
            raise TypeError("Robot pasado no es objeto tipo Pose")
        
        min_angle, max_angle = angle_range
        theta = normalize_angle(robot.theta)

        target_angle = (min_angle + max_angle)/2
        angle_err = normalize_angle(target_angle - theta)

        turn = self.k_turn*angle_err
        return (-turn, turn)
    
    def ball_distance(self, robot_pose, ball):
        dx = ball.x - robot_pose.position.x
        dy = ball.y - robot_pose.position.y
        return math.hypot(dx, dy)
