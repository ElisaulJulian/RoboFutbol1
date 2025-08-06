from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball
from strategies.kick_strategies import SimpleKickWhenClose

class GotoBall(Behavior):
    """
    Este lleva el robot hacia la pelota hasta cierta distancia de parada
    """

    def __init__(self, controller: DifferentialController | None = None, kick_strategy=None):
        self.controller = controller or DifferentialController()
        self.kick_strategy = kick_strategy or SimpleKickWhenClose()
        #significa: "si controller es None, usá un nuevo DifferentialController() como valor por defecto."

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        if ball is None:
            return 0.0, 0.0 #queda quieto pue
        left_speed, right_speed = self.controller.goto_point(robot.pose, ball.position)
        should_kick = self.kick_strategy.should_kick(robot, ball)
        return left_speed, right_speed, should_kick
    
    #Usar el controlador diferencial para calcular las velocidades de rueda necesarias para que el robot se 
    #acerque a la posición de la pelota.
    #El controlador va a decidir si avanzar, girar, o detenerse según la posición y orientación 
    #actual del robot y la pelota.

