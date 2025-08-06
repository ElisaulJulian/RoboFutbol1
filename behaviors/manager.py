from __future__ import annotations
from typing import Dict
from .behavior import Behavior
from core.types import Robot, Ball

class BehaviorManager:
    """
    Mantém um dicionário {robot_id: behavior}.
    """
    def __init__(self):
        self._behaviors: Dict[int, Behavior] = {}

    def set_behavior(self, robot_id: int, behavior: Behavior) -> None:
        self._behaviors[robot_id] = behavior
        #Asigna un comportamiento específico a un robot determinado.

    def step_all(self, robots: list[Robot], ball: Ball | None) -> dict[int, tuple[float, float]]:
        #Ejecuta un paso de simulación para todos los robots que tienen un comportamiento asignado.
        cmds: dict[int, tuple[float, float]] = {}
        #Diccionario donde se guardarán las velocidades de cada robot: {id: (vel_izq, vel_der)}.

        for r in robots:
            beh = self._behaviors.get(r.id)
            if beh:
                cmds[r.id] = beh.step(r, ball)
                #Busca el comportamiento asignado y si tiene, ejecuta step pasándole el robot y la pelota.
        return cmds
        #Es un diccionario que guarda las velocidades calculadas
