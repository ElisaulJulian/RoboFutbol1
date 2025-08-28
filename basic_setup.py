"""
Loop mínimo que:
1. Lê visão pelo V3SProtoComm
2. Atualiza um BehaviorManager
3. Envia comandos de roda
"""
import time
import logging
import math

from V3SProtoComm.core.data import FieldData
from V3SProtoComm.core.comm.vision import ProtoVisionThread
from V3SProtoComm.core.comm.controls import ProtoControl
from V3SProtoComm.core.command import TeamCommand

from behaviors.manager import BehaviorManager
from core.types import Robot, Ball, Pose, Vector2
from utils.logger import configure as configure_logger

from behaviors.defensa import Defensa
from behaviors.goleador import Goleador
#from behaviors.medidas import Medidas
from behaviors.arquero import Arquero

configure_logger(logging.INFO)


def to_framework_robots(field_data) -> list[Robot]:
    robots: list[Robot] = []
    for idx, r in enumerate(field_data.robots):
        pose = Pose(Vector2(r.position.x, r.position.y), r.position.theta)
        robots.append(Robot(id=idx, pose=pose))
    return robots


def to_framework_ball(field_data) -> Ball | None:
    if field_data.ball is None:
        return None
    return Ball(position=Vector2(field_data.ball.position.x, field_data.ball.position.y))


def main(team_color_yellow: bool = False, robot0: int = 0, robot1: int = 1, robot2: int = 2):
    field_data = FieldData()
    vision_thread = ProtoVisionThread(team_color_yellow, field_data)
    vision_thread.start()

    manager = BehaviorManager()

    team_cmd = TeamCommand()
    control = ProtoControl(team_color_yellow, team_cmd, control_port=20011)

    try:
        while True:
            robots = to_framework_robots(field_data)
            ball = to_framework_ball(field_data)

            manager.set_behavior(robot1, Defensa())
            #manager.set_behavior(robot0, Goleador())
            #manager.set_behavior(robot2, Arquero())

            wheel_cmds = manager.step_all(robots, ball)
            for rid, (ls, rs) in wheel_cmds.items():
                team_cmd.commands[rid].left_speed = ls
                team_cmd.commands[rid].right_speed = rs

            control.update()
            time.sleep(0.02)
    except KeyboardInterrupt:
        logging.info("Saindo…")


if __name__ == "__main__":
    main()
