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

from behaviors.gotoball import GotoBall
from behaviors.goalie import Goalie

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


def main(team_color_yellow: bool = False, attack_id: int = 0, goalie_id: int = 2, middle_id:int = 1):
    field_data = FieldData()
    vision_thread = ProtoVisionThread(team_color_yellow, field_data)
    vision_thread.start()

    manager = BehaviorManager()

    manager.set_behavior(middle_id, GotoBall())
    manager.set_behavior(attack_id, GotoBall())
    manager.set_behavior(
        goalie_id,
        Goalie(
            goalie_orientation=math.pi/2,
            k_turn=42.0,
            k_move=80.0,
            angle_threshold=0.02,
            stop_tolerance=0.01,
            zone_min_x=0.5,
            zone_max_x=0.7,
            zone_min_y=-0.3,
            zone_max_y=0.3,
        )
    )

    team_cmd = TeamCommand()
    control = ProtoControl(team_color_yellow, team_cmd, control_port=20011)

    try:
        while True:
            robots = to_framework_robots(field_data)
            ball = to_framework_ball(field_data)

            wheel_cmds = manager.step_all(robots, ball)
            for rid, (ls, rs, kick) in wheel_cmds.items():
                team_cmd.commands[rid].left_speed = ls
                team_cmd.commands[rid].right_speed = rs
                team_cmd.commands[rid].kick = kick

            control.update()
            time.sleep(0.02)
    except KeyboardInterrupt:
        logging.info("Saindo…")


if __name__ == "__main__":
    main()
