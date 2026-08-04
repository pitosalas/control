#!/usr/bin/env python3
# motion_behavior.py — motion intent handler
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
"""Handles motion-domain intents. No ROS2 dependency."""

from dome_control.commands.intent_parser import Intent
from dome_control.announcement_contract import make_announcement_msg, PRIORITY_QUERY_REPLY

MOTION_INTENTS = {"stop", "drive_square", "turn_right", "turn_left", "get_status"}

TURN_DEGREES = 90


class MotionBehavior:
    """Executes motion intents via RobotController."""

    def __init__(self, robot_controller):
        self.rc = robot_controller

    def handles(self, intent_name: str) -> bool:
        return intent_name in MOTION_INTENTS

    def execute(self, intent: Intent, node=None) -> None:
        if intent.name == "stop":
            self.rc.stop_robot()
        elif intent.name == "drive_square":
            meters = float(intent.slots.get("meters", 1.0))
            self.rc.script_square(meters)
        elif intent.name == "turn_right":
            self.rc.turn_clockwise(TURN_DEGREES)
        elif intent.name == "turn_left":
            self.rc.turn_counterclockwise(TURN_DEGREES)
        elif intent.name == "get_status" and node is not None:
            result = self.rc.get_robot_status()
            status = result.data.get("status", {}) if result.data else {}
            speeds = status.get("speeds", {})
            text = (
                f"linear speed {speeds.get('linear', 'unknown')}, "
                f"angular speed {speeds.get('angular', 'unknown')}"
            )
            msg = make_announcement_msg(text, priority=PRIORITY_QUERY_REPLY, source="motion_behavior")
            node.announcement_pub.publish(msg)
