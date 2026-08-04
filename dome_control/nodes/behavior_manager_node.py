#!/usr/bin/env python3
# behavior_manager_node.py — ROS2 shell for behavior manager
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
"""ROS2 node that receives intents and routes to domain behavior handlers."""

import os
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from dome_control.announcement_contract import AnnouncementMsg, make_announcement_msg, PRIORITY_QUERY_REPLY
from dome_control.commands.intent_parser import IntentParser
from dome_control.behaviors.motion_behavior import MotionBehavior
from dome_control.behaviors.perception_behavior import PerceptionBehavior
from dome_control.commands.config_manager import ConfigManager
from dome_control.commands.robot_controller import RobotController

DEFAULT_CONFIG = os.path.expanduser("~/.dome/config/control.yaml")


def default_vision_config() -> str:
    env_path = os.environ.get("DOME_VISION_CONFIG")
    if env_path:
        return os.path.expanduser(env_path)
    try:
        return os.path.join(
            get_package_share_directory("dome_vision_ros"),
            "config",
            "roboflow_oak.yaml",
        )
    except PackageNotFoundError:
        return ""


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__("behavior_manager")
        self.parser = IntentParser()

        rc = RobotController(ConfigManager.create(DEFAULT_CONFIG))
        self.handlers = [
            MotionBehavior(rc),
            PerceptionBehavior(self),
        ]

        self.declare_parameter("class_profiles_path", default_vision_config())
        vision_config_path = self.get_parameter("class_profiles_path").get_parameter_value().string_value
        self.profiles: dict = {}
        self.label_map: dict = {}
        try:
            from dome_vision.class_profiles import build_label_map, load_class_profiles
            self.profiles = load_class_profiles(vision_config_path)
            self.label_map = build_label_map(self.profiles)
        except Exception as exc:
            self.get_logger().warn(f"Could not load class profiles from {vision_config_path}: {exc}")

        self.intent_sub = self.create_subscription(
            String, "/intent", self.on_intent, 10
        )
        self.detections_sub = self.create_subscription(
            Detection2DArray, "/oak/detections", self.on_detections, 10
        )
        self.latest_detections: Detection2DArray | None = None
        self.announcement_pub = self.create_publisher(
            AnnouncementMsg, "/announcement", 10
        )
        self.describe_scene_client = self.create_client(Trigger, "/describe_scene")

    def on_detections(self, msg: Detection2DArray) -> None:
        self.latest_detections = msg

    def on_intent(self, msg: String) -> None:
        try:
            intent = self.parser.parse_intent(msg.data)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        if intent.name == "get_help":
            msg = make_announcement_msg(
                "commands are stop, explore, describe, right, left, status and help",
                priority=PRIORITY_QUERY_REPLY,
                source="behavior_manager",
            )
            self.announcement_pub.publish(msg)
            return

        for handler in self.handlers:
            if handler.handles(intent.name):
                handler.execute(intent, self)
                return

        self.get_logger().warn(f"Unhandled intent: {intent.name}")


def main():
    rclpy.init()
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
