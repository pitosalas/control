#!/usr/bin/env python3
from abc import ABC

import rclpy
from rclpy.node import Node

from ..commands.config_manager import ConfigManager


class BaseApi(Node, ABC):
    """
    Base class for all ROS2 API nodes providing shared functionality.
    Handles ROS2 initialization, config management, and common utilities.
    """

    def __init__(self, node_name: str, config_manager: ConfigManager = None):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(node_name)

        self.config = config_manager or ConfigManager()

    def get_topics(self):
        return self.get_topic_names_and_types()

    def log_debug(self, message: str):
        self.get_logger().debug(message)

    def log_info(self, message: str):
        print(message)

    def log_warn(self, message: str):
        self.get_logger().warn(message)

    def log_error(self, message: str):
        self.get_logger().error(message)

    def check_bounds(
        self, value: float, min_val: float, max_val: float, name: str
    ) -> bool:
        if not (min_val <= value <= max_val):
            self.log_warn(
                f"{name} out of bounds: {value}. Must be [{min_val}, {max_val}]"
            )
            return False
        return True

    def destroy_node(self):
        self.log_info(f"Shutting down {self.get_name()} node")
        super().destroy_node()
