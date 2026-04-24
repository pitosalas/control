#!/usr/bin/env python3
# intent_api.py — publishes normalized intent messages to /intent topic
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json

from std_msgs.msg import String

import control.commands.config_manager as cm
import control.ros2_api.base_api as base


class IntentApi(base.BaseApi):

    def __init__(self, config_manager: cm.ConfigManager = None):
        super().__init__("intent_api", config_manager)
        self.intent_pub = self.create_publisher(String, "/intent", 10)

    def publish(self, name: str, source: str, slots: dict) -> None:
        msg = String()
        msg.data = json.dumps({"name": name, "source": source, "slots": slots})
        self.intent_pub.publish(msg)
        self.log_info(f"Intent published: {msg.data}")
