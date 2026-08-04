#!/usr/bin/env python3
# telemetry_node — ROS2 node: collect combined robot telemetry, publish /telemetry
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
"""Feature F17 collector node.

Reads host (/proc, /sys) stats directly, and UPS + OAK stats by subscribing to
the standalone dome_telemetry package's /telemetry/battery topic and the vision
stack's /telemetry/oak topic, merges them onto one flat dome_control/msg/Telemetry
message, and publishes /telemetry on a timer at the configured rate
(config/telemetry.yaml, publish_rate_hz default 1). A second timer appends each
published message to a daily CSV (log_interval_s default 10).

Subscribing (rather than opening the INA219/OAK directly) lets telemetry share
each hardware device with its dedicated owner: dome_telemetry owns the INA219,
the vision stack owns the OAK — a device has one owner (F20).

Sources are optional / fail-soft: if no BatteryStats/OakStats has arrived yet
(dome_telemetry/vision down), those fields stay zero and the node keeps
publishing.
"""
import os
from datetime import datetime

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from dome_control.msg import Telemetry
from dome_control.telemetry.config import load_telemetry_config
from dome_control.telemetry.csv_logger import TelemetryCsvLogger
from dome_control.telemetry.host_stats import HostStatsReader


def default_config_path():
    return os.path.join(
        get_package_share_directory("dome_control"), "config", "telemetry.yaml"
    )


def build_message(ups, oak, host, clock):
    """Map latest BatteryStats + OakStats msgs + HostStats (any may be None) onto a
    stamped Telemetry msg. `ups` is a dome_telemetry_msgs/BatteryStats as received
    on /telemetry/battery; `oak` is a dome_telemetry_msgs/OakStats as received on
    /telemetry/oak."""
    msg = Telemetry()
    msg.header.stamp = clock.now().to_msg()
    if ups is not None:
        msg.ups_bus_voltage_v = float(ups.bus_voltage_v)
        msg.ups_current_a = float(ups.current_a)
        msg.ups_power_w = float(ups.power_w)
        msg.ups_percent = float(ups.percent)
    if oak is not None:
        msg.oak_fps = float(oak.fps)
        msg.oak_chip_temp_c = float(oak.chip_temp_c)
        msg.oak_usb_speed = int(oak.usb_speed)
        msg.oak_cmx_mem_used_mb = float(oak.cmx_mem_used_mb)
        msg.oak_ddr_mem_used_mb = float(oak.ddr_mem_used_mb)
        msg.oak_leon_css_cpu_pct = float(oak.leon_css_cpu_pct)
        msg.oak_leon_mss_cpu_pct = float(oak.leon_mss_cpu_pct)
        msg.oak_pipeline_get_ms = float(oak.pipeline_get_ms)
        msg.oak_tracker_ms = float(oak.tracker_ms)
        msg.oak_iter_ms = float(oak.iter_ms)
    if host is not None:
        msg.pi_cpu_temp_c = float(host.cpu_temp_c)
        msg.pi_cpu_pct = float(host.cpu_pct)
        msg.pi_mem_used_mb = float(host.mem_used_mb)
        msg.pi_uptime_s = float(host.uptime_s)
    return msg


class TelemetryNode(Node):
    """Collect UPS + OAK + host stats and publish /telemetry at publish_rate_hz."""

    def __init__(self):
        super().__init__("telemetry")
        self.declare_parameter("config_path", default_config_path())
        config_path = self.get_parameter("config_path").value
        cfg = load_telemetry_config(config_path)
        rate_hz = cfg["publish_rate_hz"]
        log_interval_s = cfg["log_interval_s"]

        self.pub = self.create_publisher(Telemetry, "/telemetry", 10)
        self.host = HostStatsReader()
        self.csv = TelemetryCsvLogger(max_files=cfg["max_log_files"])
        self.latest_msg = None

        # UPS and OAK stats each come from their device's dedicated owner
        # (dome_telemetry, vision stack) so telemetry never opens hardware itself.
        self.latest_ups = None
        self.latest_oak = None
        from dome_telemetry_msgs.msg import BatteryStats, OakStats
        self.create_subscription(BatteryStats, "/telemetry/battery", self.on_ups, 10)
        self.create_subscription(OakStats, "/telemetry/oak", self.on_oak, 10)

        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.log_timer = self.create_timer(log_interval_s, self.log_tick)
        self.get_logger().info(
            f"TelemetryNode publishing /telemetry at {rate_hz} Hz "
            f"(UPS via /telemetry/battery, OAK via /telemetry/oak), "
            f"logging CSV every {log_interval_s}s"
        )

    def on_ups(self, msg):
        self.latest_ups = msg

    def on_oak(self, msg):
        self.latest_oak = msg

    def tick(self):
        msg = build_message(
            self.latest_ups, self.latest_oak, self.host.read(), self.get_clock()
        )
        self.latest_msg = msg
        self.pub.publish(msg)

    def log_tick(self):
        if self.latest_msg is None:
            return
        try:
            self.csv.log(self.latest_msg, datetime.now())
        except OSError as exc:
            self.get_logger().warn(f"telemetry CSV log failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
