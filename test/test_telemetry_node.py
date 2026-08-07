"""Tests for the telemetry collector node mapping (feature F17, task TF17-T05).

Only the pure dataclass->Telemetry mapping is unit-tested. Hardware reads and the
rclpy timer are exercised manually (see feature How to Demo). Following the repo
convention (conftest fakes ROS), a fake dome_control.msg.Telemetry is injected so
the test runs without a built/sourced workspace.
"""
import sys
from types import ModuleType, SimpleNamespace


def install_fake_telemetry_msg():
    if "dome_control.msg" in sys.modules:
        return

    class FakeHeader:
        def __init__(self):
            self.stamp = None

    class Telemetry:
        # Flat fields default to 0, matching the .msg defaults.
        float_fields = (
            "ups_bus_voltage_v", "ups_current_a", "ups_power_w", "ups_percent",
            "oak_fps", "oak_chip_temp_c", "oak_cmx_mem_used_mb",
            "oak_ddr_mem_used_mb", "oak_leon_css_cpu_pct", "oak_leon_mss_cpu_pct",
            "oak_pipeline_get_ms", "oak_tracker_ms", "oak_iter_ms",
            "pi_cpu_temp_c", "pi_cpu_pct", "pi_mem_used_mb", "pi_uptime_s",
        )

        def __init__(self):
            self.header = FakeHeader()
            self.oak_usb_speed = 0
            for field in self.float_fields:
                setattr(self, field, 0.0)

    fake = ModuleType("dome_control.msg")
    fake.Telemetry = Telemetry
    sys.modules["dome_control.msg"] = fake


install_fake_telemetry_msg()

from dome_control.nodes.telemetry_node import build_message  # noqa: E402


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=42, nanosec=7))


def make_ups():
    # Stands in for a dome_telemetry_msgs/BatteryStats received on /telemetry/battery.
    return SimpleNamespace(bus_voltage_v=12.0, current_a=0.5, power_w=6.0, percent=83.3)


def make_oak():
    # Stands in for a dome_telemetry_msgs/OakStats received on /telemetry/oak.
    return SimpleNamespace(
        fps=9.5, usb_speed=3.0, chip_temp_c=57.5, leon_css_cpu_pct=42.0,
        leon_mss_cpu_pct=13.0, cmx_mem_used_mb=2.0, ddr_mem_used_mb=128.0,
        pipeline_get_ms=1.2, tracker_ms=0.8, iter_ms=11.0,
    )


def make_host():
    return SimpleNamespace(
        cpu_temp_c=47.3, cpu_pct=12.5, mem_used_mb=2976.0, uptime_s=7200.0
    )


def test_maps_all_sources():
    msg = build_message(make_ups(), make_oak(), make_host(), FakeClock())
    assert msg.ups_bus_voltage_v == 12.0
    assert msg.ups_current_a == 0.5
    assert msg.ups_power_w == 6.0
    assert abs(msg.ups_percent - 83.3) < 1e-4
    assert msg.oak_fps == 9.5
    assert msg.oak_chip_temp_c == 57.5
    assert msg.oak_usb_speed == 3            # float 3.0 -> int 3
    assert msg.oak_cmx_mem_used_mb == 2.0
    assert msg.oak_ddr_mem_used_mb == 128.0
    assert abs(msg.oak_leon_css_cpu_pct - 42.0) < 1e-4
    assert abs(msg.oak_leon_mss_cpu_pct - 13.0) < 1e-4
    assert msg.oak_iter_ms == 11.0
    assert msg.pi_cpu_temp_c == 47.3
    assert msg.pi_cpu_pct == 12.5
    assert msg.pi_mem_used_mb == 2976.0
    assert msg.pi_uptime_s == 7200.0
    assert msg.header.stamp.sec == 42


def test_ups_none_leaves_ups_zero():
    msg = build_message(None, make_oak(), make_host(), FakeClock())
    assert msg.ups_bus_voltage_v == 0.0
    assert msg.ups_percent == 0.0
    assert msg.oak_chip_temp_c == 57.5      # oak still filled


def test_oak_none_leaves_oak_zero():
    msg = build_message(make_ups(), None, make_host(), FakeClock())
    assert msg.oak_chip_temp_c == 0.0
    assert msg.oak_usb_speed == 0
    assert msg.ups_bus_voltage_v == 12.0    # ups still filled
    assert msg.pi_cpu_temp_c == 47.3        # host still filled


def test_host_none_leaves_host_zero():
    msg = build_message(make_ups(), make_oak(), None, FakeClock())
    assert msg.pi_cpu_temp_c == 0.0
    assert msg.pi_cpu_pct == 0.0
    assert msg.pi_mem_used_mb == 0.0
    assert msg.oak_fps == 9.5               # oak still filled


def test_all_none_all_zero():
    msg = build_message(None, None, None, FakeClock())
    assert msg.ups_bus_voltage_v == 0.0
    assert msg.oak_chip_temp_c == 0.0
    assert msg.oak_fps == 0.0
    assert msg.pi_cpu_temp_c == 0.0
