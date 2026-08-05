"""
conftest.py — shared test fixtures and fake ROS2 module injection.

Injects minimal fake ROS2/message modules so tests can import control
modules without a ROS2 installation. Must run before any test module
imports that trigger rclpy/geometry_msgs/nav_msgs/std_msgs/std_srvs.
"""
import sys
from types import ModuleType
from unittest.mock import Mock


def _inject_fake_ros2():
    if "rclpy" in sys.modules:
        return

    fake_node_class = type("Node", (), {
        "__init__": lambda self, name: None,
        "create_subscription": lambda *a, **kw: Mock(),
        "destroy_subscription": lambda *a, **kw: None,
        "create_publisher": lambda *a, **kw: Mock(),
        "create_client": lambda *a, **kw: Mock(),
        "get_logger": lambda self: Mock(),
    })

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.init = Mock()
    fake_rclpy.spin = Mock()
    fake_rclpy.spin_once = Mock()
    fake_rclpy.shutdown = Mock()
    fake_rclpy.ok = Mock(return_value=True)

    fake_node_mod = ModuleType("rclpy.node")
    fake_node_mod.Node = fake_node_class

    fake_qos_mod = ModuleType("rclpy.qos")
    fake_qos_mod.QoSProfile = Mock()
    fake_qos_mod.QoSDurabilityPolicy = Mock(TRANSIENT_LOCAL=Mock())

    fake_trigger = Mock()
    fake_trigger.Request.return_value = Mock()
    fake_std_srvs = ModuleType("std_srvs")
    fake_std_srvs_srv = ModuleType("std_srvs.srv")
    fake_std_srvs_srv.Trigger = fake_trigger

    fake_std_msgs = ModuleType("std_msgs")
    fake_std_msgs_msg = ModuleType("std_msgs.msg")
    fake_std_msgs_msg.String = Mock()

    fake_geometry_msgs = ModuleType("geometry_msgs")
    fake_geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    fake_geometry_msgs_msg.Twist = Mock()

    fake_nav_msgs = ModuleType("nav_msgs")
    fake_nav_msgs_msg = ModuleType("nav_msgs.msg")
    fake_nav_msgs_msg.Odometry = Mock()

    sys.modules.setdefault("rclpy", fake_rclpy)
    sys.modules.setdefault("rclpy.node", fake_node_mod)
    sys.modules.setdefault("rclpy.qos", fake_qos_mod)
    sys.modules.setdefault("std_srvs", fake_std_srvs)
    sys.modules.setdefault("std_srvs.srv", fake_std_srvs_srv)
    sys.modules.setdefault("std_msgs", fake_std_msgs)
    sys.modules.setdefault("std_msgs.msg", fake_std_msgs_msg)
    sys.modules.setdefault("geometry_msgs", fake_geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", fake_geometry_msgs_msg)
    sys.modules.setdefault("nav_msgs", fake_nav_msgs)
    sys.modules.setdefault("nav_msgs.msg", fake_nav_msgs_msg)


_inject_fake_ros2()
