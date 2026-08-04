#!/usr/bin/env python3
# test_motion_behavior.py — tests for MotionBehavior
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
from unittest.mock import Mock

from dome_control.commands.intent_parser import Intent
from dome_control.behaviors.motion_behavior import MOTION_INTENTS, MotionBehavior


def make_intent(name, slots=None):
    return Intent(name=name, source="test", slots=slots or {})


class TestMotionBehaviorHandles:

    def test_handles_stop(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("stop") is True

    def test_does_not_handle_explore(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("explore") is False

    def test_explore_removed_from_motion_intents(self):
        assert "explore" not in MOTION_INTENTS

    def test_handles_drive_square(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("drive_square") is True

    def test_handles_turn_right(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("turn_right") is True

    def test_handles_turn_left(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("turn_left") is True

    def test_handles_get_status(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("get_status") is True

    def test_does_not_handle_describe_scene(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("describe_scene") is False

    def test_does_not_handle_unknown(self):
        mb = MotionBehavior(Mock())
        assert mb.handles("fly_to_moon") is False


class TestMotionBehaviorExecute:

    def test_stop_calls_stop_robot(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("stop"))
        rc.stop_robot.assert_called_once_with()

    def test_drive_square_calls_script_square_with_meters(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("drive_square", {"meters": "2.5"}))
        rc.script_square.assert_called_once_with(2.5)

    def test_drive_square_defaults_to_1_meter(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("drive_square"))
        rc.script_square.assert_called_once_with(1.0)

    def test_turn_right_calls_turn_clockwise(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("turn_right"))
        rc.turn_clockwise.assert_called_once_with(90)

    def test_turn_left_calls_turn_counterclockwise(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("turn_left"))
        rc.turn_counterclockwise.assert_called_once_with(90)

    def test_get_status_publishes_announcement(self):
        rc = Mock()
        rc.get_robot_status.return_value = Mock(
            data={"status": {"speeds": {"linear": 0.3, "angular": 0.5}}}
        )
        node = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("get_status"), node)
        node.announcement_pub.publish.assert_called_once()

    def test_get_status_without_node_does_not_raise(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("get_status"))  # node=None, must not crash

    def test_unknown_intent_does_nothing(self):
        rc = Mock()
        mb = MotionBehavior(rc)
        mb.execute(make_intent("fly_to_moon"))
        rc.stop_robot.assert_not_called()
        rc.script_square.assert_not_called()
