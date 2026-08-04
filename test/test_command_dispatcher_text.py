#!/usr/bin/env python3
# test_command_dispatcher_text.py — tests for CommandDispatcher.dispatch_text
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
# Test tier: No-ROS
from unittest.mock import Mock

import pytest

from dome_control.commands.command_dispatcher import CommandDispatcher
from dome_control.commands.robot_controller import CommandResponse


@pytest.fixture
def rc():
    mock = Mock()
    mock.move_forward.return_value = CommandResponse(True, "Moved forward")
    mock.move_backward.return_value = CommandResponse(True, "Moved backward")
    mock.turn_degrees.return_value = CommandResponse(True, "Turned")
    mock.stop_robot.return_value = CommandResponse(True, "Stopped")
    mock.list_maps.return_value = CommandResponse(True, "Maps listed")
    mock.get_all_variables.return_value = CommandResponse(True, "Variables listed")
    mock.launch_list.return_value = CommandResponse(True, "Launches listed")
    return mock


@pytest.fixture
def dispatcher(rc):
    return CommandDispatcher(rc)


class TestDispatchTextDirect:

    def test_move_forward_with_meters(self, dispatcher, rc):
        result = dispatcher.dispatch_text("move forward 1.0")
        assert result.success is True
        rc.move_forward.assert_called_once_with(meters=1.0)

    def test_move_backward_with_meters(self, dispatcher, rc):
        result = dispatcher.dispatch_text("move backward 0.5")
        assert result.success is True
        rc.move_backward.assert_called_once_with(meters=0.5)

    def test_map_list(self, dispatcher, rc):
        result = dispatcher.dispatch_text("map list")
        assert result.success is True
        rc.list_maps.assert_called_once()

    def test_config_list(self, dispatcher, rc):
        result = dispatcher.dispatch_text("config list")
        assert result.success is True
        rc.get_all_variables.assert_called_once()

    def test_launch_list(self, dispatcher, rc):
        result = dispatcher.dispatch_text("launch list")
        assert result.success is True
        rc.launch_list.assert_called_once()


class TestDispatchTextAbbreviations:

    def test_abbreviation_m_fwd(self, dispatcher, rc):
        result = dispatcher.dispatch_text("m fwd 2.0")
        assert result.success is True
        rc.move_forward.assert_called_once_with(meters=2.0)

    def test_abbreviation_map_sav(self, dispatcher, rc):
        mock_save = Mock(return_value=CommandResponse(True, "Saved"))
        rc.map_save = mock_save
        result = dispatcher.dispatch_text("map sav")
        assert result.success is True


class TestDispatchTextErrors:

    def test_unknown_command_returns_failure(self, dispatcher):
        result = dispatcher.dispatch_text("fly to moon")
        assert result.success is False
        assert "Unknown command" in result.message

    def test_empty_input_returns_failure(self, dispatcher):
        result = dispatcher.dispatch_text("")
        assert result.success is False

    def test_whitespace_only_returns_failure(self, dispatcher):
        result = dispatcher.dispatch_text("   ")
        assert result.success is False

    def test_move_distance_removed(self, dispatcher):
        result = dispatcher.dispatch_text("move distance 1")
        assert result.success is False
        assert "Unknown command" in result.message

    def test_turn_degrees_removed(self, dispatcher):
        result = dispatcher.dispatch_text("turn degrees 90")
        assert result.success is False
        assert "Unknown command" in result.message


class TestDispatchTextBehaviorIntents:

    def _dispatcher_with_mock_publisher(self, rc):
        published = []
        from dome_control.commands.intent_publisher import IntentPublisher
        pub = IntentPublisher(publish_fn=published.append)
        return CommandDispatcher(rc, intent_publisher=pub), published

    def test_scene_describe_publishes_intent(self, rc):
        dispatcher, published = self._dispatcher_with_mock_publisher(rc)
        result = dispatcher.dispatch_text("scene describe")
        assert result.success is True
        assert "describe_scene" in result.message
        assert len(published) == 1
        import json
        payload = json.loads(published[0])
        assert payload["name"] == "describe_scene"
        assert payload["source"] == "cli"

    def test_intent_stop_publishes_stop_intent(self, rc):
        dispatcher, published = self._dispatcher_with_mock_publisher(rc)
        result = dispatcher.dispatch_text("intent stop")
        assert result.success is True
        assert len(published) == 1
        import json
        assert json.loads(published[0])["name"] == "stop"

    def test_intent_explore_publishes_explore_intent(self, rc):
        dispatcher, published = self._dispatcher_with_mock_publisher(rc)
        result = dispatcher.dispatch_text("intent explore")
        assert result.success is True
        import json
        assert json.loads(published[0])["name"] == "explore"

    def test_scene_count_publishes_count_objects_intent(self, rc):
        dispatcher, published = self._dispatcher_with_mock_publisher(rc)
        result = dispatcher.dispatch_text("scene count")
        assert result.success is True
        import json
        assert json.loads(published[0])["name"] == "count_objects"

    def test_move_forward_does_not_publish_intent(self, rc):
        dispatcher, published = self._dispatcher_with_mock_publisher(rc)
        dispatcher.dispatch_text("move forward 1.0")
        assert len(published) == 0
        rc.move_forward.assert_called_once()

    def test_robot_stop_does_not_publish_intent(self, rc):
        rc.stop_robot.return_value = CommandResponse(True, "Stopped")
        dispatcher, published = self._dispatcher_with_mock_publisher(rc)
        dispatcher.dispatch_text("robot stop")
        assert len(published) == 0
        rc.stop_robot.assert_called_once()


class TestNavCommands:

    def test_nav_go_calls_navigation_go(self, dispatcher, rc):
        rc.publish_intent_navigation_go.return_value = CommandResponse(True, "Intent published: navigation_go")
        result = dispatcher.dispatch_text("nav go chair")
        assert result.success is True
        rc.publish_intent_navigation_go.assert_called_once_with(label="chair")

    def test_nav_cancel_calls_navigation_cancel(self, dispatcher, rc):
        rc.publish_intent_navigation_cancel.return_value = CommandResponse(True, "Intent published: navigation_cancel")
        result = dispatcher.dispatch_text("nav cancel")
        assert result.success is True
        rc.publish_intent_navigation_cancel.assert_called_once()

    def test_nav_explore_calls_exploration_start(self, dispatcher, rc):
        rc.publish_intent_exploration_start.return_value = CommandResponse(True, "Intent published: exploration_start")
        result = dispatcher.dispatch_text("nav explore")
        assert result.success is True
        rc.publish_intent_exploration_start.assert_called_once()

    def test_nav_explore_stop_calls_exploration_stop(self, dispatcher, rc):
        rc.publish_intent_exploration_stop.return_value = CommandResponse(True, "Intent published: exploration_stop")
        result = dispatcher.dispatch_text("nav explore stop")
        assert result.success is True
        rc.publish_intent_exploration_stop.assert_called_once()

    def test_nav_explore_status_calls_explore_status(self, dispatcher, rc):
        rc.explore_status.return_value = CommandResponse(True, "explore status: exploring")
        result = dispatcher.dispatch_text("nav explore status")
        assert result.success is True
        rc.explore_status.assert_called_once()

    def test_nav_explore_status_does_not_publish_intent(self, dispatcher, rc):
        rc.explore_status.return_value = CommandResponse(True, "explore status: idle")
        dispatcher.dispatch_text("nav explore status")
        rc.publish_intent_exploration_start.assert_not_called()


class TestDispatchTextValueParsing:

    def test_integer_arg_parsed(self, dispatcher, rc):
        rc.system_kill = Mock(return_value=CommandResponse(True, "Killed"))
        result = dispatcher.dispatch_text("system kill 1234")
        assert result.success is True
        rc.kill_ros_process.assert_called_once_with(pid=1234)

    def test_float_arg_parsed(self, dispatcher, rc):
        result = dispatcher.dispatch_text("move forward 1.5")
        assert result.success is True
        rc.move_forward.assert_called_once_with(meters=1.5)

    def test_negative_float_parsed(self, dispatcher, rc):
        result = dispatcher.dispatch_text("move forward -1.0")
        assert result.success is True
        rc.move_forward.assert_called_once_with(meters=-1.0)
