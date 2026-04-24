#!/usr/bin/env python3
import json
import pytest
from unittest.mock import MagicMock, patch
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.robot_controller import CommandResponse, RobotController


class TestIntentApi:

    def _make_intent_api(self):
        with patch("control.ros2_api.base_api.BaseApi.__init__", return_value=None):
            import control.ros2_api.intent_api as ia
            api = ia.IntentApi.__new__(ia.IntentApi)
            api.intent_pub = MagicMock()
            api.log_info = MagicMock()
            return api

    def test_publish_serializes_json(self):
        import control.ros2_api.intent_api as ia
        api = self._make_intent_api()
        ia.IntentApi.publish(api, "stop", "cli", {})

        api.intent_pub.publish.assert_called_once()
        msg = api.intent_pub.publish.call_args[0][0]
        data = json.loads(msg.data)
        assert data["name"] == "stop"
        assert data["source"] == "cli"
        assert data["slots"] == {}

    def test_publish_includes_slots(self):
        import control.ros2_api.intent_api as ia
        api = self._make_intent_api()
        ia.IntentApi.publish(api, "count_objects", "cli", {"object_type": "chair"})

        msg = api.intent_pub.publish.call_args[0][0]
        data = json.loads(msg.data)
        assert data["slots"] == {"object_type": "chair"}


class TestIntentCommands:

    @pytest.fixture
    def mock_rc(self):
        mock = MagicMock(spec=RobotController)
        mock.publish_intent_stop.return_value = CommandResponse(True, "Intent published: stop")
        mock.publish_intent_explore.return_value = CommandResponse(True, "Intent published: explore")
        mock.publish_intent_describe_scene.return_value = CommandResponse(True, "Intent published: describe_scene")
        mock.publish_intent_count_objects.return_value = CommandResponse(True, "Intent published: count_objects")
        return mock

    @pytest.fixture
    def dispatcher(self, mock_rc):
        return CommandDispatcher(mock_rc)

    def test_intent_stop_dispatches(self, dispatcher, mock_rc):
        result = dispatcher.execute("intent.stop", {})
        assert result.success is True
        mock_rc.publish_intent_stop.assert_called_once_with()

    def test_intent_explore_dispatches(self, dispatcher, mock_rc):
        result = dispatcher.execute("intent.explore", {})
        assert result.success is True
        mock_rc.publish_intent_explore.assert_called_once_with()

    def test_intent_describe_scene_dispatches(self, dispatcher, mock_rc):
        result = dispatcher.execute("intent.describe_scene", {})
        assert result.success is True
        mock_rc.publish_intent_describe_scene.assert_called_once_with()

    def test_intent_count_objects_dispatches(self, dispatcher, mock_rc):
        result = dispatcher.execute("intent.count_objects", {"object_type": "chair"})
        assert result.success is True
        mock_rc.publish_intent_count_objects.assert_called_once_with(object_type="chair")

    def test_intent_count_objects_missing_param(self, dispatcher):
        result = dispatcher.execute("intent.count_objects", {})
        assert result.success is False
        assert "object_type" in result.message

    def test_intent_commands_in_registry(self, dispatcher):
        cmds = dispatcher.list_commands(group="intent")
        assert "intent.stop" in cmds
        assert "intent.explore" in cmds
        assert "intent.describe_scene" in cmds
        assert "intent.count_objects" in cmds
