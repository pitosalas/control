#!/usr/bin/env python3
"""
Examples showing how different interfaces (CLI, TUI, REST) would use the CommandDispatcher.
This demonstrates the unified interface design.
"""

from control.command_dispatcher import CommandDispatcher
from control.robot_controller import RobotController
from control.config_manager import ConfigManager


def cli_interface_example():
    """Example of how a CLI would use the dispatcher."""
    print("=== CLI Interface Example ===")

    # Setup (would be done once)
    config = ConfigManager()
    robot = RobotController(config)
    dispatcher = CommandDispatcher(robot)

    # Simulate user typing: "move distance 1.5"
    result = dispatcher.execute("move.distance", {"distance": 1.5})
    print(f"CLI: {result.message}")

    # Simulate user typing: "robot speeds 0.5 0.3"
    result = dispatcher.execute("robot.speeds", {"linear": 0.5, "angular": 0.3})
    print(f"CLI: {result.message}")

    # Simulate user typing: "nav start --sim-time"
    result = dispatcher.execute("nav.start", {"use_sim_time": True})
    print(f"CLI: {result.message}")

    print()


def tui_interface_example():
    """Example of how a TUI (Text User Interface) would use the dispatcher."""
    print("=== TUI Interface Example ===")

    # Setup (would be done once)
    config = ConfigManager()
    robot = RobotController(config)
    dispatcher = CommandDispatcher(robot)

    # TUI would build menus dynamically from command registry
    groups = dispatcher.get_groups()
    print("TUI Menu Structure:")

    for group in groups:
        print(f"  [{group.upper()}]")
        commands = dispatcher.list_commands(group=group)
        for cmd in commands[:2]:  # Show first 2 commands per group
            info = dispatcher.get_command_info(cmd)
            print(f"    {cmd} - {info.description}")

    # User selects "movement" -> "move.distance" -> enters "2.0"
    result = dispatcher.execute("move.distance", {"distance": 2.0})
    print(f"TUI Result: {result.message}")

    print()


def rest_api_example():
    """Example of how a REST API would use the dispatcher."""
    print("=== REST API Interface Example ===")

    # Setup (would be done once)
    config = ConfigManager()
    robot = RobotController(config)
    dispatcher = CommandDispatcher(robot)

    # Simulate REST API endpoints using the dispatcher

    # POST /api/move/distance
    def api_move_distance(distance_value):
        return dispatcher.execute("move.distance", {"distance": distance_value})

    # POST /api/robot/speeds
    def api_set_speeds(linear_val, angular_val):
        return dispatcher.execute("robot.speeds", {"linear": linear_val, "angular": angular_val})

    # GET /api/robot/status
    def api_get_status():
        return dispatcher.execute("robot.status", {})

    # GET /api/commands (list available commands)
    def api_list_commands():
        return {
            "commands": dispatcher.list_commands(),
            "groups": dispatcher.get_groups()
        }

    # Simulate API calls
    result1 = api_move_distance(1.0)
    print(f"POST /api/move/distance: {result1.message}")

    result2 = api_set_speeds(0.4, 0.2)
    print(f"POST /api/robot/speeds: {result2.message}")

    result3 = api_get_status()
    print(f"GET /api/robot/status: {result3.message}")

    commands_info = api_list_commands()
    print(f"GET /api/commands: {len(commands_info['commands'])} commands available")

    print()


def unified_error_handling_example():
    """Example showing unified error handling across all interfaces."""
    print("=== Unified Error Handling Example ===")

    config = ConfigManager()
    robot = RobotController(config)
    dispatcher = CommandDispatcher(robot)

    # All interfaces handle errors the same way

    # Missing parameter
    result = dispatcher.execute("move.distance", {})
    print(f"Missing param: {result.success} - {result.message}")

    # Invalid command
    result = dispatcher.execute("invalid.command", {})
    print(f"Invalid command: {result.success} - {result.message}")

    # Wrong parameter type
    result = dispatcher.execute("move.distance", {"distance": "not_a_number"})
    print(f"Type error: {result.success} - {result.message}")

    print()


def help_system_example():
    """Example of unified help system."""
    print("=== Help System Example ===")

    config = ConfigManager()
    robot = RobotController(config)
    dispatcher = CommandDispatcher(robot)

    # General help
    print("General Help:")
    print(dispatcher.get_help())

    # Specific command help
    print("\nSpecific Command Help:")
    print(dispatcher.get_help("move.distance"))


if __name__ == "__main__":
    # Run all examples
    cli_interface_example()
    tui_interface_example()
    rest_api_example()
    unified_error_handling_example()
    help_system_example()