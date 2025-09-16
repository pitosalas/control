#!/usr/bin/env python3

import click
import io
from contextlib import redirect_stdout, redirect_stderr
from typing import Any
from dataclasses import dataclass
from .robot_controller import RobotController, CommandResponse
from .config_manager import ConfigManager

@dataclass
class CommandResult:
    success: bool
    message: str
    data: Any = None

class CommandProcessor:
    def __init__(self):
        self.config_manager = ConfigManager()
        self.robot_controller = RobotController(self.config_manager)
        self.cli = self._create_cli()

    def save_config(self):
        """Save configuration to file"""
        self.robot_controller.save_config()

    def _create_cli(self):
        @click.group(invoke_without_command=True)
        @click.pass_context
        def cli(ctx):
            """Robot control commands"""
            if ctx.invoked_subcommand is None:
                click.echo("Type a command. Use --help for available commands.")

        @cli.command()
        @click.argument('command', required=False)
        @click.pass_context
        def help(ctx, command):
            """Show help for all commands or a specific command"""
            if command:
                # Show help for specific command
                try:
                    subcommand = ctx.parent.command.get_command(ctx.parent, command)
                    if subcommand:
                        click.echo(subcommand.get_help(ctx.parent))
                    else:
                        click.echo(f"No such command '{command}'.")
                except Exception:
                    click.echo(f"No such command '{command}'.")
            else:
                # Print the main CLI help
                click.echo(ctx.parent.get_help())

        @cli.command()
        def exit():
            """Exit the program"""
            click.echo("Exiting...")
            # Will be handled by CLI interface checking for exit in result data
            raise click.ClickException("EXIT_SIGNAL")

        @cli.group()
        def move():
            """Move the robot"""
            pass

        @move.command(context_settings={'ignore_unknown_options': True, 'allow_extra_args': True})
        @click.argument('distance', type=float)
        def dist(distance):
            """Move robot a specific distance in meters"""
            result = self.robot_controller.move_distance(distance)
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @move.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Move robot at default speed for specified duration in seconds"""
            result = self.robot_controller.move_for_time(seconds)
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @cli.group()
        def turn():
            """Turn the robot"""
            pass

        @turn.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Turn robot at default speed for specified duration in seconds"""
            result = self.robot_controller.turn_for_time(seconds)
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @turn.command()
        @click.argument('angle', type=float)
        def radians(angle):
            """Turn robot by specific angle in radians"""
            result = self.robot_controller.turn_by_radians(angle)
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @turn.command()
        @click.argument('angle', type=float)
        def degrees(angle):
            """Turn robot by specific angle in degrees"""
            result = self.robot_controller.turn_by_degrees(angle)
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @cli.command()
        def stop():
            """Stop the robot immediately"""
            result = self.robot_controller.stop_robot()
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @cli.group()
        def nav():
            """Navigation commands"""
            pass

        @nav.group()
        def start():
            """Start navigation services"""
            pass

        @start.command()
        @click.option('--use-sim-time', is_flag=True, default=False, help='Use simulation time')
        def stack(use_sim_time):
            """Start navigation stack"""
            result = self.robot_controller.start_navigation_stack(use_sim_time)
            if result.success:
                click.echo(result.message)
                if result.data and 'process_id' in result.data:
                    click.echo(f"Process ID: {result.data['process_id']}")
            else:
                raise click.ClickException(result.message)

        @nav.group()
        def kill():
            """Kill navigation services"""
            pass

        @kill.command()
        def stack():
            """Kill navigation stack"""
            result = self.robot_controller.kill_navigation_stack()
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @cli.group()
        def calibrate():
            """Calibration commands"""
            pass

        @calibrate.command()
        @click.argument('meters', type=float)
        def square(meters):
            """Move robot in a square pattern for calibration"""
            result = self.robot_controller.calibrate_square(meters)
            if result.success:
                click.echo(result.message)
            else:
                raise click.ClickException(result.message)

        @cli.command()
        @click.argument('varname')
        @click.argument('value')
        def set(varname, value):
            """Set a variable to a value"""
            result = self.robot_controller.set_variable(varname, value)
            click.echo(result.message)

        @cli.command()
        @click.argument('varname', required=False)
        def show(varname):
            """Show variable value(s). Use 'show *' for all variables or 'show topics' for ROS topics"""
            if varname == 'topics':
                result = self.robot_controller.get_topics()
                if result.success:
                    click.echo("Active ROS topics:")
                    for topic in result.data['topics']:
                        type_names = ', '.join(topic['types'])
                        click.echo(f"  {topic['name']} [{type_names}]")
                else:
                    raise click.ClickException(result.message)
            elif varname and varname != '*':
                result = self.robot_controller.get_variable(varname)
                if result.success:
                    click.echo(f"  {result.message}")
                else:
                    raise click.ClickException(result.message)
            else:
                result = self.robot_controller.get_all_variables()
                if result.success:
                    for name, value in result.data['variables'].items():
                        click.echo(f"  {name} = {value} ({type(value).__name__})")
                else:
                    raise click.ClickException(result.message)

        return cli

    def process_command(self, command_line: str) -> CommandResult:
        if not command_line.strip():
            return CommandResult(False, "Empty command")

        try:
            # Capture click output and exceptions
            stdout_buffer = io.StringIO()
            stderr_buffer = io.StringIO()

            with redirect_stdout(stdout_buffer), redirect_stderr(stderr_buffer):
                try:
                    # Handle exit command specially
                    if command_line.strip().lower() == 'exit':
                        return CommandResult(True, "Exiting...", {"exit": True})

                    self.cli(command_line.split(), standalone_mode=False)
                    output = stdout_buffer.getvalue().strip()
                    return CommandResult(True, output if output else "Command completed")

                except click.ClickException as e:
                    if str(e.message) == "EXIT_SIGNAL":
                        return CommandResult(True, "Exiting...", {"exit": True})
                    error_msg = str(e.message) if hasattr(e, 'message') else str(e)
                    return CommandResult(False, error_msg)
                except click.UsageError as e:
                    return CommandResult(False, str(e))
                except SystemExit:
                    # Click sometimes raises SystemExit, capture any output
                    output = stdout_buffer.getvalue().strip()
                    error = stderr_buffer.getvalue().strip()
                    if error:
                        return CommandResult(False, error)
                    return CommandResult(True, output if output else "Command completed")

        except Exception as e:
            return CommandResult(False, f"Command error: {str(e)}")