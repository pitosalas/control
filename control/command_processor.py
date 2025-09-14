#!/usr/bin/env python3

import click
import io
from contextlib import redirect_stdout, redirect_stderr
from typing import Any
from dataclasses import dataclass
from .teleopapi import TeleopApi
from .config_manager import ConfigManager

@dataclass
class CommandResult:
    success: bool
    message: str
    data: Any = None

class CommandProcessor:
    def __init__(self):
        self.teleop_api = None
        self.config_manager = ConfigManager()
        self.cli = self._create_cli()

    def _get_teleop_api(self):
        if self.teleop_api is None:
            self.teleop_api = TeleopApi()
        return self.teleop_api

    def save_config(self):
        """Save configuration to file"""
        self.config_manager.save_config()

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
            try:
                api = self._get_teleop_api()
                api.move_dist(distance)
                click.echo(f"Moved {distance} meters")
            except Exception as e:
                raise click.ClickException(f"Movement failed: {str(e)}")

        @move.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Move robot at default speed for specified duration in seconds"""
            try:
                api = self._get_teleop_api()
                api.move_time(seconds)
                click.echo(f"Moved for {seconds} seconds")
            except Exception as e:
                raise click.ClickException(f"Movement failed: {str(e)}")

        @cli.group()
        def turn():
            """Turn the robot"""
            pass

        @turn.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Turn robot at default speed for specified duration in seconds"""
            try:
                api = self._get_teleop_api()
                api.turn_time(seconds)
                click.echo(f"Turned for {seconds} seconds")
            except Exception as e:
                raise click.ClickException(f"Turn failed: {str(e)}")

        @turn.command()
        @click.argument('angle', type=float)
        def radians(angle):
            """Turn robot by specific angle in radians"""
            try:
                api = self._get_teleop_api()
                api.turn_amount(angle)
                click.echo(f"Turned {angle} radians")
            except Exception as e:
                raise click.ClickException(f"Turn failed: {str(e)}")

        @turn.command()
        @click.argument('angle', type=float)
        def degrees(angle):
            """Turn robot by specific angle in degrees"""
            try:
                api = self._get_teleop_api()
                api.turn_degrees(angle)
                click.echo(f"Turned {angle} degrees")
            except Exception as e:
                raise click.ClickException(f"Turn failed: {str(e)}")

        @cli.command()
        def stop():
            """Stop the robot immediately"""
            try:
                api = self._get_teleop_api()
                api.stop()
                click.echo("Robot stopped")
            except Exception as e:
                raise click.ClickException(f"Stop failed: {str(e)}")

        @cli.group()
        def calibrate():
            """Calibration commands"""
            pass

        @calibrate.command()
        @click.argument('meters', type=float)
        def square(meters):
            """Move robot in a square pattern for calibration"""
            try:
                api = self._get_teleop_api()
                api.calibrate_square(meters)
                click.echo(f"Completed square calibration with {meters}m sides")
            except Exception as e:
                raise click.ClickException(f"Calibration failed: {str(e)}")

        @cli.command()
        @click.argument('varname')
        @click.argument('value')
        def set(varname, value):
            """Set a variable to a value"""
            self.config_manager.set_variable(varname, value)
            stored_value = self.config_manager.get_variable(varname)
            click.echo(f"Set {varname} = {stored_value} ({type(stored_value).__name__})")

        @cli.command()
        @click.argument('varname', required=False)
        def show(varname):
            """Show variable value(s). Use 'show *' for all variables or 'show topics' for ROS topics"""
            if varname == 'topics':
                try:
                    api = self._get_teleop_api()
                    topics = api.get_topics()
                    click.echo("Active ROS topics:")
                    for topic_name, topic_types in topics:
                        type_names = [t for t in topic_types]
                        click.echo(f"  {topic_name} [{', '.join(type_names)}]")
                except Exception as e:
                    raise click.ClickException(f"Failed to get topics: {str(e)}")
            elif varname and varname != '*':
                value = self.config_manager.get_variable(varname)
                click.echo(f"  {varname} = {value} ({type(value).__name__})")
            else:
                variables = self.config_manager.get_all_variables()
                for name, value in variables.items():
                    click.echo(f"  {name} = {value} ({type(value).__name__})")

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