#!/usr/bin/env python3
import click
import sys
from ..commands.command_dispatcher import CommandDispatcher
from ..commands.robot_controller import RobotController
from ..commands.config_manager import ConfigManager


class ClickCLI:
    """Click-based CLI that uses the CommandDispatcher."""

    def __init__(self):
        self.config_manager = ConfigManager()
        self.robot_controller = RobotController(self.config_manager)
        self.dispatcher = CommandDispatcher(self.robot_controller)
        self.cli = self._create_cli()

    def _create_cli(self):
        """Create the Click CLI with dispatcher integration."""

        @click.group(invoke_without_command=True, context_settings={'allow_interspersed_args': False, 'ignore_unknown_options': True})
        @click.pass_context
        def cli(ctx):
            """Robot Command Interface"""
            # This will be called in interactive mode
            pass

        @cli.command()
        @click.argument('command_name', required=False)
        def help(command_name):
            """Show help for all commands or a specific command"""
            if command_name:
                # Show help for specific command group or command
                help_text = self._generate_specific_help(cli, command_name)
                click.echo(help_text)
            else:
                # Generate help automatically from Click command structure
                help_text = self._generate_help_from_commands(cli)
                click.echo(help_text)

        @cli.command()
        def exit():
            """Exit the program"""
            click.echo("Goodbye!")
            sys.exit(0)

        @cli.command()
        @click.argument('name')
        @click.argument('value')
        def set(name, value):
            """Set configuration variable"""
            result = self.dispatcher.execute("config.set", {"name": name, "value": value})
            self._handle_result(result)

        @cli.command()
        @click.argument('name')
        def get(name):
            """Get configuration variable or get * for all variables"""
            if name == "*":
                result = self.dispatcher.execute("config.list", {})
            else:
                result = self.dispatcher.execute("config.get", {"name": name})
            self._handle_result(result)

        # Movement commands
        @cli.group(context_settings={'allow_interspersed_args': False, 'ignore_unknown_options': True})
        def move():
            """Movement commands"""
            pass

        @move.command()
        @click.argument('distance', type=float)
        def dist(distance):
            """Move robot a specific distance in meters"""
            result = self.dispatcher.execute("move.distance", {"distance": distance})
            self._handle_result(result)

        @move.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Move robot for specified time duration"""
            result = self.dispatcher.execute("move.time", {"seconds": seconds})
            self._handle_result(result)

        # Turn commands
        @cli.group(context_settings={'allow_interspersed_args': False})
        def turn():
            """Turn commands"""
            pass

        @turn.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Turn robot for specified time duration"""
            result = self.dispatcher.execute("turn.time", {"seconds": seconds})
            self._handle_result(result)

        @turn.command()
        @click.argument('radians', type=float)
        def radians(radians):
            """Turn robot by specified angle in radians"""
            result = self.dispatcher.execute("turn.radians", {"radians": radians})
            self._handle_result(result)

        @turn.command()
        @click.argument('degrees', type=float)
        def degrees(degrees):
            """Turn robot by specified angle in degrees"""
            result = self.dispatcher.execute("turn.degrees", {"degrees": degrees})
            self._handle_result(result)

        # Robot control commands
        @cli.group()
        def robot():
            """Robot control commands"""
            pass

        @robot.command()
        def stop():
            """Stop robot movement"""
            result = self.dispatcher.execute("robot.stop", {})
            self._handle_result(result)

        @robot.command()
        def status():
            """Get current robot status"""
            result = self.dispatcher.execute("robot.status", {})
            self._handle_result(result)

        # Navigation commands
        @cli.group()
        def nav():
            """Navigation commands"""
            pass

        @nav.command()
        @click.option('--sim-time', is_flag=True, help='Use simulation time')
        def start(sim_time):
            """Start the navigation stack"""
            result = self.dispatcher.execute("nav.start", {"use_sim_time": sim_time})
            self._handle_result(result)

        @nav.command()
        def stop():
            """Stop the navigation stack"""
            result = self.dispatcher.execute("nav.stop", {})
            self._handle_result(result)

        # SLAM commands
        @cli.group()
        def slam():
            """SLAM commands"""
            pass

        @slam.command()
        @click.option('--sim-time', is_flag=True, help='Use simulation time')
        def start(sim_time):
            """Start SLAM"""
            result = self.dispatcher.execute("slam.start", {"use_sim_time": sim_time})
            self._handle_result(result)

        @slam.command()
        def stop():
            """Stop SLAM"""
            result = self.dispatcher.execute("slam.stop", {})
            self._handle_result(result)

        # Map commands
        @cli.group()
        def map():
            """Map commands"""
            pass

        @map.command()
        @click.argument('filename')
        def save(filename):
            """Save current map to maps/ folder"""
            result = self.dispatcher.execute("map.save", {"filename": filename})
            self._handle_result(result)

        @map.command()
        def list():
            """List available maps in maps/ folder"""
            result = self.dispatcher.execute("map.list", {})
            self._handle_result(result)

        @map.command()
        @click.argument('filename')
        def load(filename):
            """Load a map from maps/ folder"""
            result = self.dispatcher.execute("map.load", {"filename": filename})
            self._handle_result(result)

        @map.command()
        def stop_save():
            """Stop map save operation"""
            result = self.dispatcher.execute("map.stop_save", {})
            self._handle_result(result)

        @map.command()
        def stop_load():
            """Stop map load operation"""
            result = self.dispatcher.execute("map.stop_load", {})
            self._handle_result(result)

        # Configuration commands
        @cli.group()
        def config():
            """Configuration commands"""
            pass

        @config.command()
        @click.argument('name')
        @click.argument('value')
        def set(name, value):
            """Set configuration variable"""
            result = self.dispatcher.execute("config.set", {"name": name, "value": value})
            self._handle_result(result)

        @config.command()
        @click.argument('name')
        def get(name):
            """Get configuration variable"""
            result = self.dispatcher.execute("config.get", {"name": name})
            self._handle_result(result)

        @config.command()
        def list():
            """List all configuration variables"""
            result = self.dispatcher.execute("config.list", {})
            self._handle_result(result)

        # System commands
        @cli.group()
        def system():
            """System commands"""
            pass

        @system.command()
        def topics():
            """List active ROS topics"""
            result = self.dispatcher.execute("system.topics", {})
            self._handle_result(result)

        # Calibration commands
        @cli.group()
        def calibrate():
            """Calibration commands"""
            pass

        @calibrate.command()
        @click.argument('meters', type=float)
        def square(meters):
            """Perform square calibration pattern"""
            result = self.dispatcher.execute("calibrate.square", {"meters": meters})
            self._handle_result(result)

        return cli

    def _generate_help_from_commands(self, cli):
        """Generate help text automatically from Click command structure."""
        help_text = ""

        # Group commands by their parent group
        groups = {}

        for name, command in cli.commands.items():
            if isinstance(command, click.Group):
                # This is a command group
                group_name = name.upper()
                groups[group_name] = []

                for subname, subcmd in command.commands.items():
                    # Get parameters for the subcommand
                    params = []
                    for param in subcmd.params:
                        if isinstance(param, click.Argument):
                            params.append(f"<{param.name}>")
                        elif isinstance(param, click.Option):
                            if param.is_flag:
                                params.append(f"[{param.opts[0]}]")
                            else:
                                params.append(f"[{param.opts[0]} <value>]")

                    param_str = " ".join(params)
                    full_cmd = f"{name} {subname} {param_str}".strip()
                    description = subcmd.help or subcmd.__doc__ or "No description"
                    groups[group_name].append((full_cmd, description))

            elif name not in ['help', 'exit']:
                # Standalone command - these are legacy/deprecated
                if 'OTHER' not in groups:
                    groups['OTHER'] = []
                description = command.help or command.__doc__ or "No description"
                groups['OTHER'].append((name, description))

        # Add help and exit to OTHER
        if 'OTHER' not in groups:
            groups['OTHER'] = []
        groups['OTHER'].extend([
            ('help [command]', 'Show help for all commands or specific command'),
            ('exit', 'Exit the program')
        ])

        # Format the help text with no indentation in alphabetical order
        for group_name in sorted(groups.keys()):
            for cmd, desc in groups[group_name]:
                help_text += f"{cmd:<32} - {desc}\n"

        return help_text

    def _generate_specific_help(self, cli, command_name):
        """Generate help for a specific command or command group."""
        # Check if it's a command group
        if command_name in cli.commands:
            command = cli.commands[command_name]

            if isinstance(command, click.Group):
                # Generate help for the entire group without header
                help_text = ""

                for subname, subcmd in command.commands.items():
                    # Get parameters for the subcommand
                    params = []
                    for param in subcmd.params:
                        if isinstance(param, click.Argument):
                            params.append(f"<{param.name}>")
                        elif isinstance(param, click.Option):
                            if param.is_flag:
                                params.append(f"[{param.opts[0]}]")
                            else:
                                params.append(f"[{param.opts[0]} <value>]")

                    param_str = " ".join(params)
                    full_cmd = f"{command_name} {subname} {param_str}".strip()
                    description = subcmd.help or subcmd.__doc__ or "No description"
                    help_text += f"{full_cmd:<32} - {description}\n"

                return help_text
            else:
                # Single command
                description = command.help or command.__doc__ or "No description"
                return f"{command_name} - {description}\n"

        # Check if it's a subcommand (e.g., "config set")
        parts = command_name.split()
        if len(parts) == 2:
            group_name, sub_name = parts
            if group_name in cli.commands:
                group = cli.commands[group_name]
                if isinstance(group, click.Group) and sub_name in group.commands:
                    subcmd = group.commands[sub_name]

                    # Get parameters for the subcommand
                    params = []
                    for param in subcmd.params:
                        if isinstance(param, click.Argument):
                            params.append(f"<{param.name}>")
                        elif isinstance(param, click.Option):
                            if param.is_flag:
                                params.append(f"[{param.opts[0]}]")
                            else:
                                params.append(f"[{param.opts[0]} <value>]")

                    param_str = " ".join(params)
                    full_cmd = f"{group_name} {sub_name} {param_str}".strip()
                    description = subcmd.help or subcmd.__doc__ or "No description"

                    return f"{full_cmd} - {description}\n"

        return f"Command '{command_name}' not found. Use 'help' to see all available commands.\n"

    def _handle_result(self, result):
        """Handle command result and display to user."""
        if result.success:
            if result.message:
                click.echo(f"✓ {result.message}")

            # Display data if it contains meaningful information
            if result.data:
                self._display_result_data(result.data)
        else:
            if result.message:
                click.echo(f"✗ {result.message}")
            else:
                click.echo("✗ Command failed")

    def _display_result_data(self, data):
        """Display result data in a formatted way."""
        if isinstance(data, dict):
            for key, value in data.items():
                if isinstance(value, dict):
                    click.echo(f"{key}:")
                    for sub_key, sub_value in value.items():
                        click.echo(f"  {sub_key}: {sub_value}")
                else:
                    click.echo(f"{key}: {value}")
        else:
            click.echo(str(data))

    def get_cli(self):
        """Get the Click CLI instance."""
        return self.cli