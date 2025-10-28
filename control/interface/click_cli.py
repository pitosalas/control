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

        # Movement commands (full: move, abbr: mov)
        @cli.group(context_settings={'allow_interspersed_args': False, 'ignore_unknown_options': False})
        def move():
            """Movement commands"""
            pass

        @move.command()
        @click.option('--meters', '-m', type=float, required=True, help='Distance in meters (negative = backward)')
        def dist(meters):
            """Move robot a specific distance in meters"""
            result = self.dispatcher.execute("move.distance", {"distance": meters})
            self._handle_result(result)

        @move.command()
        @click.argument('seconds', type=float)
        def time(seconds):
            """Move robot for specified time duration"""
            result = self.dispatcher.execute("move.time", {"seconds": seconds})
            self._handle_result(result)

        @click.command()
        @click.argument('meters', type=float)
        def forward(meters):
            """Move robot forward by distance in meters"""
            result = self.dispatcher.execute("move.forward", {"meters": meters})
            self._handle_result(result)

        move.add_command(forward, name='forward')
        move.add_command(forward, name='fwd')

        @click.command()
        @click.argument('meters', type=float)
        def backward(meters):
            """Move robot backward by distance in meters"""
            result = self.dispatcher.execute("move.backward", {"meters": meters})
            self._handle_result(result)

        move.add_command(backward, name='backward')
        move.add_command(backward, name='bak')

        # Turn commands (full: turn, abbr: trn)
        @cli.group(context_settings={'allow_interspersed_args': False, 'ignore_unknown_options': False})
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
        @click.option('--radians', '-r', type=float, required=True, help='Angle in radians (negative = clockwise)')
        def radians(radians):
            """Turn robot by specified angle in radians"""
            result = self.dispatcher.execute("turn.radians", {"radians": radians})
            self._handle_result(result)

        @turn.command()
        @click.option('--degrees', '-d', type=float, required=True, help='Angle in degrees (negative = clockwise)')
        def degrees(degrees):
            """Turn robot by specified angle in degrees"""
            result = self.dispatcher.execute("turn.degrees", {"degrees": degrees})
            self._handle_result(result)

        @click.command()
        @click.argument('degrees', type=float)
        def clockwise(degrees):
            """Turn robot clockwise by angle in degrees"""
            result = self.dispatcher.execute("turn.clockwise", {"degrees": degrees})
            self._handle_result(result)

        turn.add_command(clockwise, name='clockwise')
        turn.add_command(clockwise, name='clk')

        @click.command()
        @click.argument('degrees', type=float)
        def counterclockwise(degrees):
            """Turn robot counterclockwise by angle in degrees"""
            result = self.dispatcher.execute("turn.counterclockwise", {"degrees": degrees})
            self._handle_result(result)

        turn.add_command(counterclockwise, name='counterclockwise')
        turn.add_command(counterclockwise, name='ccw')

        # Robot control commands (full: robot, abbr: rob)
        @cli.group()
        def robot():
            """Robot control commands"""
            pass

        @click.command()
        def stop():
            """Stop robot movement"""
            result = self.dispatcher.execute("robot.stop", {})
            self._handle_result(result)

        robot.add_command(stop, name='stop')
        robot.add_command(stop, name='stp')

        @click.command()
        def status():
            """Get current robot status"""
            result = self.dispatcher.execute("robot.status", {})
            self._handle_result(result)

        robot.add_command(status, name='status')
        robot.add_command(status, name='sts')

        # Launch commands (full: launch, abbr: lch)
        @cli.group()
        def launch():
            """Launch commands"""
            pass

        @click.command()
        def launch_list():
            """List available launch types and their status"""
            result = self.dispatcher.execute("launch.list", {})
            self._handle_result(result)

        launch.add_command(launch_list, name='list')
        launch.add_command(launch_list, name='lst')

        @click.command()
        @click.argument('launch_type')
        @click.option('--sim-time', is_flag=True, help='Use simulation time')
        @click.option('--map-name', help='Map name for map launch type (without extension)')
        def launch_start(launch_type, sim_time, map_name):
            """Start a specific launch file type (nav, slam, map)"""
            params = {"launch_type": launch_type}
            if sim_time:
                params["use_sim_time"] = sim_time
            if map_name:
                params["map_name"] = map_name
            result = self.dispatcher.execute("launch.start", params)
            self._handle_result(result)

        launch.add_command(launch_start, name='start')
        launch.add_command(launch_start, name='sta')

        @click.command()
        @click.argument('launch_type')
        def launch_kill(launch_type):
            """Stop a specific launch file type"""
            result = self.dispatcher.execute("launch.kill", {"launch_type": launch_type})
            self._handle_result(result)

        launch.add_command(launch_kill, name='kill')
        launch.add_command(launch_kill, name='kil')

        @click.command()
        @click.argument('launch_type', required=False)
        def launch_status(launch_type):
            """Show status of launch processes"""
            params = {}
            if launch_type:
                params["launch_type"] = launch_type
            result = self.dispatcher.execute("launch.status", params)
            self._handle_result(result)

        launch.add_command(launch_status, name='status')
        launch.add_command(launch_status, name='sts')

        @launch.command()
        def doctor():
            """Diagnose launch process conflicts and suggest fixes"""
            result = self.dispatcher.execute("launch.doctor", {})
            self._handle_result(result)

        @launch.command(name='kill-all')
        @click.argument('launch_type')
        def kill_all(launch_type):
            """Kill ALL instances of a launch type (tracked + external)"""
            result = self.dispatcher.execute("launch.kill-all", {"launch_type": launch_type})
            self._handle_result(result)

        # Map commands (full: map, abbr: map - already 3 letters)
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



        # Configuration commands (full: config, abbr: cfg)
        @cli.group()
        def config():
            """Configuration commands"""
            pass

        @click.command()
        @click.argument('name')
        @click.argument('value')
        def config_set(name, value):
            """Set configuration variable"""
            result = self.dispatcher.execute("config.set", {"name": name, "value": value})
            self._handle_result(result)

        config.add_command(config_set, name='set')

        @click.command()
        @click.argument('name')
        def config_get(name):
            """Get configuration variable"""
            result = self.dispatcher.execute("config.get", {"name": name})
            self._handle_result(result)

        config.add_command(config_get, name='get')

        @click.command()
        def config_list():
            """List all configuration variables"""
            result = self.dispatcher.execute("config.list", {})
            self._handle_result(result)

        config.add_command(config_list, name='list')
        config.add_command(config_list, name='lst')

        # System commands (full: system, abbr: sys)
        @cli.group()
        def system():
            """System commands"""
            pass

        @click.command()
        def system_topics():
            """List active ROS topics"""
            result = self.dispatcher.execute("system.topics", {})
            self._handle_result(result)

        system.add_command(system_topics, name='topics')
        system.add_command(system_topics, name='top')

        # Script commands (full: script, abbr: scr)
        @cli.group()
        def script():
            """Script commands"""
            pass

        @click.command()
        @click.argument('meters', type=float)
        def script_square(meters):
            """Execute square movement pattern"""
            result = self.dispatcher.execute("script.square", {"meters": meters})
            self._handle_result(result)

        script.add_command(script_square, name='square')
        script.add_command(script_square, name='sqr')

        @click.command()
        def script_stress_test():
            """Run continuous stress test with voltage monitoring"""
            result = self.dispatcher.execute("script.stress_test", {})
            self._handle_result(result)

        script.add_command(script_stress_test, name='stress_test')
        script.add_command(script_stress_test, name='str')

        return cli

    def _generate_help_from_commands(self, cli):
        """Generate help text automatically from Click command structure."""
        help_text = ""

        # Group commands by their parent group
        groups = {}

        # Define abbreviations to skip
        abbreviations = {'fwd', 'bak', 'clk', 'ccw', 'sqr', 'str', 'stp', 'sts',
                        'lst', 'sta', 'kil', 'top', 'deg', 'rad', 'dis', 'tim', 'doc'}

        for name, command in cli.commands.items():
            if isinstance(command, click.Group):
                # This is a command group
                group_name = name.upper()
                groups[group_name] = []

                for subname, subcmd in command.commands.items():
                    # Skip abbreviated command names
                    if subname in abbreviations:
                        continue

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
        # Define abbreviations to skip
        abbreviations = {'fwd', 'bak', 'clk', 'ccw', 'sqr', 'str', 'stp', 'sts',
                        'lst', 'sta', 'kil', 'top', 'deg', 'rad', 'dis', 'tim', 'doc'}

        # Check if it's a command group
        if command_name in cli.commands:
            command = cli.commands[command_name]

            if isinstance(command, click.Group):
                # Generate help for the entire group without header
                help_text = ""

                for subname, subcmd in command.commands.items():
                    # Skip abbreviated command names
                    if subname in abbreviations:
                        continue

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