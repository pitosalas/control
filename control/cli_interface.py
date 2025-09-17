#!/usr/bin/env python3

import click
from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory
from pathlib import Path
from .click_cli import ClickCLI


class CLIInterface:
    def __init__(self):
        self.click_cli = ClickCLI()
        self.cli = self.click_cli.get_cli()
        self.prompt_text = "> "
        history_file = Path.home() / ".config" / "control_command_history.txt"
        self.history = FileHistory(str(history_file))

    def run(self):
        print("Robot Command Interface")
        print("Type 'help' for commands or 'exit' to quit")

        self._interactive_mode()

    def _interactive_mode(self):
        """Interactive mode using testskelclick.py pattern."""
        while True:
            try:
                # Get user input with history
                user_input = prompt(self.prompt_text, history=self.history).strip()

                # Handle empty input
                if not user_input:
                    continue

                # Handle exit commands
                if user_input.lower() in ['exit', 'quit', 'q']:
                    print("Goodbye!")
                    break

                # Parse and execute command using Click
                try:
                    args = user_input.split()

                    # Create context and invoke command with resilient parsing
                    with self.cli.make_context('cli', args, resilient_parsing=True) as ctx:
                        try:
                            self.cli.invoke(ctx)
                        except click.UsageError as e:
                            print(f"✗ {e}")
                        except click.ClickException as e:
                            e.show()
                        except SystemExit:
                            # Prevent program exit on command errors
                            pass

                except Exception as e:
                    print(f"✗ Error parsing command: {e}")

            except KeyboardInterrupt:
                print("\nUse 'exit' to quit.")
            except EOFError:
                print("\nGoodbye!")
                break

        # Save configuration on exit
        self.click_cli.config_manager.save_config()