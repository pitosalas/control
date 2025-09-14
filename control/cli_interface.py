#!/usr/bin/env python3

from .command_processor import CommandProcessor
from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory
from pathlib import Path

class CLIInterface:
    def __init__(self):
        self.processor = CommandProcessor()
        self.prompt_text = "> "
        history_file = Path(__file__).parent.parent / "command_history.txt"
        self.history = FileHistory(str(history_file))

    def run(self):
        print("Robot Command Interface")
        print("Type 'help' for commands or 'exit' to quit")

        while True:
            try:
                command_line = prompt(self.prompt_text, history=self.history)
                if command_line.lower() in ['quit', 'q']:
                    break

                if command_line.strip():
                    result = self.processor.process_command(command_line)
                    if result.success:
                        if result.message:
                            print(f"✓ {result.message}")
                        # Check if exit command was called
                        if result.data and result.data.get("exit"):
                            break
                    else:
                        if result.message:
                            print(f"✗ {result.message}")

            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except EOFError:
                break

        # Save configuration on exit
        self.processor.save_config()