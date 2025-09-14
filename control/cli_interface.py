#!/usr/bin/env python3

from .command_processor import CommandProcessor

class CLIInterface:
    def __init__(self):
        self.processor = CommandProcessor()
        self.prompt = "> "

    def run(self):
        print("Robot Command Interface")
        print("Type 'quit' to exit")

        while True:
            try:
                command_line = input(self.prompt)
                if command_line.lower() in ['quit', 'exit', 'q']:
                    break

                if command_line.strip():
                    result = self.processor.process_command(command_line)
                    if result.success:
                        print(f"✓ {result.message}")
                    else:
                        print(f"✗ {result.message}")

            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except EOFError:
                break