#!/usr/bin/env python3

from .command_processor import CommandProcessor

class CLIInterface:
    def __init__(self):
        self.processor = CommandProcessor()
        self.prompt = "> "

    def run(self):
        print("Robot Command Interface")
        print("Type 'help' for commands or 'exit' to quit")

        while True:
            try:
                command_line = input(self.prompt)
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