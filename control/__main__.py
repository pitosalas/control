#!/usr/bin/env python3
"""Main entry point for running the control CLI."""

from .cli_interface import CLIInterface

def main():
    """Run the CLI interface."""
    cli = CLIInterface()
    cli.run()

if __name__ == "__main__":
    main()