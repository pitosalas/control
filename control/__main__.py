#!/usr/bin/env python3
"""Main entry point for running the control CLI."""

# New simple parser CLI (default)
from .interface.simple_cli import main as simple_cli_main

# Old Click-based CLI (kept for reference)
# from .interface.cli_interface import CLIInterface
#
# def main():
#     """Run the CLI interface."""
#     cli = CLIInterface()
#     cli.run()

def main():
    """Run the CLI interface using SimpleCLI."""
    simple_cli_main()

if __name__ == "__main__":
    main()