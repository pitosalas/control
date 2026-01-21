#!/usr/bin/env python3
"""Main entry point for running the control CLI."""

import control.interface.simple_cli as cli

# Old Click-based CLI (kept for reference)
# from .interface.cli_interface import CLIInterface
#
# def main():
#     """Run the CLI interface."""
#     cli = CLIInterface()
#     cli.run()

def main():
    """Run the CLI interface using SimpleCLI."""
    cli.main()

if __name__ == "__main__":
    main()