#!/usr/bin/env python3
import rclpy
import control.interface.simple_cli as cli


def main():
    """Run the CLI interface using SimpleCLI."""
    rclpy.init()
    try:
        cli.main()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
