#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import typer
from .cli_interface import CLIInterface

class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.get_logger().info("Control Node initialized")

app = typer.Typer()

@app.command()
def cli():
    """Run the interactive CLI interface"""
    interface = CLIInterface()
    interface.run()

@app.command()
def node():
    """Run as ROS2 node"""
    rclpy.init()
    node = Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    app()

if __name__ == '__main__':
    main()