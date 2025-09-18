#!/usr/bin/env python3
from .interface.cli_interface import CLIInterface

def main():
    interface = CLIInterface()
    interface.run()

if __name__ == '__main__':
    main()