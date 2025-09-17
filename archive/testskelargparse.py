#!/usr/bin/env python3
"""
CLI implementation using argparse
"""
import argparse
import shlex
import sys


def run_command(command_name):
    """Helper function that prints the running command message."""
    print(f"running command {command_name}")


def create_parser():
    """Create the argument parser with all commands."""
    parser = argparse.ArgumentParser(
        prog='cli',
        description='Interactive CLI application using argparse',
        add_help=False  # We'll handle help manually
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Help command
    help_parser = subparsers.add_parser('help', help='Show available commands')
    
    # Exit command  
    exit_parser = subparsers.add_parser('exit', help='Exit the program')
    
    # Show command group
    show_parser = subparsers.add_parser('show', help='Show commands')
    show_subparsers = show_parser.add_subparsers(dest='show_subcommand')
    
    # Show design command
    design_parser = show_subparsers.add_parser('design', help='Show design information')
    design_parser.add_argument('-brief', action='store_true', help='Show brief information')
    
    # Print command group
    print_parser = subparsers.add_parser('print', help='Print commands')
    print_subparsers = print_parser.add_subparsers(dest='print_subcommand')
    
    # Print answer command
    answer_parser = print_subparsers.add_parser('answer', help='Print answer with optional parameters')
    answer_parser.add_argument('number', nargs='?', type=int, help='Optional number parameter')
    answer_parser.add_argument('float_val', nargs='?', type=float, help='Optional float parameter')
    
    return parser


def show_help():
    """Show available commands."""
    print("\nAvailable commands:")
    print("  help                           - Show this help")
    print("  exit                          - Exit the program") 
    print("  show design [-brief]          - Show design information")
    print("  print answer [number] [float] - Print answer with optional parameters")
    print()


def handle_command(args):
    """Handle parsed command arguments."""
    if args.command == 'help':
        run_command("help")
        show_help()
        
    elif args.command == 'exit':
        run_command("exit")
        print("Goodbye!")
        return True  # Signal to exit
        
    elif args.command == 'show':
        if args.show_subcommand == 'design':
            command_str = "show design"
            if args.brief:
                command_str += " -brief"
            run_command(command_str)
        else:
            print("Error: 'show' requires a subcommand (design)")
            
    elif args.command == 'print':
        if args.print_subcommand == 'answer':
            command_str = "print answer"
            if args.number is not None:
                command_str += f" {args.number}"
            if args.float_val is not None:
                command_str += f" {args.float_val}"
            run_command(command_str)
        else:
            print("Error: 'print' requires a subcommand (answer)")
            
    else:
        print(f"Unknown command: {args.command}")
        print("Type 'help' for available commands")
    
    return False  # Continue running


def interactive_mode():
    """Start interactive mode."""
    parser = create_parser()
    print("Interactive CLI - Type commands or 'help' for available commands")
    
    while True:
        try:
            # Get user input
            user_input = input(">>> ").strip()
            
            # Handle empty input
            if not user_input:
                continue
            
            # Parse the command
            try:
                args = parser.parse_args(shlex.split(user_input))
                should_exit = handle_command(args)
                if should_exit:
                    break
                    
            except argparse.ArgumentError as e:
                print(f"Error: {e}")
            except SystemExit:
                # argparse calls sys.exit on error, catch and continue
                print("Type 'help' for available commands")
                
        except KeyboardInterrupt:
            print("\nUse 'exit' to quit.")
        except EOFError:
            print("\nGoodbye!")
            break


def main():
    """Entry point for the CLI application."""
    try:
        interactive_mode()
    except KeyboardInterrupt:
        print("\n\nGoodbye!")


if __name__ == '__main__':
    main()