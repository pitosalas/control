#!/usr/bin/env python3
"""
CLI implementation using Click
"""
import click
import sys


def run_command(command_name):
    """Helper function that prints the running command message."""
    print(f"running command {command_name}")


@click.group(invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """Interactive CLI application using Click."""
    if ctx.invoked_subcommand is None:
        interactive_mode()


def interactive_mode():
    """Start interactive mode where user can enter commands."""
    print("Interactive CLI - Type commands or 'help' for available commands")
    
    while True:
        try:
            # Get user input
            user_input = input(">>> ").strip()
            
            # Handle empty input
            if not user_input:
                continue
            
            # Handle exit commands
            if user_input.lower() in ['exit', 'quit']:
                print("Goodbye!")
                break
            
            # Parse and execute command
            try:
                args = user_input.split()
                
                # Handle help specially
                if args == ['help']:
                    show_help()
                    continue
                
                # Create context and invoke command
                with cli.make_context('cli', args, resilient_parsing=True) as ctx:
                    try:
                        cli.invoke(ctx)
                    except click.UsageError as e:
                        print(f"Error: {e}")
                    except click.ClickException as e:
                        e.show()
                    except SystemExit:
                        pass  # Prevent program exit on command errors
                        
            except Exception as e:
                print(f"Error parsing command: {e}")
                
        except KeyboardInterrupt:
            print("\nUse 'exit' to quit.")
        except EOFError:
            print("\nGoodbye!")
            break


def show_help():
    """Show available commands."""
    print("\nAvailable commands:")
    print("  help                           - Show this help")
    print("  exit                          - Exit the program")
    print("  show design [-brief]          - Show design information")
    print("  print answer [number] [float] - Print answer with optional parameters")
    print()


@cli.command()
def help():
    """Show available commands."""
    run_command("help")
    show_help()


@cli.command()
def exit():
    """Exit the program."""
    run_command("exit")
    print("Goodbye!")
    sys.exit(0)


@cli.group()
def show():
    """Show commands."""
    pass


@show.command()
@click.option('--brief', '-brief', is_flag=True, help='Show brief information')
def design(brief):
    """Show design information."""
    command_str = "show design"
    if brief:
        command_str += " -brief"
    run_command(command_str)


@cli.group(name='print')
def print_group():
    """Print commands."""
    pass


@print_group.command()
@click.argument('number', required=False, type=int)
@click.argument('float_val', required=False, type=float)
def answer(number, float_val):
    """Print answer with optional parameters."""
    command_str = "print answer"
    if number is not None:
        command_str += f" {number}"
    if float_val is not None:
        command_str += f" {float_val}"
    run_command(command_str)


if __name__ == '__main__':
    cli()