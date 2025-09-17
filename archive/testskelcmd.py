#!/usr/bin/env python3
"""
CLI implementation using Python's cmd module
"""
import cmd
import shlex


def run_command(command_name):
    """Helper function that prints the running command message."""
    print(f"running command {command_name}")


class InteractiveCLI(cmd.Cmd):
    """Interactive CLI application using Python's cmd module."""
    
    intro = 'Interactive CLI - Type commands or help for available commands'
    prompt = '>>> '
    
    def emptyline(self):
        """Override to do nothing on empty line."""
        pass
    
    # Single word commands
    
    def do_help(self, args):
        """Show available commands."""
        run_command("help")
        print("\nAvailable commands:")
        print("  help                           - Show this help")
        print("  exit                          - Exit the program")
        print("  show design [-brief]          - Show design information") 
        print("  print answer [number] [float] - Print answer with optional parameters")
        print()
    
    def do_exit(self, args):
        """Exit the program."""
        run_command("exit")
        print("Goodbye!")
        return True
    
    # Two word commands with subcommand handling
    
    def do_show(self, args):
        """Show commands.
        Usage: show design [-brief]
        """
        tokens = shlex.split(args) if args.strip() else []
        
        if not tokens:
            print("Error: 'show' requires a subcommand")
            print("Available: show design [-brief]")
            return
        
        subcommand = tokens[0]
        
        if subcommand == 'design':
            # Check for -brief flag
            command_str = "show design"
            if len(tokens) > 1 and tokens[1] == '-brief':
                command_str += " -brief"
            elif len(tokens) > 1:
                print(f"Error: Unknown option '{tokens[1]}'")
                print("Usage: show design [-brief]")
                return
            
            run_command(command_str)
        else:
            print(f"Error: Unknown subcommand '{subcommand}'")
            print("Available: show design [-brief]")
    
    def do_print(self, args):
        """Print commands.
        Usage: print answer [number] [float]
        """
        tokens = shlex.split(args) if args.strip() else []
        
        if not tokens:
            print("Error: 'print' requires a subcommand")
            print("Available: print answer [number] [float]")
            return
        
        subcommand = tokens[0]
        
        if subcommand == 'answer':
            command_str = "print answer"
            
            # Parse optional parameters
            if len(tokens) > 1:
                try:
                    # Try to parse first parameter as int
                    number = int(tokens[1])
                    command_str += f" {number}"
                    
                    # Try to parse second parameter as float if present
                    if len(tokens) > 2:
                        float_val = float(tokens[2])
                        command_str += f" {float_val}"
                        
                except ValueError as e:
                    print(f"Error: Invalid parameter - {e}")
                    print("Usage: print answer [number] [float]")
                    return
            
            run_command(command_str)
        else:
            print(f"Error: Unknown subcommand '{subcommand}'")
            print("Available: print answer [number] [float]")
    
    # Tab completion
    
    def complete_show(self, text, line, begidx, endidx):
        """Tab completion for show command."""
        words = line.split()
        if len(words) <= 2:  # Still completing subcommand
            options = ['design']
            return [option for option in options if option.startswith(text)]
        elif len(words) == 3 and words[1] == 'design':  # Completing options for 'show design'
            options = ['-brief']
            return [option for option in options if option.startswith(text)]
        return []
    
    def complete_print(self, text, line, begidx, endidx):
        """Tab completion for print command."""
        words = line.split()
        if len(words) <= 2:  # Still completing subcommand
            options = ['answer']
            return [option for option in options if option.startswith(text)]
        return []
    
    # Handle Ctrl+D
    def do_EOF(self, args):
        """Handle Ctrl+D (EOF)."""
        print()  # Print newline for clean exit
        return True


def main():
    """Entry point for the CLI application."""
    try:
        InteractiveCLI().cmdloop()
    except KeyboardInterrupt:
        print("\n\nGoodbye!")


if __name__ == '__main__':
    main()