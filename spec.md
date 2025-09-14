# Project

    * We are creating a python tool for ros2 which will give commands and control a robot. There will be three command interfaces. 
    * CLI mode will print a prompt and the user will type in commands
    * TUI mode will be use textual to provide a fancier interface (future)
    * ros2 topic mode will subscribe to a new topic which will receive the commands
    * We want the same command "back end" to implement all three
    * We will start with easy commands throught the cli
    * The commands will have a standard syntax that is easy to parse: <command> <subcommand> <arguments>
    * We will use typer as the standard parsing as an experiment
    * After each command I want to be able to test it
    * The prompt will be "> " initially
    * All commands can be abbreviated to the first four letters
    * The first command is move dist <float>
    

