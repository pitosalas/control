# Control Package

ROS2 robot control package with interactive CLI interface.

## Installation

### Prerequisites
- ROS2 (tested with Humble)
- Python 3.8+
- colcon build tools

### Install Dependencies
```bash
pip install prompt_toolkit click --break-system-packages
```

**Note**: The `--break-system-packages` flag bypasses Python's protection against installing packages system-wide. This can potentially conflict with system packages managed by your OS package manager. Consider using a virtual environment if you prefer to avoid this risk.

### Build Package
```bash
cd /home/pitosalas/ros2_ws
colcon build --packages-select control
source install/setup.bash
```

## Running

### Start the Interactive CLI
```bash
cd /home/pitosalas/ros2_ws/src/control
python control/main.py
```

### Available Commands

#### Movement
- `move dist <meters>` - Move forward/backward by distance
- `move time <seconds>` - Move for specified duration

#### Turning
- `turn time <seconds>` - Turn for specified duration
- `turn radians <angle>` - Turn by angle in radians
- `turn degrees <angle>` - Turn by angle in degrees

#### Control
- `stop` - Stop robot immediately

#### Calibration
- `calibrate square <meters>` - Move in square pattern for calibration

#### Configuration
- `set <variable> <value>` - Set configuration variable
- `show` - Show all variables
- `show <variable>` - Show specific variable
- `show *` - Show all variables
- `show topics` - Show active ROS topics

#### General
- `help` - Show all commands
- `help <command>` - Show help for specific command
- `exit` - Exit the program

### Configuration

Configuration is stored in `control_config.json` in the package directory.

Default speeds:
- `linear_speed`: 0.3 m/s
- `angular_speed`: 0.4 rad/s

### Features

- Command history with up/down arrow keys
- Automatic type conversion for variables (int, float, bool, string)
- Configuration persistence across sessions
- Real-time ROS topic listing
- Safety limits for movement speeds