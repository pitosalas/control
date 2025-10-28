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

### 3-Letter Abbreviations

**All commands support 3-letter abbreviations!** You can use either full words or abbreviated forms:

| Full | Abbreviation | Example Full Command | Example Abbreviated |
|------|--------------|---------------------|---------------------|
| move | mov | `move forward 1.5` | `mov fwd 1.5` |
| turn | trn | `turn clockwise 90` | `trn clk 90` |
| robot | rob | `robot stop` | `rob stp` |
| config | cfg | `config get linear_speed` | `cfg get linear_speed` |
| launch | lch | `launch start nav` | `lch sta nav` |
| script | scr | `script square -m 1` | `scr sqr -m 1` |
| system | sys | `system topics` | `sys top` |

**All 31 keyword abbreviations:**
all, bak (backward), clk (clockwise), cfg (config), ccw (counterclockwise), deg (degrees), dis (distance), doc (doctor), fwd (forward), get, kil (kill), lch (launch), lst (list), map, mov (move), prc (process), rad (radians), rob (robot), run (running), sav (save), scr (script), set, sqr (square), sta (start), sts (status), stp (stop), str (stress_test), sys (system), tim (time), top (topics), trn (turn)

### Available Commands

#### Movement (Simple Commands)
- `move forward <meters>` - Move robot forward
  - Example: `move forward 1.5`
- `move backward <meters>` - Move robot backward
  - Example: `move backward 2.0`
- `move time <seconds>` - Move for specified duration

#### Movement (Advanced)
- `move dist --meters <value>` - Move with explicit +/- direction
  - Example: `move dist -m 1.5` (forward)
  - Example: `move dist -m -1.5` (backward)

#### Turning (Simple Commands)
- `turn clockwise <degrees>` - Turn robot clockwise
  - Example: `turn clockwise 90`
- `turn counterclockwise <degrees>` - Turn robot counterclockwise
  - Example: `turn counterclockwise 90`
- `turn time <seconds>` - Turn for specified duration

#### Turning (Advanced)
- `turn degrees --degrees <angle>` - Turn with explicit +/- sign
  - Example: `turn degrees -d 90` (counterclockwise)
  - Example: `turn degrees -d -90` (clockwise)
- `turn radians --radians <angle>` - Turn by angle in radians
  - Example: `turn radians -r 1.57`

#### Control
- `robot stop` - Stop robot immediately
- `robot status` - Get current robot status

#### Script Commands
- `script square --meters <value>` or `script square -m <value>` - Move in square pattern
  - Example: `script square -m 1.0` (1m square)
  - Example: `script square -m -1.0` (1m square in reverse)
- `script stress_test` - Run continuous stress test with voltage monitoring

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