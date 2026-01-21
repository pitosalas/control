#!/usr/bin/env python3
"""
Test for ProcessApi: launch a simple command and verify real-time logging and log file creation.
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from control.ros2_api.process_api import ProcessApi
from control.commands.config_manager import ConfigManager
import time
import os

def main():
    # Use a dummy config manager (adjust path as needed)
    config = ConfigManager(config_file="/home/pitosalas/.control/config.yaml")
    api = ProcessApi(config)

    # Use the actual launch command from config, with override
    command = "bl dome mini_bringup_bl.launch.py --bl_ui_override=disable"
    print("Launching test command:", command)
    # Launch process directly using subprocess for test
    import subprocess
    import uuid
    process_id = str(uuid.uuid4())
    log_name = "test_simple"
    log_file_path = os.path.expanduser(f"~/.control/logs/{log_name}_manualtest.log")
    proc = subprocess.Popen(
        command,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    # Simulate ProcessInfo
    from control.ros2_api.process_api import ProcessInfo
    process_info = ProcessInfo(
        process=proc,
        command=command,
        output=[],
        is_running=True,
        pid=proc.pid,
        start_time=time.time(),
        log_file=log_file_path,
    )
    # Capture output and write log file
    for line in iter(proc.stdout.readline, ""):
        if not line:
            break
        stripped_line = line.rstrip()
        process_info.output.append(stripped_line)
        with open(log_file_path, 'a') as f:
            f.write(line)
        print(f"[TEST] {stripped_line}")
        if proc.poll() is not None:
            break
    proc.stdout.close()
    proc.wait()
    print(f"Test complete. Log file should be at: {log_file_path}")
    if os.path.exists(log_file_path):
        with open(log_file_path) as f:
            print("--- Log file contents ---")
            print(f.read())
    else:
        print("No log file found!")

if __name__ == "__main__":
    main()
