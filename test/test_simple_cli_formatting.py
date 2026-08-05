#!/usr/bin/env python3
# test_simple_cli_formatting.py — tests for CLI structured output formatting
# Test tier: No-ROS

import inspect

from dome_control.interface.simple_cli import SimpleCLI, format_table


def test_config_path_is_fixed_under_dome_config():
    src = inspect.getsource(SimpleCLI.__init__)
    assert "CONTROL_CONFIG" not in src
    assert '".dome"' in src and '"config"' in src and '"control.yaml"' in src


def test_format_table_aligns_columns():
    rows = [
        {"name": "linear_speed", "value": 0.2},
        {"name": "dry_run", "value": False},
    ]

    assert format_table(rows, [("name", "NAME"), ("value", "VALUE")]) == (
        "NAME          VALUE\n"
        "------------  -----\n"
        "linear_speed  0.2\n"
        "dry_run       no"
    )


def test_print_variables_uses_table(capsys):
    cli = object.__new__(SimpleCLI)

    cli.print_data({"variables": {"linear_speed": 0.2, "dry_run": False}})

    output = capsys.readouterr().out
    assert "NAME" in output
    assert "VALUE" in output
    assert "linear_speed" in output
    assert "dry_run" in output
    assert "no" in output


def test_print_status_uses_named_tables(capsys):
    cli = object.__new__(SimpleCLI)

    cli.print_data({
        "status": {
            "speeds": {"linear": 0.2, "angular": 1.5},
            "processes": {
                "robot": {
                    "running": True,
                    "process_id": "proc-1",
                    "pid": 1234,
                },
            },
            "nodes": {"movement_api": "running"},
        },
    })

    output = capsys.readouterr().out
    assert "configured speeds:" in output
    assert "launch processes:" in output
    assert "api nodes:" in output
    assert "RUNNING" in output
    assert "yes" in output
    assert "movement_api" in output


def test_print_subsystems_uses_table(capsys):
    cli = object.__new__(SimpleCLI)

    cli.print_data({
        "subsystems": {
            "gendrv": {"running": False, "processes": []},
            "control": {"running": True, "processes": [{"pid": "1"}]},
            "mission": {"running": True, "processes": [{"pid": "2"}], "state": "EXPLORING"},
        },
    })

    output = capsys.readouterr().out
    assert "SUBSYSTEM" in output
    assert "RUNNING" in output
    assert "DETAIL" in output
    assert "gendrv" in output and "no" in output
    assert "control" in output
    assert "mission" in output and "EXPLORING" in output
