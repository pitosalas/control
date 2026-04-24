#!/usr/bin/env python3
# intent_commands.py — CLI command definitions for intent publishing
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import control.commands.command_def as cd
import control.commands.parameter_def as pd


def build_intent_commands() -> dict[str, cd.CommandDef]:
    return {
        "intent.stop": cd.CommandDef(
            method_name="publish_intent_stop",
            parameters=[],
            description="Publish stop intent to /intent topic",
            group="intent"
        ),
        "intent.explore": cd.CommandDef(
            method_name="publish_intent_explore",
            parameters=[],
            description="Publish explore intent to /intent topic",
            group="intent"
        ),
        "intent.describe_scene": cd.CommandDef(
            method_name="publish_intent_describe_scene",
            parameters=[],
            description="Publish describe_scene intent to /intent topic",
            group="intent"
        ),
        "intent.count_objects": cd.CommandDef(
            method_name="publish_intent_count_objects",
            parameters=[
                pd.ParameterDef("object_type", str, True, None, "Type of object to count")
            ],
            description="Publish count_objects intent with required object_type",
            group="intent"
        ),
    }
