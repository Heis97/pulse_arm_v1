from __future__ import annotations

from ..classes.enum_classes.controller_commands import RealtimeCommands
from .command_registry import CommandRegistry, CommandSpec


def init_realtime_controller_commands() -> None:
    CommandRegistry.register(CommandSpec(RealtimeCommands.REALTIME_STOPJ))
    CommandRegistry.register(
        CommandSpec(
            RealtimeCommands.REALTIME_SERVOJ,
            pack_fmt="7d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            RealtimeCommands.REALTIME_SPEEDJ,
            pack_fmt="7d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            RealtimeCommands.REALTIME_SET_SERVOJ_PARAMS,
            pack_fmt="2d",
        )
    )
