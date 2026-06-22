from .command_registry import CommandRegistry
from .controller_protocol import init_controller_commands
from .realtime_protocol import init_realtime_controller_commands

_initialized = False


def _ensure_initialized() -> None:
    global _initialized
    if _initialized:
        return
    init_controller_commands()
    init_realtime_controller_commands()
    _initialized = True


_ensure_initialized()

__all__ = ["CommandRegistry"]
