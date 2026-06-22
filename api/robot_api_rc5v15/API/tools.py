"""
Модуль с инструментами, предоставляемыми пакетом API.

> from API import tools
> from API.tools import sleep
"""

import sys
from typing import TYPE_CHECKING

# Для инициализации Impulse bridge (не убирать!)
from .source.core.impulse_bridge.bridge import (
    ImpulseBridge,  # noqa: F401
)

# Для отображения RobotApi в pdoc
if TYPE_CHECKING or "pdoc" in sys.modules:
    from .source.core.impulse_bridge import (
        ImpulseVarsProxy,
        impulse_vars,
        load_impulse_vars,
        save_impulse_vars,
        send_error_to_impulse,
    )
    from .source.features.additional_tools import PeriodicPublisher
    from .source.features.additional_tools.g_code_executor import (
        GCodeExecutor,
        gcode_val_to_meters,
    )
    from .source.features.additional_tools.tcp_calibrator import calibrate_tcp
    from .source.features.tools import sleep

__all__ = [
    "sleep",
    "calibrate_tcp",
    # Impulse
    "load_impulse_vars",
    "save_impulse_vars",
    "send_error_to_impulse",
    "impulse_vars",
    "ImpulseVarsProxy",
    # Additional tools
    "PeriodicPublisher",
    "GCodeExecutor",
    "gcode_val_to_meters",
]


def __getattr__(name: str):  # noqa: C901
    if name == "sleep":
        from .source.features.tools import sleep

        return sleep

    if name == "calibrate_tcp":
        from .source.features.additional_tools.tcp_calibrator import (
            calibrate_tcp,
        )

        return calibrate_tcp

    if name == "ImpulseVarsProxy":
        from .source.core.impulse_bridge import ImpulseVarsProxy

        return ImpulseVarsProxy

    if name == "impulse_vars":
        from .source.core.impulse_bridge import impulse_vars

        return impulse_vars

    if name == "load_impulse_vars":
        from .source.core.impulse_bridge import load_impulse_vars

        return load_impulse_vars
    if name == "save_impulse_vars":
        from .source.core.impulse_bridge import save_impulse_vars

        return save_impulse_vars
    if name == "send_error_to_impulse":
        from .source.core.impulse_bridge import send_error_to_impulse

        return send_error_to_impulse

    if name == "PeriodicPublisher":
        from .source.features.additional_tools import PeriodicPublisher

        return PeriodicPublisher

    if name == "GCodeExecutor":
        from .source.features.additional_tools.g_code_executor import (
            GCodeExecutor,
        )

        return GCodeExecutor

    if name == "gcode_val_to_meters":
        from .source.features.additional_tools.g_code_executor import (
            gcode_val_to_meters,
        )

        return gcode_val_to_meters

    raise AttributeError(f"module 'API.tools' has no attribute '{name}'")
