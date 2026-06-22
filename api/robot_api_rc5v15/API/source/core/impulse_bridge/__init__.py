import sys
from typing import TYPE_CHECKING

if TYPE_CHECKING or "pdoc" in sys.modules:
    from .impulse_compat import (
        add_impulse_vars,
        impulse_vars,
        load_impulse_vars,
        save_impulse_vars,
        send_error_to_impulse,
    )
    from .vars_proxy import ImpulseVarsProxy

__all__ = [
    "load_impulse_vars",
    "save_impulse_vars",
    "add_impulse_vars",
    "impulse_vars",
    "ImpulseVarsProxy",
    "send_error_to_impulse",
]


def __getattr__(name: str):  # noqa: C901
    if name == "ImpulseVarsProxy":
        from .vars_proxy import ImpulseVarsProxy

        return ImpulseVarsProxy

    if name == "add_impulse_vars":
        from .impulse_compat import add_impulse_vars

        return add_impulse_vars

    if name == "impulse_vars":
        from .impulse_compat import impulse_vars

        return impulse_vars

    if name == "load_impulse_vars":
        from .impulse_compat import load_impulse_vars

        return load_impulse_vars

    if name == "save_impulse_vars":
        from .impulse_compat import save_impulse_vars

        return save_impulse_vars

    if name == "send_error_to_impulse":
        from .impulse_compat import send_error_to_impulse

        return send_error_to_impulse
