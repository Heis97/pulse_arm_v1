from __future__ import annotations

import functools
import inspect

from api.robot_api_rc5v15.API.source.ap_interface.motion import CoordinateSystem


def transforms_coordinates(
    system_param: str | None = None,
):
    """
    Декоратор для автоматического преобразования координат с помощью
    контекстного менеджера CoordinateSystem.in_frame.

    Args:
        system_param: Имя параметра, задающего систему координат в методе.
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            coord_system: CoordinateSystem | None = getattr(
                CoordinateSystem._local, "active_context", None
            )
            if coord_system is None:
                return func(self, *args, **kwargs)

            sig = inspect.signature(func)
            bound_args = sig.bind(self, *args, **kwargs)
            bound_args.apply_defaults()

            if (
                system_param is not None
                and system_param in bound_args.arguments
                and bound_args.arguments[system_param] is None
            ):
                bound_args.arguments[system_param] = coord_system

            return func(*bound_args.args, **bound_args.kwargs)

        return wrapper

    return decorator
