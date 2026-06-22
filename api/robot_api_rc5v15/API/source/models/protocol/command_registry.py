from __future__ import annotations

import dataclasses
import struct
from dataclasses import dataclass
from typing import Any, ClassVar, Union

from api.robot_api_rc5v15.API.source.core.exceptions.base_api_error import ApiError
from api.robot_api_rc5v15.API.source.features.tools import flatten_recursive

from ..classes.enum_classes.controller_commands import (
    ControllerCommands,
    RealtimeCommands,
)
from ..constants import EMPTY_BYTES

Commands = Union[ControllerCommands, RealtimeCommands]


@dataclass(frozen=True)
class CommandSpec:
    command: Commands
    pack_fmt: str = ""
    unpack_fmt: str = ""
    variant_pack_formats: dict[str, str] | None = None

    @property
    def name(self) -> str:
        return self.command.name

    @property
    def id(self) -> int:
        return self.command.value

    def pack(self, *args: Any, variant: str = "") -> bytes:
        fmt = self.pack_fmt
        if self.variant_pack_formats and variant:
            fmt = self.variant_pack_formats.get(variant)
            if not fmt:
                raise ValueError(
                    f"Unknown variant '{variant}' for command {self.name}"
                )
        if not fmt:
            return EMPTY_BYTES

        flat_args = self._flatten_payload(args)
        return struct.pack(fmt, *flat_args)

    def unpack(self, raw: bytes) -> tuple:
        if not self.unpack_fmt:
            return ()
        return struct.unpack(self.unpack_fmt, raw)

    @staticmethod
    def _flatten_payload(args: tuple[Any, ...]) -> tuple[Any, ...]:
        flat = []
        for arg in args:
            if dataclasses.is_dataclass(arg) and not isinstance(arg, type):
                flat.extend(flatten_recursive(arg))
            elif isinstance(arg, (list, tuple)):
                flat.extend(arg)
            else:
                flat.append(arg)
        return tuple(flat)


class CommandRegistry:
    """Единый реестр всех команд протокола."""

    _by_name: ClassVar[dict[str, CommandSpec]] = {}

    @classmethod
    def register(cls, spec: CommandSpec) -> None:
        if spec.name in cls._by_name:
            raise ApiError(
                f"Can't register command {spec.name} ({spec.id}) - "
                f"command already registered"
            )
        cls._by_name[spec.name] = spec

    @classmethod
    def get(cls, command: Commands) -> CommandSpec:
        if command.name not in cls._by_name:
            raise ApiError(
                f"Command {command.name} ({command.value}) is not in "
                f"commands registry. In most cases, this is an "
                f"internal API error. Please contact your integrator."
            )
        return cls._by_name[command.name]

    @classmethod
    def pack(cls, command: ControllerCommands, *values: Any) -> bytes:
        return cls.get(command).pack(*values)

    @classmethod
    def unpack(
        cls, command: ControllerCommands, raw: bytes
    ) -> tuple[Any, ...]:
        return cls.get(command).unpack(raw)
