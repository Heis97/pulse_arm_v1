from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Any


class MsgType(IntEnum):
    DATA_INIT = 0
    DATA_LOAD = 1
    DATA_SAVE = 2
    DATA_ERR = 3
    DATA_EXIT = 4


class MsgFields(str, Enum):
    fld_type = "type"
    fld_imports = "imports"
    fld_vars = "vars"
    fld_message = "message"


@dataclass
class Message:
    msg_type: MsgType
    variables: dict[str, Any] = None  # type: ignore
    imports: list[str] = None  # type: ignore
    message: str = ""

    def __post_init__(self):
        if self.variables is None:
            self.variables = {}
        if self.imports is None:
            self.imports = []

    def to_dict(self) -> dict[str, Any]:
        """Преобразует объект в словарь с ключами из MsgFields."""
        return {
            MsgFields.fld_type: self.msg_type,
            MsgFields.fld_vars: self.variables,
            MsgFields.fld_imports: self.imports,
            MsgFields.fld_message: self.message,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Message:
        """Создаёт объект Message из словаря с ключами из MsgFields."""
        raw_type = data.get(MsgFields.fld_type)
        if raw_type is None:
            raise ValueError(f"Missing required field: {MsgFields.fld_type}")

        if isinstance(raw_type, MsgType):
            msg_type = raw_type
        elif isinstance(raw_type, str):
            try:
                msg_type = MsgType(raw_type)
            except ValueError:
                raise ValueError(f"Invalid message type: {raw_type}")
        else:
            raise TypeError(
                f"Expected str or MsgType for 'type', got {type(raw_type)}"
            )

        variables = data.get(MsgFields.fld_vars, {})
        imports = data.get(MsgFields.fld_imports, [])
        message = data.get(MsgFields.fld_message, "")

        if not isinstance(variables, dict):
            raise TypeError(f"'{MsgFields.fld_vars}' must be a dict")
        if not isinstance(imports, list):
            raise TypeError(f"'{MsgFields.fld_imports}' must be a list")
        if not isinstance(message, str):
            raise TypeError(f"'{MsgFields.fld_message}' must be a string")

        return cls(
            msg_type=msg_type,
            variables=variables,
            imports=imports,
            message=message,
        )

    def to_json(self) -> str:
        """Метод для сериализации в JSON."""
        return json.dumps(
            self.to_dict(), separators=(",", ":"), ensure_ascii=False
        )

    @classmethod
    def from_json(cls, json_str: str) -> Message:
        """Создаёт объект из JSON-строки."""
        data = json.loads(json_str)
        return cls.from_dict(data)

    def __str__(self) -> str:
        parts = [f"{MsgFields.fld_type.value}={self.msg_type}"]
        if self.variables:
            parts.append(f"{MsgFields.fld_vars.value}={self.variables}")
        if self.imports:
            parts.append(f"{MsgFields.fld_imports.value}={self.imports}")
        if self.message:
            parts.append(f"{MsgFields.fld_message.value}={self.message}")
        return f"Message({', '.join(parts)})"
