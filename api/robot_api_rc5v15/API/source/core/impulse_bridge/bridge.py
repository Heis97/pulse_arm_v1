"""
Основной файл, реализующий функционал режима совместимости с ПО Импульс.
"""

from __future__ import annotations

import atexit
import inspect
import json
import os
import select
import sys
import threading
import time
import traceback
import types
from pathlib import Path
from typing import Any, TextIO


from api.robot_api_rc5v15.API.source.core.connection_state import (
    is_in_impulse_compat_mode,
)
from api.robot_api_rc5v15.API.source.core.exceptions.bridge_error import (
    BridgeError,
    PipeError,
    PipeNameError,
    PipeTimeoutError,
)

from .structures import Message, MsgFields, MsgType
from .tools import get_short_session_id

_MODULE_DIR = Path(__file__).parent.resolve()

_MESSAGES_SEPARATOR: str = "\r"
_MESSAGES_ENCODING: str = "utf-8"
_RECEIVE_TIMEOUT: float = 0.5  # sec
_DEBUG: bool = True

_read_pipe: TextIO | None = None
_write_pipe: TextIO | None = None

_session_id: str = get_short_session_id()
_last_exception: str | None = None


_initial_excepthook = sys.excepthook


def _custom_excepthook(exc_type, exc_value, exc_traceback):
    """Сохраняет последний traceback для использования в atexit."""
    global _last_exception
    _last_exception = "".join(
        traceback.format_exception(exc_type, exc_value, exc_traceback)
    )
    _initial_excepthook(exc_type, exc_value, exc_traceback)


# Глобальное событие завершения
_shutdown_event = threading.Event()


def _interruptible_sleep(seconds: float) -> None:
    """Прерываемая версия time.sleep, совместимая с режимом 'Импульс'."""
    if seconds <= 0:
        return
    # Ждём либо таймаут, либо сигнал завершения
    _shutdown_event.wait(timeout=seconds)


class ImpulseBridge:
    _impulse_var_names: set[str] = set()
    _is_first_load: bool = True
    _read_buffer: str = ""

    @classmethod
    def load_vars(
        cls,
        vars_names: list[str] | None = None,
        target: Any | None = None,
    ) -> set[str]:
        """Загружает переменные из Impulse. Если target=None, обновляет globals вызывающего фрейма."""
        if vars_names is None:
            vars_names = []

        if cls._is_first_load:
            response = cls._request(
                msg_type=MsgType.DATA_INIT, vars_names=vars_names
            )
            cls._is_first_load = False
        else:
            response = cls._request(
                msg_type=MsgType.DATA_LOAD, vars_names=vars_names
            )
        if response:
            cls.print(f"DEBUG: Received message {response}")

            if response.imports and isinstance(response.imports, list):
                caller_globals = cls.get_frame_params()
                cls._load_imports(response.imports, caller_globals)

            if cls._is_first_load:
                return set()

            valid_vars = {
                k: v
                for k, v in (response.variables or {}).items()
                if isinstance(k, str)
                and k.isidentifier()
                and not k.startswith("_")
            }
            if vars_names:
                valid_vars = {
                    k: v for k, v in valid_vars.items() if k in vars_names
                }

            if target is not None:
                cls._update_target(target, valid_vars)
            else:
                frame_globals = cls.get_frame_params()
                if frame_globals is not None:
                    frame_globals.update(valid_vars)

            cls._impulse_var_names |= set(valid_vars.keys())
            cls.print(f"DEBUG: loaded {list(valid_vars.keys())}")

        return cls._impulse_var_names.copy()

    @classmethod
    def save_vars(
        cls,
        vars_names: list[str] | None = None,
        target: Any | None = None,
    ):
        """Сохраняет переменные в Impulse. Если target=None, считывает из globals вызывающего фрейма."""
        if vars_names is None:
            vars_names = []

        source_dict = None
        if target is not None:
            source_dict = cls._target_to_dict(target)
        else:
            frame_globals = cls.get_frame_params()
            source_dict = frame_globals

        cls._request(
            msg_type=MsgType.DATA_SAVE,
            vars_names=vars_names,
            globals_dict=source_dict,
        )

    @classmethod
    def add_impulse_var(cls, var_name: str):
        """
        Добавляет новую переменную в набор Impulse переменных.
        """
        if cls._impulse_var_names is None:
            cls._impulse_var_names = set()
        cls._impulse_var_names.add(var_name)

    @classmethod
    def get_impulse_var_names(cls) -> set[str]:
        """
        Возвращает копию набора имен переменных Impulse.
        """
        return (
            cls._impulse_var_names.copy() if cls._impulse_var_names else set()
        )

    @classmethod
    def send_error(cls, message: str):
        cls._request(msg_type=MsgType.DATA_ERR, extra_message=message)

    @classmethod
    def _load_imports(
        cls,
        imports: list[str],
        caller_globals: dict[str, Any] | None = None,
    ):
        """
        Служебная функция для автоматического импорта дополнительных скриптов.
        """
        if not isinstance(imports, list):
            return

        imported_modules = []
        for path in imports:
            if not isinstance(path, str) or not path.endswith(".py"):
                cls.print(f"WARNING: skipping import of '{path}'")
                continue

            module_name = path[:-3].replace("/", ".").replace("\\", ".")
            cls._import_module(module_name, path, caller_globals)
            imported_modules.append(module_name)

        cls.print(
            f"DEBUG: successfully imported {len(imported_modules)} modules"
        )

    @classmethod
    def _import_module(
        cls,
        module_name: str,
        path: str,
        caller_globals: dict[str, Any] | None,
    ) -> bool:
        try:
            import importlib.util

            spec = importlib.util.spec_from_file_location(module_name, path)
            if spec is None or spec.loader is None:
                cls.print(f"ERROR: cannot find '{path}'")
                return False

            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls.print(f"DEBUG: loaded module '{module_name}' from '{path}'")

            exported = 0
            for name, value in vars(module).items():
                if name.startswith("__") and name.endswith("__"):
                    continue
                if not name.startswith("_") and caller_globals is not None:
                    caller_globals[name] = value
                    exported += 1

                # Тип для отладки
                type_name = type(value).__name__
                cls.print(
                    f"DEBUG: imported var '{name}' with type '{type_name}'"
                )

            cls.print(f"DEBUG: exported {exported} items from '{module_name}'")
            return True
        except Exception as e:
            cls.print(f"ERROR: import '{path}': {e}")
            return False

    @staticmethod
    def get_frame_params() -> dict[str, Any] | None:
        """
        Получает параметры фрейма вызывающего кода вне текущего модуля.
        """
        frame_type = ImpulseBridge._get_caller_frame()
        return frame_type.f_globals if frame_type else None

    @staticmethod
    def is_json_serializable(obj: Any) -> bool:
        if obj is None:
            return True
        if isinstance(obj, (int, float, str, bool)):
            return True
        if isinstance(obj, (list, dict, tuple)):
            return True
        return False

    @staticmethod
    def _get_caller_frame() -> types.FrameType | None:
        """Находит первый фрейм вызывающего кода вне текущего модуля."""
        frame = None
        try:
            frame = inspect.currentframe()
            # Пропускаем внутренние вызовы (3 уровня вверх)
            for _ in range(3):
                if frame is None:
                    return None
                frame = frame.f_back

            while frame:
                frame_path = Path(frame.f_code.co_filename).resolve()
                if not frame_path.is_relative_to(_MODULE_DIR):
                    return frame
                frame = frame.f_back
            return None
        finally:
            del frame  # Предотвращаем reference cycles

    @staticmethod
    def _update_target(target: Any, data: dict[str, Any]) -> None:
        """Универсальное обновление target: dict, proxy или произвольный объект."""
        if hasattr(target, "update"):
            target.update(data)
        elif hasattr(target, "__dict__"):
            target.__dict__.update(data)
        else:
            for k, v in data.items():
                setattr(target, k, v)

    @staticmethod
    def _target_to_dict(target: Any) -> dict[str, Any] | None:
        """Приводит target к dict для сериализации."""
        if isinstance(target, dict):
            return target
        if hasattr(target, "to_dict"):
            return target.to_dict()
        if hasattr(target, "__dict__"):
            return target.__dict__
        try:
            return dict(target)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _request(
        cls,
        msg_type: MsgType,
        vars_names: list[str] = [],
        globals_dict: dict[str, Any] | None = None,
        locals_dict: dict[str, Any] | None = None,
        extra_message: str | None = None,
    ) -> Message | None:
        response = None
        if msg_type == MsgType.DATA_INIT:
            if not cls._is_first_load:
                raise BridgeError(
                    "Trying to call 'DATA_INIT' request more then one time"
                )
            response = cls._receive_message(msg_type, timeout=_RECEIVE_TIMEOUT)

        elif msg_type == MsgType.DATA_LOAD:
            cls._send_message(
                msg_type,
                vars_names=vars_names,
                globals_dict=globals_dict,
                locals_dict=locals_dict,
            )
            response = cls._receive_message(msg_type, timeout=_RECEIVE_TIMEOUT)

        elif msg_type == MsgType.DATA_SAVE:
            cls._send_message(
                msg_type,
                vars_names=vars_names,
                globals_dict=globals_dict,
                locals_dict=locals_dict,
            )

        elif msg_type == MsgType.DATA_ERR:
            cls._send_message(
                msg_type,
                extra_message=extra_message,
            )
        else:
            raise BridgeError(f"Unknown message type '{msg_type}'")

        return response

    @classmethod
    def _receive_message(
        cls, msg_type: MsgType, timeout: float = 0.5
    ) -> Message | None:
        cls.print(f"DEBUG: Receiving message with type {msg_type}")

        json_str = cls._read_from_pipe(timeout=timeout)

        if not json_str:
            return None

        data: dict[str, Any] = json.loads(json_str)

        if MsgFields.fld_type.value not in data:
            raise BridgeError(
                f"No field '{MsgFields.fld_type.value}' in received message"
            )
        received_msg_type = data[MsgFields.fld_type.value]
        if msg_type != received_msg_type:
            raise BridgeError(
                f"Received message type '{received_msg_type}', "
                f"expected '{msg_type}'"
            )

        message = Message(msg_type=data[MsgFields.fld_type.value])
        message.variables = data.get(MsgFields.fld_vars.value, {})
        message.imports = data.get(MsgFields.fld_imports.value, [])
        return message

    @classmethod
    def _send_message(  # noqa: C901
        cls,
        msg_type: MsgType,
        vars_names: list[str] = [],
        globals_dict: dict[str, Any] | None = None,
        locals_dict: dict[str, Any] | None = None,
        extra_message: str | None = None,
    ):
        cls.print(f"DEBUG: Sending message with type '{msg_type}'")

        if msg_type == MsgType.DATA_INIT:
            raise BridgeError(
                "Sending message with type 'DATA_INIT' is not permitted"
            )
        elif msg_type == MsgType.DATA_ERR:
            if not extra_message:
                cls.print(
                    f"DEBUG: Can't send message with type {MsgType.DATA_ERR} "
                    f"without {MsgFields.fld_message.name} field"
                )
                return
            message = Message(msg_type=msg_type, message=extra_message)
            cls.print(f"DEBUG: Sending message {message}")
            cls._write_to_pipe(message.to_json())
            return

        elif msg_type == MsgType.DATA_LOAD:
            message = Message(msg_type=msg_type)
            message.variables = {k: None for k in vars_names}
            cls.print(f"DEBUG: Sending message {message}")
            cls._write_to_pipe(message.to_json())
            return

        elif msg_type == MsgType.DATA_SAVE:
            impulse_names = cls.get_impulse_var_names()
            vars_to_save = {}

            if globals_dict:
                for name in impulse_names:
                    if vars_names and name not in vars_names:
                        continue
                    if name in globals_dict:
                        val = globals_dict[name]
                        if (
                            not name.startswith("_")
                            and name != "imports"
                            and cls.is_json_serializable(val)
                        ):
                            vars_to_save[name] = val

            msg = Message(msg_type=msg_type)
            msg.variables = vars_to_save
            cls.print(f"DEBUG: Sending message {msg}")
            cls._write_to_pipe(msg.to_json())
            return

        elif msg_type == MsgType.DATA_EXIT:
            message = Message(msg_type=msg_type)
            if extra_message:
                message.message = extra_message
            cls.print(f"DEBUG: Sending message {message}")
            cls._write_to_pipe(message.to_json())
            return

        else:
            raise BridgeError(f"Unknown message type '{msg_type}'")

    @classmethod
    def _read_from_pipe(cls, timeout: float | None = None) -> str:
        if _read_pipe is None:
            raise PipeNameError("No read pipe name provided")

        fd = _read_pipe.fileno()
        start_time = time.time()

        while _MESSAGES_SEPARATOR not in cls._read_buffer:
            if timeout is not None and timeout > 0:
                remaining = timeout - (time.time() - start_time)
                if remaining <= 0:
                    raise PipeTimeoutError(
                        f"Timeout ({timeout}s) while waiting for data"
                    )

                ready, _, _ = select.select([fd], [], [], remaining)
                if not ready:
                    raise PipeTimeoutError(
                        f"Timeout ({timeout}s) while waiting for data"
                    )

            chunk = os.read(fd, 1024)
            if not chunk:
                break  # EOF
            cls._read_buffer += chunk.decode(_MESSAGES_ENCODING)

        if _MESSAGES_SEPARATOR in cls._read_buffer:
            data, cls._read_buffer = cls._read_buffer.split(
                _MESSAGES_SEPARATOR, 1
            )
        else:
            data = ""

        cls.print(f"DEBUG: Received data from pipe: {data}")
        return data

    @classmethod
    def _write_to_pipe(cls, data: str):
        if _write_pipe is None:
            raise PipeNameError("No write pipe name provided")
        data = data + _MESSAGES_SEPARATOR
        cls.print(f"DEBUG: Writing data to pipe: {data}")

        _write_pipe.write(data)
        _write_pipe.flush()

    @classmethod
    def exit(cls):
        _shutdown_event.set()

        global _last_exception
        if _last_exception:
            cls.print(
                "CRITICAL: Program terminated due to unhandled exception:"
            )
            cls.print(_last_exception)

        cls.print("DEBUG: Sending exit message")
        try:
            cls._send_message(MsgType.DATA_EXIT, extra_message=_last_exception)
        except Exception as e:
            cls.print(f"WARNING: Failed to send exit message, error: {e}")
            pass
        cls.print("DEBUG: Closing pipes")
        if _read_pipe:
            try:
                _read_pipe.close()
            except Exception:
                pass
        if _write_pipe:
            try:
                _write_pipe.close()
            except Exception:
                pass

    @staticmethod
    def print(*messages: str):
        if _DEBUG:
            print(f"[{_session_id}]", *messages)


if is_in_impulse_compat_mode():
    # Логика однократного открытия FIFO
    try:
        read_pipe = sys.argv[1]
        write_pipe = sys.argv[2]
        ImpulseBridge.print(
            f"DEBUG: Script arguments: read_pipe={read_pipe}, write_pipe={write_pipe}"
        )
    except Exception as e:
        raise BridgeError(f"Wrong script params number, error: {e}")

    if not os.path.exists(read_pipe):
        raise PipeError(f"Pipe '{read_pipe}' does not exists")
    if not os.path.exists(write_pipe):
        raise PipeError(f"Pipe '{write_pipe}' does not exists")
    try:
        if _read_pipe is None:
            _read_pipe = open(read_pipe, encoding=_MESSAGES_ENCODING)
        if _write_pipe is None:
            _write_pipe = open(write_pipe, "w", encoding=_MESSAGES_ENCODING)
    except Exception as e:
        raise PipeNameError(f"Failed to get file descriptors: {e}")

    try:
        os.setresuid(1000, 1000, 1000)
        ImpulseBridge.print("DEBUG: UID changed to 1000")
    except Exception as e:
        ImpulseBridge.print(f"ERROR: failed to change UID to 1000, error: {e}")

    ImpulseBridge.print(f"DEBUG: message encoding: {_MESSAGES_ENCODING}")
    ImpulseBridge.print(
        f"DEBUG: message separator: {repr(_MESSAGES_SEPARATOR)}"
    )
    ImpulseBridge.print(f"DEBUG: receive timeout {_RECEIVE_TIMEOUT} sec")

    sys.excepthook = _custom_excepthook
    atexit.register(ImpulseBridge.exit)

    # Подменяем time.sleep на прерываемый
    time.sleep = _interruptible_sleep
