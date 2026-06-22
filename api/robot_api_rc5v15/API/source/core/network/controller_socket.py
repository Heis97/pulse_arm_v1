from __future__ import annotations

import logging
import struct
import threading
from typing import Any

from api.robot_api_rc5v15.API.source.core.connection_state import (
    is_in_impulse_compat_mode,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    CorruptedPackageError,
)
from api.robot_api_rc5v15.API.source.core.network.socket_wrapper import (
    SocketWrapper,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    CMD_PORT,
    CTRLR_CMD_DATA_PACK_UNPACK_FORMAT,
    EMPTY_BYTES,
    IMPULSE_PORT,
)
from api.robot_api_rc5v15.API.source.models.protocol import CommandRegistry


class Controller(LoggerMixin):
    """
    Класс-драйвер для отправки/приема управляющих команд на/с робота.
    """

    _COMMAND_RECEIVE_TIMEOUT: float = 1  # sec
    _MAX_BUFFERED_PER_COMMAND: int = 16
    _HEADER_SIZE: int = struct.calcsize(CTRLR_CMD_DATA_PACK_UNPACK_FORMAT)

    def __init__(
        self, ip: str, timeout: int, logger: logging.Logger | None = None
    ):
        """
        Создать новый объект класса.

        Args:
            ip (str): IPv4 адрес робота;
            timeout (float): максимальное время ожидания подключения к роботу;
            logger (optional, logging.Logger): если задан, то логи будут писаться.
        """
        self._ip: str = ip
        self._timeout: float = timeout
        self._set_logger(logger)

        if is_in_impulse_compat_mode():
            port = IMPULSE_PORT
            self._write_log(
                "debug",
                f"API running in Impulse compatible mode, controller port was changed from {CMD_PORT} to {IMPULSE_PORT}",
            )
        else:
            port = CMD_PORT

        self._socket: SocketWrapper = SocketWrapper(
            ip=self._ip,
            port=port,
            connect_timeout=self._timeout,
            receive_timeout=self._COMMAND_RECEIVE_TIMEOUT,
        )
        self._is_connected: bool = False

        self._message_buffer: dict[int, list[bytes]] = {}
        self._buffer_lock = threading.Lock()

    def connect(self) -> bool:
        """
        Подключиться к управляющему сокету робота.

        Returns:
            bool: True если подключение произведено успешно.
        """
        self._write_log("debug", "Connecting control socket")
        if self._socket.connect():
            self._is_connected = True
            return True
        self._is_connected = False
        return False

    def is_connected(self):
        """
        Получить статус подключения к роботу.

        Returns:
            bool: True если подключение активно.
        """
        return self._is_connected and self._socket.is_connected()

    def disconnect(self):
        """
        Отключиться от управляющего сокета робота.
        """
        if self.is_connected():
            self._write_log("debug", "Disconnecting control socket")
            self._socket.disconnect()
        self._is_connected = False

    def __del__(self):
        if self.is_connected():
            self.disconnect()

    def _add_to_buffer(self, cmd_type: int, payload: bytes) -> None:
        """Добавляет сообщение в буфер, удаляя самое старое при переполнении."""
        with self._buffer_lock:
            queue = self._message_buffer.setdefault(cmd_type, [])
            if len(queue) >= self._MAX_BUFFERED_PER_COMMAND:
                queue.pop(0)  # Drop oldest
            queue.append(payload)
            self._write_log(
                "warning",
                f"Buffered unexpected command type: {cmd_type} (buffer size: {len(queue)})",
            )

    def receive(
        self, command_type: int, struct_format: str
    ) -> tuple[Any, ...]:
        """
        Принять команду с робота.

        Args:
            command_type (int): ожидаемый тип команды;
            struct_format (str): ожидаемый формат данных.
        """
        with self._buffer_lock:
            if command_type in self._message_buffer:
                payload = self._message_buffer[command_type].pop(0)
                if not self._message_buffer[command_type]:
                    del self._message_buffer[command_type]
                return struct.unpack(struct_format, payload)

        while True:
            cmd_length_data = self._socket.receive(self._HEADER_SIZE)
            if cmd_length_data is None:
                return ()  # Таймаут чтения заголовка

            cmd_length = struct.unpack(
                CTRLR_CMD_DATA_PACK_UNPACK_FORMAT, cmd_length_data
            )
            if cmd_length[0] < self._HEADER_SIZE:
                raise CorruptedPackageError(
                    "Received data length less than expected"
                )

            cmd_type_data = self._socket.receive(self._HEADER_SIZE)
            if cmd_type_data is None:
                raise CorruptedPackageError("Failed to receive command type")

            received_cmd_type = struct.unpack(
                CTRLR_CMD_DATA_PACK_UNPACK_FORMAT, cmd_type_data
            )

            self._write_log(
                "debug",
                f"Received response command-type: {received_cmd_type[0]}",
            )

            cmd_data = self._socket.receive(cmd_length[0] - self._HEADER_SIZE)
            if cmd_data is None:
                raise CorruptedPackageError(
                    "Failed to receive command body data"
                )
            try:
                if received_cmd_type[0] == command_type:
                    return struct.unpack(struct_format, cmd_data)
                else:
                    # Команда не ожидалась -> сохраняем и продолжаем чтение
                    self._add_to_buffer(received_cmd_type[0], cmd_data)
            except Exception:
                self._write_log(
                    "error",
                    f"Received data len: {len(cmd_data)} "
                    f"(required {struct.calcsize(struct_format)}) for "
                    f"command type {received_cmd_type[0]}",
                )
                raise

    def send(self, command_type: int, payload: bytes = EMPTY_BYTES) -> bool:
        """
        Отправить команду роботу.

        Args:
            command_type (int): тип отправляемой команды;
            payload (bytes): тело отправляемой команды.
        """
        frame = struct.pack(
            CTRLR_CMD_DATA_PACK_UNPACK_FORMAT,
            len(payload) + self._HEADER_SIZE,
        ) + struct.pack(CTRLR_CMD_DATA_PACK_UNPACK_FORMAT, command_type)
        if payload:
            frame += payload
        self._socket.send(frame)
        self._write_log("debug", f"Sent command-type: {command_type}")
        return True

    def send_command(
        self, command: ControllerCommands, *args, variant: str = ""
    ) -> bool:
        """
        Отправить команду по имени через CommandRegistry.
        Автоматически упаковывает payload через CommandSpec.pack().
        """
        spec = CommandRegistry.get(command)
        payload = spec.pack(*args, variant=variant)
        return self.send(spec.id, payload)

    def receive_command(self, command: ControllerCommands) -> tuple[Any, ...]:
        """
        Принять ответ от робота по имени команды.
        Автоматически распаковывает через CommandSpec.unpack_fmt.
        """
        spec = CommandRegistry.get(command)
        return self.receive(spec.id, spec.unpack_fmt)

    def request(
        self, command: ControllerCommands, *args, variant: str = ""
    ) -> tuple[Any, ...]:
        """
        Удобный паттерн: отправить команду и сразу дождаться ответа.
        """
        self.send_command(command, *args, variant=variant)
        return self.receive_command(command)
