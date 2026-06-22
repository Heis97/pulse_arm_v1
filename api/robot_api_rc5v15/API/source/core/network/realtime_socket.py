from __future__ import annotations

import logging
import struct

from api.robot_api_rc5v15.API.source.core.network.socket_wrapper import SocketWrapper
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    RealtimeCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    CTRLR_CMD_DATA_PACK_UNPACK_FORMAT,
    EMPTY_BYTES,
    REALTIME_API_PORT,
)
from api.robot_api_rc5v15.API.source.models.protocol import CommandRegistry


class RealtimeController(LoggerMixin):
    """
    Класс-драйвер для отправки управляющих команд на робота в режиме управления
    в реальном времени.
    """

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
        self._ip = ip
        self._timeout = timeout
        self._set_logger(logger)
        self._socket = SocketWrapper(
            ip=self._ip,
            port=REALTIME_API_PORT,
            connect_timeout=self._timeout,
        )
        self._is_connected = False

    def connect(self) -> bool:
        """
        Подключиться к Real-time сокету робота.

        Returns:
            bool: True если подключение произведено успешно.
        """
        self._write_log("debug", "Connecting realtime socket")
        try:
            if self._socket.connect():
                self._is_connected = True
                return True
            self._is_connected = False
        except Exception as e:
            self._write_log(
                "warning", f"Couldn't connect realtime socket, error: {e}"
            )
            self._is_connected = False
        return False

    def is_connected(self) -> bool:
        """
        Получить статус подключения к роботу.

        Returns:
            bool: True если подключение активно.
        """
        return self._is_connected and self._socket.is_connected()

    def disconnect(self):
        """
        Отключиться от управляющего Real-time сокета робота.
        """
        if self.is_connected():
            self._write_log("debug", "Disconnecting realtime socket")
            self._socket.disconnect()
        self._is_connected = False

    def __del__(self):
        if self.is_connected():
            self.disconnect()

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
        return True

    def send_command(self, command: RealtimeCommands, *args) -> bool:
        """
        Отправить команду по имени через CommandRegistry.
        Автоматически упаковывает payload через CommandSpec.pack().
        """
        spec = CommandRegistry.get(command)
        payload = spec.pack(*args)
        return self.send(spec.id, payload)
