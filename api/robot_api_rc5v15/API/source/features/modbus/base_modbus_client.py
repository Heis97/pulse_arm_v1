from __future__ import annotations

import logging

from pymodbus.client.base import ModbusBaseSyncClient
from pymodbus.transport import CommParams

from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import ResponseTimeoutError
from api.robot_api_rc5v15.API.source.core.network.rs_485_protocol import RS485Protocol
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.rs_485_structures import (
    Rs485ReturnCodes,
)

try:
    from pymodbus import FramerType  # type: ignore
except ImportError:
    # Fallback для Python 3.8
    from pymodbus.framer.rtu_framer import ModbusRtuFramer  # type: ignore
    from pymodbus.framer.socket_framer import (  # type: ignore
        ModbusSocketFramer,
    )

    class FramerType:
        SOCKET = ModbusSocketFramer
        RTU = ModbusRtuFramer


class BaseModbusRS485Client(ModbusBaseSyncClient, LoggerMixin):
    """Базовый синхронный Modbus RTU-клиент для работы через RS-485 интерфейс.

    Инкапсулирует интеграцию стандартного Modbus-стека с пользовательским
    RS-485-транспортом (реализованным в `RS485Protocol`). Класс не управляет
    соединением напрямую — вместо этого делегирует операции чтения/записи
    переданному экземпляру `RS485Protocol`.

    Предназначен исключительно для наследования. Конкретные реализации
    (например, `WristModbusRS485Client`) определяют способ создания
    и управления транспортным слоем.

    Attributes:
        host (str): IP-адрес или хостнейм устройства, к которому подключён RS-485-порт.
    """

    _rs485: RS485Protocol

    def __init__(
        self,
        rs_485_protocol: RS485Protocol,
        logger: logging.Logger | None = None,
    ):
        """Инициализирует базовый Modbus RTU-клиент поверх заданного RS-485 транспорта.

        Args:
            rs_485_protocol (RS485Protocol): Экземпляр транспорта, реализующий
                низкоуровневый обмен данными по RS-485.
            logger (Optional[logging.Logger]): Логгер для записи диагностических
                сообщений. Если не указан, логирование отключено.

        Notes:
            - Клиент настроен на работу в режиме **Modbus RTU**.
            - Параметры соединения (таймауты, количество повторов) управляются
                как на уровне Modbus-стека, так и на уровне переданного транспорта.
        """
        super().__init__(
            framer=FramerType.RTU,  # type: ignore
            retries=3,
            comm_params=CommParams(),
            trace_packet=None,
            trace_pdu=None,
            trace_connect=None,
        )
        self._rs485 = rs_485_protocol
        self._set_logger(logger)

        self._is_connected: bool = False

    def is_connected(self) -> bool:
        """Проверяет, активно ли соединение.

        Returns:
            bool:
                - `True` — подключение активно;
                - `False` — подключение разорвано.

        Examples:
            >>> if client.is_connected():
            ...     regs = client.read_holding_registers(0x01, 2)
            ... else:
            ...     print("Требуется подключение")
        """
        return self._is_connected and self._rs485.is_connected()

    def connect(self) -> bool:
        if self._is_connected:
            return True

        self._write_log(
            "info", f"Connecting Modbus RS-485 client to {self._rs485.host}"
        )
        if not self._rs485.connect():
            return False
        response = self._rs485.reset()
        self._rs485.clear_buffer()
        if response is None:
            raise ResponseTimeoutError(
                "Haven't received response to initialization"
            )

        if response.is_ok:
            self._is_connected = True
            return True

        if response.return_code == Rs485ReturnCodes.no_wrist_board:
            self._write_log("error", "No wrist board found")
            return False

        if response.return_code == Rs485ReturnCodes.wrong_wrist_mode:
            self._write_log(
                "error", "Incorrect mode of wrist board - should be RS485"
            )
            return False
        self._write_log(
            "error",
            f"Incorrect return code of response: {response.return_code.name}",
        )
        return False

    def close(self):
        self._write_log(
            "info",
            f"Disconnecting Modbus RS-485 client from {self._rs485.host}",
        )
        self._is_connected = False
        self._rs485.disconnect()

    def send(self, request: bytes, addr: tuple | None = None) -> int:
        if not self._is_connected:
            return 0

        self._write_log(
            "debug",
            f"Sending Modbus RS-485 client request to {self._rs485.host}: {request}",
        )

        response = self._rs485.write(request)
        if response is None:
            raise ResponseTimeoutError("Failed to get response sending data")
        if response.is_ok:
            return len(request)

        self._write_log(
            "error",
            f"Failed to send Modbus data, return code: {response.return_code.name}",
        )
        return 0

    def recv(self, size: int | None = None) -> bytes:
        if not self._is_connected:
            return b""

        self._write_log(
            "debug",
            f"Receiving Modbus RS-485 client data from {self._rs485.host}",
        )

        response = self._rs485.read(size)
        if response is None:
            raise ResponseTimeoutError("Failed to get response receiving data")

        if not response.is_ok:
            self._write_log(
                "error",
                f"Failed to receive Modbus data, return code: {response.return_code.name}",
            )
            return response.raw_data

        self._write_log(
            "debug",
            f"Received Modbus RS-485 client data: {bytes(response.raw_data)}",
        )
        return bytes(response.raw_data)

    def __str__(self) -> str:
        status = "connected" if self.is_connected() else "disconnected"
        class_name = self.__class__.__name__
        return f"{class_name}(host={self._rs485.host}, status={status})"

    def __del__(self):
        try:
            if self.is_connected():
                self.close()
        except Exception:
            pass
