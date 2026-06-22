from __future__ import annotations

import logging

from api.robot_api_rc5v15.API.source.core.network.rs_485_protocol import RS485Protocol
from api.robot_api_rc5v15.API.source.core.network.socket_wrapper import SocketWrapper
from api.robot_api_rc5v15.API.source.models.constants import WRIST_RS_485_PORT


class WristRS485(RS485Protocol):
    """Управление встроенным RS-485-интерфейсом платы запястья робота.

    Предоставляет serial-like API (`write`, `read`, `query`) для обмена данными
    с внешними устройствами, подключёнными к RS-485-порту запястья.

    Все операции выполняются через контроллер робота, который управляет
    встроенным RS-485-адаптером. Класс инкапсулирует создание соединения
    и настройку таймаутов, характерных для этого конкретного порта.

    Attributes:
        host (str): IP-адрес контроллера робота, к которому подключена плата запястья.
    """

    _COMMAND_RECEIVE_TIMEOUT: float = 0.5  # sec

    def __init__(
        self,
        host: str,
        timeout: float = 1,
        use_buffer: bool = True,
        logger: logging.Logger | None = None,
    ):
        """Инициализирует интерфейс для управления портом RS-485 на платы запястья.

        Args:
            host (str): IP-адрес контроллера робота.
            timeout (float): Таймаут подключения к контроллеру (в секундах).
            use_buffer (bool): Включить буферизацию "лишних" ответов от устройства.
                По умолчанию `True` — рекомендуется для большинства сценариев.
            logger (Optional[logging.Logger]): Экземпляр логгера для записи
                отладочной информации. Если не указан, логирование отключено.

        Examples:
            >>> rs485 = WristRS485(host="192.168.1.10")
            >>> with rs485.connected():
            ...     rs485.write(b"MEAS?\\r\\n")
            ...     data = rs485.read()
        """
        socket: SocketWrapper = SocketWrapper(
            ip=host,
            port=WRIST_RS_485_PORT,
            connect_timeout=timeout,
            receive_timeout=self._COMMAND_RECEIVE_TIMEOUT,
        )
        super().__init__(socket=socket, use_buffer=use_buffer, logger=logger)
