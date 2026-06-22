from __future__ import annotations

import logging
import struct
import sys
from collections.abc import Generator
from contextlib import contextmanager

from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import ConnectionError
from api.robot_api_rc5v15.API.source.core.network.socket_wrapper import SocketWrapper
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.data_classes.rs_485_structures import (
    Response,
    Rs485Frame,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.rs_485_structures import (
    Rs485Commands,
    Rs485ReturnCodes,
)
from api.robot_api_rc5v15.API.source.models.constants import EMPTY_BYTES


class RS485Protocol(LoggerMixin):
    """Базовая реализация протокола обмена данными с RS-485-устройством через сокет.

    Инкапсулирует низкоуровневую логику взаимодействия с RS-485-шлюзом:
    формирование кадров, отправку команд, приём ответов, буферизацию "лишних"
    фреймов и повторные попытки при ошибках связи.

    Класс предназначен для наследования конкретными реализациями
    (например, WristRS485), которые определяют контекст использования
    и управление соединением.

    Attributes:
        host (str): IP-адрес удалённого RS-485 шлюза.
    """

    _REQUEST_RETRYING_ATTEMPTS: int = 3
    _MAX_RECEIVE_ATTEMPTS: int = 3

    _socket: SocketWrapper

    def __init__(
        self,
        socket: SocketWrapper,
        use_buffer: bool = True,
        logger: logging.Logger | None = None,
    ):
        """Инициализирует экземпляр протокола RS-485.

        Args:
            socket (SocketWrapper): Обёртка над сокетом, подключённым к RS-485 шлюзу.
            use_buffer (bool): Включить буферизацию "лишних" ответов от устройства.
                Если True, все полученные фреймы, не относящиеся к текущему запросу,
                сохраняются во внутреннем буфере и могут быть использованы последующими
                вызовами. По умолчанию True.
            logger (Optional[logging.Logger]): Экземпляр логгера для записи отладочной
                и диагностической информации. Если не указан, логирование отключено.

        Examples:
            >>> rs485 = RS485Protocol(host="192.168.1.50")
        """
        self._socket: SocketWrapper = socket
        self._use_buffer: bool = use_buffer
        self._set_logger(logger)

        self._is_connected: bool = False
        self._buffer: list[Rs485Frame] = []

    @property
    def host(self) -> str:
        """Возвращает IP-адрес RS-485 шлюза.

        Returns:
            str: Хост, к которому подключён сокет.
        """
        return self._socket.host

    def connect(self) -> bool:
        """Устанавливает соединение с RS-485 шлюзом через сокет.

        Метод пытается установить соединение с устройством, управляющим
        RS-485-шиной. Если соединение уже активно, метод немедленно возвращает
        `True` без повторного подключения.

        Returns:
            bool:
                - `True` — если соединение успешно установлено или уже активно;
                - `False` — если произошла ошибка при подключении (например,
                    таймаут, недоступность хоста).

        Examples:
            >>> if rs485.connect():
            ...     print("Подключение к RS-485 шлюзу установлено")
            ... else:
            ...     print("Не удалось подключиться")

            >>> # Повторный вызов безопасен и не приведёт к ошибке
            >>> rs485.connect()  # вернёт True, если уже подключён

        Notes:
            - Для управления жизненным циклом соединения рекомендуется использовать
              контекстный менеджер `with` с экземпляром класса или методом `connected`.
        """
        if self.is_connected():
            return True
        self._write_log(
            "debug", f"Connecting RS-485 socket to {self._socket.host}"
        )
        if self._socket.connect(is_blocking=False):
            self._is_connected = True
            return True
        self._is_connected = False
        return False

    @contextmanager
    def connected(self) -> Generator[RS485Protocol, None, None]:
        """Обеспечивает временное подключение к RS-485 шлюзу в рамках контекстного менеджера.

        Контекстный менеджер автоматически:
        - устанавливает соединение с RS-485 шлюзом, если оно ещё не активно;
        - гарантирует отключение после выхода из блока, даже при возникновении исключения.

        Если соединение уже было установлено до входа в контекст, оно **не разрывается**
        при выходе — менеджер сохраняет исходное состояние.

        Yields:
            RS485Protocol: Текущий экземпляр объекта, готовый к обмену данными по RS-485.

        Examples:
            >>> with rs485.connected() as port:
            ...     port.write(b"MEAS?\\n")
            ...     data = port.read()
            ...     # Соединение автоматически закроется здесь, если было открыто внутри

            >>> # Если соединение уже активно, оно сохраняется:
            >>> rs485.connect()
            >>> with rs485.connected():
            ...     rs485.get_status()
            >>> assert rs485.is_connected()  # всё ещё подключён

        Notes:
            - Безопасен для вложенного использования.
        """
        connected = self.is_connected()

        if not connected:
            self.connect()
        try:
            yield self
        finally:
            if not connected:
                self.disconnect()

    def is_connected(self):
        """Проверяет текущее состояние подключения к RS-485 шлюзу.

        Метод возвращает актуальный статус соединения.

        Returns:
            bool:
                - `True` — если соединение с RS-485 шлюзом активно и работоспособно;
                - `False` — если соединение не установлено, закрыто или потеряно.

        Examples:
            >>> if rs485.is_connected():
            ...     print("Готов к обмену данными")
            ... else:
            ...     print("Требуется подключение")

            >>> # Безопасная проверка перед отправкой команды
            >>> if not rs485.is_connected():
            ...     rs485.connect()
            >>> rs485.write(b"STATUS?\\n")
        """
        return self._is_connected and self._socket.is_connected()

    def reset(self) -> Response | None:
        """Выполняет сброс состояния RS-485 соединения на стороне сервиса робота.

        Метод отправляет команду сброса встроенному RS-485-сервису робота.
        Эта операция:
            - очищает внутренние буферы ошибок и данных,
            - сбрасывает флаги неисправностей,
            - приводит интерфейс RS-485 в исходное рабочее состояние.

        Используется для восстановления после ошибок связи, зависаний или неожиданного
        поведения шины.

        Returns:
            Optional[Response]:
                - Объект ``Response``, содержащий результат выполнения команды, если ответ получен;
                - ``None``, если произошёл таймаут, потеря соединения или команда не подтверждена.

        Examples:
            >>> with rs485.connected():
            ...     data = rs485.read()
            ...     # После ошибки чтения
            ...     if data is None:
            ...         rs485.reset()  # попытка восстановления
            ...         data = rs485.read()

        Notes:
            - Рекомендуется вызывать этот метод после подключения и при
                возникновении устойчивых ошибок обмена.
        """
        if not self.is_connected():
            raise ConnectionError(
                "Can't reset state - RS-485 is not connected"
            )
        frame = self._request(Rs485Commands.reset)
        if frame is None:
            return None
        return Response.from_frame(frame)

    def read(self, size: int | None = None) -> Response | None:
        """Считывает данные, полученные по RS-485 шине от подключённого устройства.

        Метод инициирует запрос на получение данных через встроенный RS-485-сервис робота,
        а затем собирает входящие фреймы до достижения указанного объёма или завершения
        передачи. Поддерживает как чтение всего доступного потока, так и ограниченное
        количество байт.

        Args:
            size (Optional[int]): Максимальное количество байт для чтения.
                - Если указано — метод вернёт не более этого числа байт;
                - Если `None` (по умолчанию) — читает все доступные данные до
                    завершения передачи.

        Returns:
            Optional[Response]:
                - Объект `Response`, содержащий прочитанные данные в атрибуте `raw_data`,
                    если операция успешна;
                - `None`, если произошла ошибка.

        Examples:
            >>> with rs485.connected():
            ...     # Прочитать все доступные данные
            ...     resp = rs485.read()
            ...     if resp:
            ...         print("Получено:", resp.raw_data)

            >>> # Прочитать ровно 10 байт (если доступны)
            >>> resp = rs485.read(size=10)
            >>> if resp and len(resp.raw_data) == 10:
            ...     print(resp.raw_data)

        Notes:
            - Метод **блокирующий**: ожидает поступления данных в течение
                внутреннего таймаута.
            - Если устройство не отправляет данные, вызов может завершиться с
                пустым результатом или `None`, в зависимости от состояния шины.
        """
        if not self.is_connected():
            raise ConnectionError("Can't read data - RS-485 is not connected")
        response = self._request(Rs485Commands.receive)
        if response is None:
            return None

        if response.return_code != Rs485ReturnCodes.ok:
            self._write_log(
                "error",
                f"Failed to receive RS-485 data, return code: {response.return_code.name}",
            )
            return Response.from_frame(response)

        result = bytearray()
        max_bytes = size if size is not None else sys.maxsize
        while len(result) < max_bytes:
            data_frame = self._receive_command(
                command=Rs485Commands.received_data,
                check_buffer=self._use_buffer,
            )

            if data_frame is None:
                break

            if not data_frame.data:
                break

            needed = max_bytes - len(result)
            result.extend(data_frame.data[:needed])
            if len(result) >= max_bytes:
                break

        self._write_log(
            "debug", f"Received RS-485 client data: {bytes(result)}"
        )
        response.data = bytes(result)
        return Response.from_frame(response)

    def write(self, payload: bytes) -> Response | None:
        """Отправляет данные на внешнее устройство через RS-485 шину.

        Метод передаёт указанный байтовый поток встроенному RS-485-сервису робота,
        который ретранслирует его в физическую RS-485-шину. Подключённое к шине устройство
        (датчик, контроллер, исполнительный механизм) получает эти данные напрямую.

        Args:
            payload (bytes): Байтовые данные для передачи по RS-485.
                Может содержать команды, параметры или произвольные данные в формате,
                поддерживаемом целевым устройством.

        Returns:
            Optional[Response]:
                - Объект `Response` с подтверждением от RS-485-сервиса, если команда
                    успешно отправлена;
                - `None`, если произошла ошибка связи (например, таймаут, потеря соединения).

        Examples:
            >>> with rs485.connected():
            ...     # Отправка текстовой команды
            ...     rs485.write(b"MEAS?\\r\\n")

        Notes:
            - Метод **не гарантирует получения ответа** от конечного устройства
                — он только подтверждает, что данные были переданы в шину.
            - Для двунаправленного обмена используйте связку `write()` + `read()`
                или метод `query()`.
            - Передаваемые данные не интерпретируются — они отправляются "как есть".
        """
        if not self.is_connected():
            raise ConnectionError("Can't write data - RS-485 is not connected")
        frame = self._request(Rs485Commands.transmit, payload)
        if frame is None:
            return None
        return Response.from_frame(frame)

    def query(self, payload: bytes) -> Response | None:
        """Выполняет полный цикл запрос-ответ по RS-485 шине.

        Метод отправляет команду во внешнее устройство и ожидает ответ от него.
        Реализует типичный паттерн взаимодействия с датчиками, контроллерами и
        другими устройствами, поддерживающими синхронный обмен данными.

        Последовательность операций:
        1. Отправка `payload` через `write()`;
        2. Проверка подтверждения от RS-485-сервиса;
        3. Чтение ответа через `read()`.

        Args:
            payload (bytes): Байтовая команда, отправляемая во внешнее устройство.
                Формат должен соответствовать протоколу целевого устройства
                (например, Modbus RTU, ASCII-команды, бинарный протокол).

        Returns:
            Optional[Response]:
                - Объект `Response`, содержащий данные, полученные от устройства,
                  если запрос и чтение выполнены успешно;
                - `None`, если:
                    - не удалось отправить команду,
                    - не поступило данных от устройства в течение таймаута.

        Examples:
            >>> with rs485.connected():
            ...     # Запрос идентификатора устройства по ASCII-протоколу
            ...     resp = rs485.query(b"ID?\\r\\n")
            ...     if resp:
            ...         print("Устройство:", resp.raw_data.decode().strip())

        Notes:
            - Метод **блокирующий**: ожидает как отправки, так и получения данных.
            - Не проверяет корректность формата ответа — только факт его наличия.
            - Если устройство не отвечает, метод завершится с `None` после внутреннего таймаута.
            - Для асинхронного обмена или сложных сценариев используйте отдельные вызовы
              `write()` и `read()`.
        """
        if not self.is_connected():
            raise ConnectionError("Can't make query - RS-485 is not connected")
        response = self.write(payload)
        if response is None:
            return None
        if response.has_error:
            return response
        return self.read()

    def get_status(self) -> Response | None:
        """Запрашивает диагностический статус встроенного RS-485-сервиса робота.

        Используется для мониторинга работоспособности RS-485-интерфейса и диагностики
        проблем связи с внешними устройствами.

        Returns:
            Optional[Response]:
                - Объект `Response`, содержащий данные от сервиса,
                  если запрос выполнен успешно;
                - `None`, если произошла ошибка связи (потеря соединения, таймаут)
                  или сервис не ответил.

        Examples:
            >>> with rs485.connected():
            ...     status = rs485.get_status()
            ...     if status:
            ...         print("Статус RS-485:", status.raw_data)
        """
        if not self.is_connected():
            raise ConnectionError("Can't get status - RS-485 is not connected")
        frame = self._request(Rs485Commands.status)
        if frame is None:
            return None
        return Response.from_frame(frame)

    def clear_buffer(self) -> None:
        """Очищает внутренний буфер непрочитанных фреймов RS-485.

        Метод удаляет все ранее полученные, но не обработанные фреймы из внутреннего буфера.
        Это может быть полезно в следующих случаях:
        - при переходе к новому этапу обмена,
        - при возникновении рассинхронизации между отправкой команды и
            ожидаемым ответом.

        Буфер используется для хранения "лишних" ответов, полученных во время ожидания
        конкретной команды (например, асинхронные сообщения от устройства).

        Examples:
            >>> with rs485.connected():
            ...     # После неожиданного сбоя
            ...     rs485.reset()
            ...     rs485.clear_buffer()  # игнорировать старые данные
            ...     resp = rs485.query(b"CMD\\n")  # начать с чистого листа

        Notes:
            - Метод безопасен для вызова в любое время, даже если буфер пуст.
            - Эффект зависит от параметра `use_buffer`, переданного при создании
                объекта: если буферизация отключена, метод не имеет видимого эффекта.
        """
        self._buffer.clear()

    def disconnect(self):
        """Завершает соединение с RS-485 шлюзом робота.

        Метод закрывает соединение с встроенным RS-485-сервисом и сбрасывает
        внутренний флаг подключения. Безопасен для повторного вызова — не вызывает
        ошибок, если соединение уже разорвано или никогда не устанавливалось.

        После вызова все последующие операции обмена данными (`read`, `write`,
        `query` и др.) завершатся с ошибкой до повторного подключения.

        Examples:
            >>> rs485.connect()
            >>> rs485.write(b"DATA\\n")
            >>> rs485.disconnect()  # соединение закрыто

            >>> # Повторный вызов безопасен
            >>> rs485.disconnect()  # ничего не делает, ошибки нет

        Notes:
            - Рекомендуется использовать контекстный менеджер (`with`) или метод
              `connected()` для автоматического управления жизненным циклом соединения.
            - Метод не отправляет команды в RS-485-шину — он только закрывает сокет.
        """
        if self.is_connected():
            self._write_log(
                "debug",
                f"Disconnecting RS-485 socket from {self._socket.host}",
            )
            self._socket.disconnect()
        self._is_connected = False

    def _send_command(
        self, command: Rs485Commands, payload: bytes = EMPTY_BYTES
    ) -> bool:
        """
        Отправить команду роботу.

        Args:
            command (int): тип отправляемой команды;
            payload (bytes): тело отправляемой команды.
        """
        frame_len = 1 + len(payload)  # command byte + payload
        header = struct.pack(
            "<I", frame_len
        )  # server expects host-order; code uses memcpy without ntohl
        frame = header + bytes([command.value]) + payload

        view = memoryview(frame)
        while view:
            sent = self._socket.send(bytes(view))
            view = view[sent:]

        self._write_log("debug", f"Sent RS-485 command-type: {command.name}")
        return True

    def _receive_command(
        self,
        command: Rs485Commands | None = None,
        check_buffer: bool = True,
    ) -> Rs485Frame | None:
        if command is None:
            return self._receive()

        if check_buffer and len(self._buffer) > 0:
            self._write_log(
                "debug", f"Going to command buffer for command {command.name}"
            )
            for response in self._buffer:
                self._write_log(
                    "debug", f"Going to command buffer: {response}"
                )
                if response.command == command:
                    self._buffer.remove(response)
                    return response

        for _ in range(self._MAX_RECEIVE_ATTEMPTS):
            response = self._receive()
            if response is None:
                continue

            if response.command == command:
                return response

            if (
                command == Rs485Commands.receive
                and response.command == Rs485Commands.received_data
            ):
                return response

            # Got not needed response
            self._write_log("debug", f"Adding to command buffer: {response}")
            self._buffer.append(response)
        return None

    def _request(
        self,
        command: Rs485Commands,
        payload: bytes = EMPTY_BYTES,
    ) -> Rs485Frame | None:
        for i in range(self._REQUEST_RETRYING_ATTEMPTS + 1):
            if not self._send_command(command=command, payload=payload):
                self._write_log(
                    "warning",
                    f"Failed to send request, retrying attempt {i + 1}",
                )
                continue
            response = self._receive_command(
                command=command, check_buffer=self._use_buffer
            )
            if response:
                return response
            if i != self._REQUEST_RETRYING_ATTEMPTS:
                self._write_log(
                    "warning",
                    f"Failed to get request response, retrying attempt {i + 1}",
                )
        return None

    def _receive(self) -> Rs485Frame | None:
        """Считывает один входящий фрейм от RS-485-сервиса.

        Ожидает получение фрейма в формате:
            [длина: 1 байт][код_команды: 1 байт][данные: N байт]
        """
        length_byte = self._socket.receive(1)
        if length_byte is None:
            return None
        payload_len = length_byte[0]

        command_byte = self._socket.receive(1)
        if command_byte is None:
            return None
        command = command_byte[0]

        payload = self._socket.receive(payload_len - 1)
        if payload is None:
            return None
        frame = Rs485Frame.from_response(command_id=command, payload=payload)
        if frame.return_code != Rs485ReturnCodes.ok:
            self._write_log(
                "warning", f"Received return code {frame.return_code.name}"
            )
        return frame

    def __enter__(self) -> RS485Protocol:
        """
        Служебный метод для использования RS485Protocol с контекстным
        менеджером with.
        """
        if not self.is_connected():
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Служебный метод для использования RS485Protocol с контекстным
        менеджером with.
        """
        if self.is_connected():
            self.disconnect()

    def __del__(self):
        if self.is_connected():
            self.disconnect()
