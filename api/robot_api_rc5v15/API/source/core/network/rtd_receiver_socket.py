from __future__ import annotations

import logging
import struct
import time
from collections.abc import Callable
from threading import Lock, Thread, main_thread

from api.robot_api_rc5v15.API.source.core.exceptions.base_api_error import ApiError
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    RTDReceivingError,
)
from api.robot_api_rc5v15.API.source.core.network.socket_wrapper import (
    SocketWrapper,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.data_classes.rtd_structure import (
    RTD,
    RTDataPackageBorders,
)
from api.robot_api_rc5v15.API.source.models.constants import RTD_PORT
from api.robot_api_rc5v15.API.source.models.type_aliases import ExcInfoType


class RTDReceiver(LoggerMixin):
    """
    Класс-драйвер для приема RTD данных с робота.
    """

    _DATA_RECEIVE_TIMEOUT: float = 0.05  # sec (> 0.005)
    _MAX_RECEIVING_FAILURES: int = 10
    _MAX_CONSECUTIVE_RECONNECT_FAILURES: int = 3
    # Параметры выше взяты с большим запасом. Уменьшение их значений может
    # привести к нестабильному приему на ОС Windows

    _MAX_BUFFER_SIZE: int = RTD.get_struct_size() * 10  # bytes
    _RECONNECT_TIMEOUT: float = 0.005  # sec
    _CONNECTION_ATTEMPTS: int = 3

    def __init__(
        self,
        ip: str,
        timeout: float,
        allow_reconnection: bool,
        logger: logging.Logger | None = None,
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
        self._allow_reconnection: bool = allow_reconnection
        self._set_logger(logger)
        self._socket: SocketWrapper = SocketWrapper(
            ip=ip,
            port=RTD_PORT,
            connect_timeout=timeout,
            receive_timeout=self._DATA_RECEIVE_TIMEOUT,
        )
        self._is_loop_running: bool = False
        self._is_connected: bool = False
        self._thread: Thread | None = None
        self._lock = Lock()

        self._rt_data = RTD()

        self._struct_size: int = RTD.get_struct_size()
        self._begin_bytes = struct.pack(
            RTDataPackageBorders.rtd_packet_begin_structure,
            RTDataPackageBorders.rtd_packet_begin,
        )
        self._end_bytes = struct.pack(
            RTDataPackageBorders.rtd_packet_end_structure,
            RTDataPackageBorders.rtd_packet_end,
        )
        self._begin_size = len(self._begin_bytes)
        self._end_size = len(self._end_bytes)
        self._recv_buffer: bytearray = bytearray()

        self._is_reconnecting: bool = False

    def connect(self) -> bool:
        """
        Подключиться к RTD сокету робота.

        Returns:
            bool: True если подключение произведено успешно.
        """
        self._write_log("debug", "Connecting RTD receiver")
        self._is_reconnecting = False
        for _ in range(self._CONNECTION_ATTEMPTS):
            if self._socket.connect() and self._receive_rtd():
                self._is_connected = True
                return True
            self._recv_buffer.clear()
            self._socket.disconnect()
            time.sleep(self._RECONNECT_TIMEOUT)
        self._is_connected = False
        return False

    def is_connected(self) -> bool:
        """
        Получить статус подключения к роботу.

        Returns:
            bool: True если подключение активно.
        """
        with self._lock:
            return (
                self._is_connected and self._socket.is_connected()
            ) or self._is_reconnecting

    def get_data(self) -> RTD:
        """
        Получить текущие RTD данные.

        Returns:
            RTD: Дата класс с данными.
        """
        if not self.is_connected():
            raise ApiError("Can't get RTD data - robot is not connected")
        with self._lock:
            return self._rt_data

    def start_loop(
        self,
        error_callback: Callable[[ExcInfoType], None] | None = None,
        thread_name: str = "RTD-Receiver-loop",
    ):
        """
        Запустить цикл приема RTD данных в отдельном потоке.

        Args:
            error_callback: функция, которая будет вызвана при возникновении
                исключения в цикле приема RTD данных;
            thread_name: имя потока, в котором будет запущен цикл приема.
        """
        self._write_log("debug", "Starting RTD receiving loop")
        self._thread = Thread(
            target=self._receiving_loop,
            args=(error_callback,),
            daemon=False,
            name=thread_name,
        )
        self._is_loop_running = True
        self._thread.start()

    def stop_loop(self):
        """
        Остановить цикл приема RTD данных.
        """
        if self._thread is None:
            return
        if not self._is_loop_running:
            return
        self._write_log("debug", "Stopping RTD receiving loop")
        with self._lock:
            self._is_loop_running = False
        try:
            self._thread.join(2)
        except Exception:
            pass

    def is_running(self) -> bool:
        """
        Получить статус работы цикла приема данных.

        Returns:
            bool: True если цикл запущен.
        """
        with self._lock:
            is_running = self._is_loop_running
        return is_running and self.is_connected()

    def _receiving_loop(  # noqa: C901
        self, error_callback: Callable[[ExcInfoType], None] | None
    ):
        """
        Цикл приема данных.
        """
        self._is_loop_running = True
        self._recv_buffer = bytearray()
        consecutive_failures = 0
        reconnect_attempts = 0
        self._write_log("debug", "RTD receiving loop was started")
        try:
            while self._is_loop_running and main_thread().is_alive():
                try:
                    success = self._receive_rtd()
                except Exception as e:
                    self._write_log(
                        "error", f"Error occurred during RTD receiving: {e}"
                    )
                    if not self._allow_reconnection:
                        raise RTDReceivingError(
                            "Persistent data loss (reconnection disabled)"
                        )

                    reconnect_attempts += 1
                    if (
                        reconnect_attempts
                        > self._MAX_CONSECUTIVE_RECONNECT_FAILURES
                    ):
                        raise RTDReceivingError("Persistent data loss")

                    self._write_log(
                        "warning",
                        f"Attempting consecutive reconnection "
                        f"{reconnect_attempts}/{self._MAX_CONSECUTIVE_RECONNECT_FAILURES}...",
                    )
                    if self._reconnect_socket():
                        reconnect_attempts = 0
                    continue

                if success:
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    # Принято много невалидных пакетов
                    if consecutive_failures >= self._MAX_RECEIVING_FAILURES:
                        self._write_log(
                            "error", "Too many RTD receive failures"
                        )
                        if not self._allow_reconnection:
                            raise RTDReceivingError(
                                "Persistent data loss (reconnection disabled)"
                            )

                        reconnect_attempts += 1
                        if (
                            reconnect_attempts
                            > self._MAX_CONSECUTIVE_RECONNECT_FAILURES
                        ):
                            raise RTDReceivingError("Persistent data loss")

                        self._write_log(
                            "warning",
                            f"Attempting consecutive reconnection "
                            f"{reconnect_attempts}/{self._MAX_CONSECUTIVE_RECONNECT_FAILURES}...",
                        )
                        if not self._reconnect_socket():
                            reconnect_attempts = 0

            self._is_loop_running = False
            self._write_log("debug", "RTD receiving loop was stopped")
        except Exception as error:
            self._write_log(
                "debug", f"RTD receiving loop failed with error: {error}"
            )
            self._is_loop_running = False
            if error_callback is not None:
                error_callback(error)

    def disconnect(self) -> None:
        """
        Отключиться от RTD сокета робота.
        """

        if self.is_running() or self.is_connected():
            self._write_log("debug", "Disconnecting RTD receiver")
            self.stop_loop()
        self._socket.disconnect()
        self._is_connected = False
        self._thread = None

    def _receive_rtd(self) -> bool:  # noqa: C901
        """
        Принять RTD данные один раз.
        """
        while len(self._recv_buffer) < self._struct_size:
            chunk = self._socket.receive(
                self._struct_size - len(self._recv_buffer)
            )
            if not chunk:
                return False
            self._recv_buffer.extend(chunk)

        if len(self._recv_buffer) > self._MAX_BUFFER_SIZE:
            del self._recv_buffer[
                : len(self._recv_buffer) - self._MAX_BUFFER_SIZE
            ]

        while len(self._recv_buffer) >= self._struct_size:
            pos = self._recv_buffer.find(self._begin_bytes)
            if pos == -1:
                self._recv_buffer.clear()
                return False

            if pos > 0:
                del self._recv_buffer[:pos]

            if len(self._recv_buffer) < self._struct_size:
                break

            expected_end_pos = self._struct_size - self._end_size
            actual_end = self._recv_buffer[
                expected_end_pos : self._struct_size
            ]

            if actual_end == self._end_bytes:
                try:
                    rtd_data = RTD.from_bytes(
                        self._recv_buffer[: self._struct_size]
                    )
                except Exception:
                    del self._recv_buffer[0]
                    continue

                if not rtd_data.is_valid_content(logger=self._get_logger()):
                    del self._recv_buffer[0]
                    continue

                with self._lock:
                    self._rt_data = rtd_data
                del self._recv_buffer[: self._struct_size]
                return True

            del self._recv_buffer[: self._begin_size]

        return False

    def _reconnect_socket(self) -> bool:
        with self._lock:
            self._is_reconnecting = True
            self._is_connected = False

        try:
            self._socket.disconnect()
            self._recv_buffer.clear()
            time.sleep(self._RECONNECT_TIMEOUT)

            if self._socket.connect():
                self._write_log("info", "Reconnection successful")

                with self._lock:
                    self._is_connected = True
                    self._is_reconnecting = False

                return True
            self._write_log("warning", "Reconnection failed")
            self._is_reconnecting = False
            return False
        except Exception as e:
            self._write_log("warning", f"Reconnection failed with error: {e}")
            self._is_reconnecting = False
            return False

    def __del__(self):
        if self.is_connected():
            self.disconnect()
