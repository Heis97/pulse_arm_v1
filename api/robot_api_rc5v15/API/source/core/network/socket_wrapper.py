import select
import socket
import time
from typing import Optional

from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import (
    ClientDisconnectedError,
    ConnectionError,
    EmptyPackageError,
    ServerConnectionError,
)
from api.robot_api_rc5v15.API.source.models.constants import EMPTY_BYTES


class SocketWrapper:
    """
    Обертка для сокета.
    """

    _RECEIVE_TIMEOUT: float = 0.5  # sec

    def __init__(
        self,
        ip: str,
        port: int,
        connect_timeout: float,
        receive_timeout: float = _RECEIVE_TIMEOUT,
    ):
        """
        Создать новый объект класса.

        Args:
            ip (str): IPv4 адрес целевого подключения;
            port (int): порт целевого подключения;
            connect_timeout (float): максимальное время ожидания при подключении;
            receive_timeout (float): максимальное время ожидания при
                приеме/отправке данных.
        """
        self._ip: str = ip
        self._port: int = port
        self._connect_timeout: float = connect_timeout
        self._receive_timeout: float = receive_timeout
        self._socket: Optional[socket.socket] = None
        self._is_connected: bool = False
        self._is_blocking: bool = True

    @property
    def host(self) -> str:
        return self._ip

    @property
    def port(self) -> int:
        return self._port

    def connect(self, is_blocking: bool = True) -> bool:
        """
        Установить соединение.

        Args:
            is_blocking: подключиться и производить обмен в блокирующем или
                неблокирующем режиме.
        """
        self._is_blocking = is_blocking
        try:
            self._socket = socket.create_connection(
                address=(self._ip, self._port),
                timeout=self._connect_timeout,
            )
            self._configure_socket()
            if is_blocking:
                self._socket.setblocking(True)
                self._socket.settimeout(self._receive_timeout)
            else:
                self._socket.setblocking(False)
                self._socket.settimeout(0)
            self._set_is_connected(True)
        except Exception as e:
            raise ServerConnectionError(
                f"Server connection failed. Specified port ({self._port}) is unavailable."
            ) from e

        return True

    def receive(self, message_length: int) -> Optional[bytes]:
        """
        Принять данные.
        """
        if self._socket is None:
            raise ConnectionError(
                "Failed to receive package from server - socket is not connected"
            )

        if self._is_blocking:
            return self._receive_exact_blocking(message_length)
        else:
            return self._receive_exact(
                message_length=message_length, timeout=self._receive_timeout
            )

    def _receive_exact(
        self, message_length: int, timeout: float
    ) -> Optional[bytes]:
        """
        Метод для приема сообщения в неблокирующем режиме.
        """
        if self._socket is None:
            raise ConnectionError("Socket not connected")

        deadline = time.monotonic() + timeout
        buf = bytearray()

        while len(buf) < message_length:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ConnectionError("Socket receive timeout")

            ready, _, _ = select.select([self._socket], [], [], remaining)
            if not ready:
                return None

            try:
                chunk = self._socket.recv(message_length - len(buf))
                if chunk == EMPTY_BYTES:
                    raise ClientDisconnectedError("Peer closed connection")
                buf.extend(chunk)
            except BlockingIOError:
                continue  # не должно случиться

        return bytes(buf)

    def send(self, package: bytes) -> int:
        """
        Отправить данные.
        """
        if self._socket is None:
            raise ConnectionError(
                "Failed to send package to server - socket is not connected"
            )
        if package == EMPTY_BYTES:
            return 0
        if self._is_blocking:
            try:
                self._socket.sendall(package)
            except TimeoutError:
                return 0
            except Exception as e:
                raise ClientDisconnectedError(
                    "Failed to send package to server"
                ) from e
            return len(package)

        else:
            total_sent = 0
            view = memoryview(package)
            while view:
                try:
                    sent = self._socket.send(view)
                    total_sent += sent
                    view = view[sent:]
                except BlockingIOError:
                    # Ждём, пока сокет станет готов к записи
                    ready, _, _ = select.select(
                        [], [self._socket], [], self._receive_timeout
                    )
                    if not ready:
                        raise ConnectionError("Send timeout")
            return total_sent

    def disconnect(self):
        """
        Разорвать соединение.
        """
        self._set_is_connected(False)
        if self._socket is not None:
            self._socket.close()
        self._socket = None

    def is_connected(self) -> bool:
        """
        Получить статус подключения.
        """
        return self._is_connected

    def _set_is_connected(self, state: bool):
        self._is_connected = state

    def __del__(self):
        self.disconnect()

    @property
    def socket(self) -> Optional[socket.socket]:
        return self._socket

    def _configure_socket(self) -> None:
        if self._socket is None:
            return

        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        tcp_quickack = getattr(socket, "TCP_QUICKACK", None)
        if tcp_quickack is not None:
            try:
                self._socket.setsockopt(socket.IPPROTO_TCP, tcp_quickack, 1)
            except OSError:
                pass

    def _receive_exact_blocking(self, message_length: int) -> Optional[bytes]:
        if self._socket is None:
            raise ConnectionError("Socket not connected")

        deadline = time.monotonic() + self._receive_timeout
        buf = bytearray()

        while len(buf) < message_length:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if not buf:
                    return None
                raise ConnectionError("Socket receive timeout")

            try:
                self._socket.settimeout(remaining)
                chunk = self._socket.recv(message_length - len(buf))
                if chunk == EMPTY_BYTES:
                    raise EmptyPackageError(
                        "Received empty package from server"
                    )
                buf.extend(chunk)
            except (TimeoutError, socket.timeout):  # for python >=3.10, <3.10
                if not buf:
                    return None
                raise ConnectionError("Socket receive timeout")
            except EmptyPackageError:
                raise
            except Exception as e:
                raise ClientDisconnectedError(
                    f"Failed to receive package from server: {type(e)} - {e}"
                ) from e
            finally:
                self._socket.settimeout(self._receive_timeout)

        return bytes(buf)
