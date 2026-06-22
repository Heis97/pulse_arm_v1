from .base_api_error import ApiError


class ConnectionError(ApiError):
    pass


class ResponseTimeoutError(ConnectionError):
    pass


class ServerConnectionError(ConnectionError):
    pass


class ServerPingError(ConnectionError):
    pass


class EmptyPackageError(ConnectionError):
    pass


class ClientDisconnectedError(ConnectionError):
    def __init__(self, message: str = ""):
        _message = "Client has been disconnected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class RuntimeMotionError(ConnectionError):
    pass


class ControllerUnlockError(ApiError):
    def __init__(self, message: str = ""):
        _message = "Server access denied"
        if message:
            _message += f" ({message})"
        super().__init__(_message)
