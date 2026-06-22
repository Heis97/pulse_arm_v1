from __future__ import annotations

from ..base_api_error import ApiError


class DataError(ApiError):
    pass


class CorruptedPackageError(DataError):
    def __init__(self, message: str | None = None):
        _message = "Received corrupted package from server"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class RTDParsingError(DataError):
    def __init__(self, message: str | None = None):
        _message = "Can not parse RT data package"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class RTDReceivingError(DataError):
    def __init__(self, message: str | None = None):
        _message = "Failed to receive RT data package"
        if message:
            _message += f" ({message})"
        super().__init__(_message)
