from __future__ import annotations

from ..base_api_error import ApiError


class ArgumentError(ApiError):
    pass


class ArgSignError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect sign selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgUnitsError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect units of measurement selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgPositionFormatError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect type of position format selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgValueError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect value selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgControllerStateError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect controller state selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgMotionModeError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect motion mode selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgSafetyStatusError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect safety status selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArrayLengthError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect array length"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgInputFunctionError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect input function selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgOutputFunctionError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect output function selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class ArgToolModeError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect tool mode selected"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class WristOutputActivationTypeError(ArgumentError):
    def __init__(self, message: str | None = None):
        _message = "Incorrect wrist output activation type"
        if message:
            _message += f" ({message})"
        super().__init__(_message)
