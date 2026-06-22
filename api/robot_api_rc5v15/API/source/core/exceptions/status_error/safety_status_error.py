from ..base_api_error import ApiError


class EmergencyStopError(ApiError):
    """Safety status: 5."""

    def __init__(self, message: str = ""):
        self.message = "Robot emergency stop (1-category stop event)"
        if message:
            self.message = self.message + f" ({message})"
        super().__init__(self.message)


class ControllerFaultError(ApiError):
    """Safety status: 6."""

    def __init__(self, message: str = ""):
        self.message = "Robot software fault (0-category stop event). Check core log-files"
        if message:
            self.message = self.message + f" ({message})"
        super().__init__(self.message)


class SafetyViolationError(ApiError):
    """Safety status: 7."""

    def __init__(self, message: str = ""):
        self.message = "Robot safety violation (0-category stop event)"
        if message:
            self.message = self.message + f" ({message})"
        super().__init__(self.message)
