from __future__ import annotations

from ..base_api_error import ApiError


class ControllerFailureError(ApiError):
    """Controller state: 6."""

    def __init__(self, message: str | None = None):
        self.message = (
            "Controller inner error happened. Controller reboot needed"
        )
        if message:
            self.message = self.message + f" ({message})"
        super().__init__(self.message)
