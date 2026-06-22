from __future__ import annotations

from api.robot_api_rc5v15.API.source.models.classes.data_classes.service_types import (
    JointAngleDiscrepancy,
)

from ..base_api_error import ApiError


class RobotCalibrationPositionError(ApiError):
    def __init__(self, pos_info: tuple[JointAngleDiscrepancy]):
        """
        Ошибка рассогласования углов поворотов моторов.

        Args:
            pos_info: Информация о позициях звеньев, представленная в виде кортежа
                из объектов дата-класса JointAngleDiscrepancy.
        """
        TAB = " " * 4

        _message = "Robot position discrepancy. Manual calibration needed\n"
        rows = []
        for joint in pos_info:
            rows.append(
                f"{TAB}{TAB}Joint {joint.joint_number}: [Saved: {joint.saved_position:>10.3f}°,"
                f"{TAB}Actual:{joint.actual_position:>10.3f}°]"
            )
        joints = "\n".join(rows)
        _message += (
            f"{TAB}Discrepancy between the current and last saved positions "
            f"is more than {pos_info[0].allowed_discrepancy:.1f} degrees:\n"
            f"{joints}"
        )

        super().__init__(_message)


class SavedPositionError(ApiError):
    pass


class CalculatePlaneError(ApiError):
    def __init__(self, message: str | None = None):
        _message = "Failed to calculate plane using three points"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class CalibrateTcpError(ApiError):
    def __init__(self, message: str | None = None):
        _message = "Failed to calibrate TCP params"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class FunctionTimeOutError(ApiError):
    def __init__(
        self,
        error_target: str = "",
        timeout_sec: float = 0,
        message: str | None = None,
    ):
        _message = f"{error_target} did not change in {timeout_sec} sec"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class AddWaypointError(ApiError):
    def __init__(self, message: str | None = None):
        _message = "Failed to add the waypoint"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class WristStateError(ApiError):
    def __init__(self, message: str | None = None):
        _message = "Wrist in wrong state. Check wrist mode"
        if message:
            _message += f" ({message})"
        super().__init__(_message)


class WristAIOUnitsNotSet(ApiError):
    def __init__(self, message: str | None = None):
        _message = "Failed to set analog inputs units"
        if message:
            _message += f" ({message})"
        super().__init__(_message)
