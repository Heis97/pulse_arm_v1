"""
Модуль с типами данных и литеральными перечислениями, используемыми в
пакете API.

> from API import types
> from API.types import RobotInfo
"""

from .source.features.additional_tools.g_code_executor import (
    GCodeCommand,
    GCodeExecutionContext,
    GCodeHandlerType,
    GCodeState,
)
from .source.features.additional_tools.tcp_calibrator import (
    TcpCalibrationResult,
)
from .source.models.classes.data_classes.api_version import RobotInfo
from .source.models.classes.data_classes.realtime_structures import (
    MotionQueueStatus,
)
from .source.models.classes.data_classes.rs_485_structures import Response
from .source.models.classes.data_classes.service_types import (
    DhModelParams,
    JointAngleDiscrepancy,
    JointsLimitsParams,
    SafetyLimitsParams,
)
from .source.models.classes.enum_classes.rs_485_structures import (
    Rs485ReturnCodes,
)
from .source.models.classes.enum_classes.various_types import (
    CoordinateSystemInfoType,
)
from .source.models.type_aliases import (
    AnalogIndex,
    AnalogWristIndex,
    AngleUnits,
    CompareSigns,
    ControllerStateName,
    CoordinateSystemProperty,
    DigitalIndex,
    DigitalSafetyIndex,
    DigitalWristIndex,
    InputFunctionName,
    JogAxis,
    JogDirection,
    JointIndex,
    MotionModeName,
    OutputFunctionName,
    PositionFormat,
    PositionOrientation,
    PowerUnits,
    ReferenceFrame,
    SafetyStatusName,
    WristInputActivationType,
    WristModeName,
)

__all__ = [
    # Data classes
    "RobotInfo",
    "DhModelParams",
    "JointAngleDiscrepancy",
    "SafetyLimitsParams",
    "JointsLimitsParams",
    "CoordinateSystemInfoType",
    "MotionQueueStatus",
    ## Wrist IO
    "Response",
    "Rs485ReturnCodes",
    # Type aliases
    "AnalogIndex",
    "AngleUnits",
    "CompareSigns",
    "ControllerStateName",
    "CoordinateSystemProperty",
    "DigitalIndex",
    "DigitalSafetyIndex",
    "DigitalWristIndex",
    "InputFunctionName",
    "JogAxis",
    "JogDirection",
    "JointIndex",
    "MotionModeName",
    "OutputFunctionName",
    "PositionFormat",
    "PositionOrientation",
    "PowerUnits",
    "ReferenceFrame",
    "SafetyStatusName",
    "WristInputActivationType",
    "WristModeName",
    "AnalogWristIndex",
    # G code
    "GCodeCommand",
    "GCodeExecutionContext",
    "GCodeHandlerType",
    "GCodeState",
    # TCP
    "TcpCalibrationResult",
]
