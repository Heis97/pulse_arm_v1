from types import TracebackType
from typing import List, Literal, Optional, Tuple, Type, Union

# IO
max_dig_in_index_count = 24  # real count
max_dig_out_index_count = 24  # real count

AnalogIndex = Literal[0, 1, 2, 3]
DigitalIndex = Literal[
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
]
DigitalSafetyIndex = Literal[0, 1, 2, 3, 4, 5, 6, 7]
DigitalWristIndex = Literal[0, 1]
AnalogWristIndex = Literal[0, 1, 2, 3]

# Logger
_SysExcInfoType = Union[
    Tuple[Type[BaseException], BaseException, Optional[TracebackType]],
    Tuple[None, None, None],
]
ExcInfoType = Union[None, bool, _SysExcInfoType, BaseException]

# Diff args
JointIndex = Literal[0, 1, 2, 3, 4, 5]
PositionOrientation = Union[List[float], Tuple[float, ...]]

IkSolutionId = Literal[-1, 0, 1, 2, 3, 4, 5, 6, 7]
LiteralType = Literal[
    "compare",
    "math",
    "angle",
    "position",
    "power",
    "axis",
    "reference_frame",
    "activation_type",
    "cs",
    "mm",
    "ss",
    "in_f",
    "out_f",
    "Wm",
    "ik_id",
]
CompareSigns = Literal[">", "<"]
JogDirection = Literal["+", "-"]
ReferenceFrame = Literal["base", "tcp"]
AngleUnits = Literal["rad", "deg"]
CoordinateSystemProperty = Literal["pose", "units"]
PositionFormat = Literal["joints", "tcp"]
PowerUnits = Literal["mA", "V"]
JogAxis = Literal["X", "Y", "Z", "Rx", "Ry", "Rz"]
ControllerStateName = Literal["on", "off", "run"]
MotionModeName = Literal["move", "move_adv", "pause", "hold", "realtime"]
SafetyStatusName = Literal["recovery", "normal", "reduced", "safeguard_stop"]
InputFunctionName = Literal[
    "no_func",
    "move",
    "move_adv",
    "hold",
    "pause",
    "zero_gravity",
    "run",
    "move_to_home",
]
OutputFunctionName = Literal[
    "no_func",
    "no_move_signal_false",
    "no_move_signal_true",
    "move_status_signal_true_false",
    "run_signal_true",
    "warning_signal_true",
    "error_signal_true",
]
WristModeName = Literal["off", "rs485", "analog_in", "nc", "gnd"]
WristInputActivationType = Literal["hold", "trigger"]
