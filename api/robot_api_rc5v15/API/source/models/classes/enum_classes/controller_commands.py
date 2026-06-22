from enum import IntEnum


class WayPointType(IntEnum):
    # Motion types for waypoints
    joint = 0
    linear_cart = 1
    tcp_pose = 2


class RpmpWayPointType(IntEnum):
    # Motion types for RPMP waypoints
    movel = 0
    movep = 1
    movec = 2
    movej = 3
    movej_tcp = 4


class JogModes(IntEnum):
    off = 0
    force = 1
    velocity = 2
    position = 3


class JointJogModes(IntEnum):
    off = 0
    on = 1


class PauseModes(IntEnum):
    disable = 0
    enable = 1


class RealtimeCommands(IntEnum):
    REALTIME_STOPJ = 0
    REALTIME_SERVOJ = 1
    REALTIME_SPEEDJ = 2
    REALTIME_SET_SERVOJ_PARAMS = 3


class ControllerCommands(IntEnum):
    CTRLR_COMS_STOP = 0
    CTRLR_COMS_MOVE_ADD_WP = 1
    CTRLR_COMS_MOVE_LAUNCH = 2  # Sets to run mode. Impel to move.
    CTRLR_COMS_PAUSE = (
        4  # Pause-control: use '1'- to set pause, '0'- to discard pause
    )
    CTRLR_COMS_POWER = 5  # Power control
    CTRLR_COMS_CTRL_DISCONNECT = (
        6  # Graceful disconnection of control socket, without error
    )
    CTRLR_COMS_JOG = 8
    CTRLR_COMS_JOINT_JOG = 10
    CTRLR_COMS_MOVE_RPMP_RUN = 11
    CTRLR_COMS_RPMP_PAUSE = 12
    CTRLR_COMS_ENABLE_REALTIME = 13
    CTRLR_COMS_SET_OUTPUTS = 14  # Sets digital & analog outputs
    CTRLR_COMS_ZG = 15  # Zero Gravity
    CTRLR_COMS_CONFIRM_POSITION = 16  # Confirm that start position of robot is same in interface and in real
    CTRLR_COMS_SET_WRIST_IO = (
        17  # Set digital outputs (and modes of analog inputs)
    )
    CTRLR_COMS_RPMP_MOVE_ADD_WP = 18

    CTRLR_COMS_UNLOCK = 100

    # Getters
    CTRLR_COMS_GET_LAST_POS = 901
    CTRLR_COMS_GET_MOVE_SCALE = 1115
    CTRLR_COMS_GET_GRAVITY = 1116
    CTRLR_COMS_GET_SFTY_LIMITS = 1118
    ctrlr_coms_get_current_limits = 1119
    CTRLR_COMS_GET_PAYLOAD = 1121
    CTRLR_COMS_GET_TOOL = 1122
    CTRLR_COMS_GET_HOME_POSE = 1150
    CTRLR_COMS_GET_JOINT_POS_LIMITS = 1170
    CTRLR_COMS_GET_JOINT_VEL_LIMITS = 1171
    CTRLR_COMS_GET_JOINT_CURRENT_LIM = 1172
    CTRLR_COMS_GET_JOINT_NOMINAL_TRQ = 1173
    CTRLR_COMS_GET_JOINT_MAX_TRQ = 1174
    CTRLR_COMS_GET_AN_OUT_LIMITS = 1180
    CTRLR_COMS_GET_AN_IN_LIMITS = 1181
    CTRLR_COMS_GET_MAX_PAYLOAD = 1191
    CTRLR_COMS_GET_ROBOT_VIEW_INFO = 4000
    CTRLR_COMS_GET_SW_VERSION = 3000
    CTRLR_COMS_GET_PROTO_VERSION = 3001
    # get kinematics
    CTRLR_COMS_FKINE = 2000
    CTRLR_COMS_IKINE = 2001
    CTRLR_COMS_IKINE_OPTIMAL = 2002
    # io func
    CTRLR_COMS_GET_DIG_INPUT_FUNC = 1132
    CTRLR_COMS_GET_DIG_OUTPUT_FUNC = 1133
    CTRLR_COMS_CBOX_GET_SFTY_INPUT_FUNC = 1601

    # Setters
    CTRLR_COMS_SET_MOVE_SCALE = 1015
    CTRLR_COMS_SET_GRAVITY = 1016
    CTRLR_COMS_SET_SFTY_LIMITS = 1018
    CTRLR_COMS_SET_PAYLOAD = 1021
    CTRLR_COMS_SET_TOOL = 1022
    CTRLR_COMS_SET_JOG_PARAM = 1023
    CTRLR_COMS_SET_HOME_POSE = 1050
    CTRLR_COMS_STORE_SETTINGS = 1300

    # io func
    CTRLR_COMS_SET_DIG_INPUT_FUNC = 1032
    CTRLR_COMS_SET_DIG_OUTPUT_FUNC = 1033

    # wrist func
    CTRLR_COMS_SET_WRIST_DIG_INPUT_FUNC = 1053
    CTRLR_COMS_SET_WRIST_DIG_OUTPUT_FUNC = 1054
