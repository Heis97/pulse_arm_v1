from enum import IntEnum


from .controller_commands import ControllerCommands


class InComingSafetyStatus(IntEnum):
    """
    Класс входящих состояний безопасности.
    Значения класса используются только для сравнения значений с rt_data.
    В остальной логике программы используются только имена полей.
    """

    deinit = 0
    recovery = 1
    normal = 2
    reduced = 3
    safeguard_stop = 4
    emergency_stop = 5
    fault = 6
    violation = 7


class InComingMotionMode(IntEnum):
    """
    Класс входящих режимов движения.
    Значения класса используются только для сравнения значений с rt_data.
    В остальной логике программы используются только имена полей.
    """

    hold = 0
    pause = 5
    move = 4
    realtime = 6
    move_adv = 7
    zero_gravity = 1
    jog = 2
    joint_jog = 3


class OutComingMotionMode(IntEnum):
    """
    Класс исходящих режимов движения.
    Значения класса используются только для отправки команд.
    В остальной логике программы используются только имена полей.
    """

    hold = ControllerCommands.CTRLR_COMS_STOP
    pause = ControllerCommands.CTRLR_COMS_PAUSE
    move = ControllerCommands.CTRLR_COMS_MOVE_LAUNCH
    move_adv = ControllerCommands.CTRLR_COMS_MOVE_RPMP_RUN
    realtime = ControllerCommands.CTRLR_COMS_ENABLE_REALTIME
    zero_gravity = ControllerCommands.CTRLR_COMS_ZG
    jog = ControllerCommands.CTRLR_COMS_JOG
    joint_jog = ControllerCommands.CTRLR_COMS_JOINT_JOG


class InComingControllerState(IntEnum):
    """
    Класс входящих состояний контроллера.
    Значения класса используются только для сравнения значений с rt_data.
    В остальной логике программы используются только имена полей.
    """

    idle = 0
    off = 1
    stby = 2
    on = 3
    run = 4
    calibration = 5
    failure = 6
    force_exit = 7


class OutComingControllerState(IntEnum):
    """
    Класс исходящих состояний контроллера.
    Значения класса используются только для отправки команд.
    В остальной логике программы используются только имена полей.
    """

    off = 0
    stby = 1
    on = 2
    run = 3


class WristMode(IntEnum):
    """
    Класс режимов инструмента.
    Значения класса используются только для сравнения значений с rt_data.
    В остальной логике программы используются только имена полей.
    """

    off = 0
    rs485 = 1
    analog_in = 2
    nc = 3
    gnd = 4


class MotionWarning(IntEnum):
    """
    Класс статусов предупреждения о состоянии робота
    Значения класса используются только для сравнения значений с rt_data.
    В остальной логике программы используются только имена полей.
    """

    no_warning = 0
    protective_stop = 1
    self_collision = 2
