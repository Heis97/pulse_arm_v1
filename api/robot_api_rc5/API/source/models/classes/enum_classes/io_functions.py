from source.models.classes.enum_classes.base_int_enum import BaseIntEnum


class InputFunction(BaseIntEnum):
    no_func: int = 0
    move: int = 1
    hold: int = 2
    pause: int = 3
    zero_gravity: int = 4
    run: int = 5
    move_to_home: int = 6


class OutputFunction(BaseIntEnum):
    no_func: int = 0
    no_move_signal_false: int = 1
    no_move_signal_true: int = 2
    move_status_signal_true_false: int = 3
    run_signal_true: int = 4
    warning_signal_true: int = 5
    error_signal_true: int = 6
