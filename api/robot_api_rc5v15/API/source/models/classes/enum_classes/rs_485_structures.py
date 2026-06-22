from enum import IntEnum


class Rs485Commands(IntEnum):
    """
    Команды управления встроенным RS-485-сервисом робота.

    unknown: не является командой протокола передачи, зарезервировано для
    приема неизвестных команд.
    """

    reset = 0
    transmit = 1
    receive = 2
    status = 3
    received_data = 4
    unknown = 5


class Rs485ReturnCodes(IntEnum):
    """
    Коды возврата от RS-485-сервиса робота. Определяются используемым
    протоколом передачи данных.

    Attributes:
        ok: Операция выполнена успешно.
        timeout: Таймаут ожидания ответа от внешнего устройства.
        tx_failed: Ошибка передачи данных по RS-485 шине.
        rx_no_response: Ответ от устройства не получен.
        no_wrist_board: Плата запястья отсутствует или не обнаружена.
        wrong_wrist_mode: Плата запястья находится в неподходящем режиме.
        op_in_progress: Операция уже выполняется (повторный запрос отклонён).
        failure: Общая ошибка выполнения операции.
        overflow: Переполнение буфера данных.
        not_init: RS-485-сервис не инициализирован.
    """

    ok = 0
    timeout = 1
    tx_failed = 2
    rx_no_response = 3
    no_wrist_board = 4
    wrong_wrist_mode = 5
    op_in_progress = 6
    failure = 7
    overflow = 8
    not_init = 9
