_MM_TO_M = 1e-3
_INCH_TO_MM = 25.4


def gcode_val_to_meters(val: float, units_mm: bool) -> float:
    """Конвертирует линейное значение из единиц G-code в метры.

    Выполняет преобразование миллиметров или дюймов в метры (SI) в зависимости
    от текущего режима единиц измерения (G21/G20). Используется при парсинге
    координат, смещений центра дуги и параметров подачи.

    Args:
        val: Числовое значение в исходных единицах.
        units_mm: Флаг единиц измерения. `True` = миллиметры (G21), `False` = дюймы (G20).

    Returns:
        Значение, приведённое к метрам.

    Examples:
        >>> gcode_val_to_meters(100.0, units_mm=True)
        0.1
        >>> gcode_val_to_meters(1.0, units_mm=False)  # 1 дюйм
        0.0254
    """
    if not units_mm:
        val *= _INCH_TO_MM
    return val * _MM_TO_M
