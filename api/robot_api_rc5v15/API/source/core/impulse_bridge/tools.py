from __future__ import annotations

import time


def to_base36(n: int) -> str:
    """
    Преобразует целое число в строку в системе счисления base36
    (цифры 0-9 и заглавные буквы A-Z).
    """
    if n == 0:
        return "0"
    chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    result = ""
    while n:
        result = chars[n % 36] + result
        n //= 36
    return result


def get_short_session_id() -> str:
    """
    Генерирует короткий уникальный идентификатор сессии (6 символов),
    основанный на текущем времени в микросекундах, закодированном в base36.
    """
    us = int(time.time() * 1_000_000)
    return to_base36(us)[-6:]


def normalize_var_names(
    *names: str | list[str] | tuple[str, ...],
) -> list[str]:
    """
    Преобразует аргументы в плоский список строк.

    Поддерживает:
      - f("a", "b")
      - f(["a", "b"])
      - f(("a", "b"))
      - f() → []
    """
    if len(names) == 1 and isinstance(names[0], (list, tuple)):
        items = names[0]
    else:
        items = names

    result = []
    for item in items:
        if isinstance(item, str):
            result.append(item)
        else:
            raise TypeError(
                f"All variable names must be strings, got {type(item)}: {item!r}"
            )
    return result
