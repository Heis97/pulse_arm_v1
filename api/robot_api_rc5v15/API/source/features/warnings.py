from __future__ import annotations

import os
import sys
import warnings
from typing import TYPE_CHECKING

__all__ = ["warning", "deprecated", "APIWarning", "APIDeprecationWarning"]


class APIWarning(UserWarning):
    """Базовое предупреждение для пакета API."""

    pass


class APIDeprecationWarning(DeprecationWarning):
    """Предупреждение об устаревании для пакета API."""

    pass


def warning(
    message: str, category: type[Warning] = APIWarning, stacklevel: int = 1
):
    """Вывести предупреждение через кастомный форматтер."""
    warnings.warn(message, category=category, stacklevel=stacklevel)


if TYPE_CHECKING:
    # Для подсветки в IDE используем оригинальный декоратор
    from typing_extensions import deprecated
else:

    def deprecated(
        message: str,
        *,
        category: type[Warning] = APIDeprecationWarning,
        stacklevel: int = 1,
    ):
        """Декоратор для устаревших функций с кастомным предупреждением."""

        
        from typing_extensions import deprecated

        return deprecated(message, category=category, stacklevel=stacklevel)  # type: ignore


_COLORS = {
    "DeprecationWarning": "\033[33m",  # Жёлтый
    "FutureWarning": "\033[35m",  # Фиолетовый
    "RuntimeWarning": "\033[31m",  # Красный
    "UserWarning": "\033[36m",  # Голубой
    "Warning": "\033[37m",  # Белый
}
_RESET = "\033[0m"


def _get_color(category: type[Warning] | None) -> str:
    """Возвращает ANSI-код для типа предупреждения."""
    if category is None:
        return _COLORS.get("Warning", "")

    for base in category.__mro__:
        if base.__name__ in _COLORS:
            return _COLORS[base.__name__]

    return _COLORS.get("Warning", "")


def _supports_color(file) -> bool:
    """Безопасно проверяет, поддерживает ли поток вывод цветов."""
    # Стандарт NO_COLOR: https://no-color.org/
    if os.getenv("NO_COLOR"):
        return False
    return bool(getattr(file, "isatty", lambda: False)())


_original_showwarning = warnings.showwarning


def _custom_showwarning(
    message,
    category: type[Warning],
    filename,
    lineno,
    file=None,
    line=None,
):
    """
    Форматирует предупреждение без пути к файлу и номера строки.
    """
    if os.getenv("SUPPRESS_INTERNAL_API_WARNING"):
        return

    is_custom_category = category and issubclass(
        category, (APIWarning, APIDeprecationWarning)
    )

    if not is_custom_category:
        # Если предупреждение НЕ из нашего пакета,
        # передаем его в стандартный обработчик Python
        _original_showwarning(message, category, filename, lineno, file, line)
        return

    target_file = file if file is not None else sys.stderr
    use_color = _supports_color(target_file)

    cat_name = category.__name__ if category else "Warning"
    msg_str = str(message)

    color = _get_color(category) if use_color else ""
    reset = _RESET if use_color else ""

    # Красим только тег и заголовок, чтобы не перегружать вывод
    warning_msg = (
        f"{color}{'=' * 80}{reset}\n"
        f"{color}[{cat_name}] API WARNING: {msg_str}{reset}\n"
        f"{color}{'=' * 80}{reset}\n"
    )

    target_file.write(warning_msg)


warnings.showwarning = _custom_showwarning
warnings.filterwarnings("always", category=UserWarning)
warnings.filterwarnings("always", category=DeprecationWarning)
