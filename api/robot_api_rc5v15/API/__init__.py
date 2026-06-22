"""
API - программный интерфейс для взаимодействия с коллаборативными роботами серии
RC компании "РобоПро".

Пакет содержит инструменты, позволяющие реализовывать взаимодействие с
коллаборативными роботами серии RC (включая управление одним или более роботом,
получение параметров роботов, настройку роботов). Помимо этого, пакет
предоставляет методы для управления входами и выходами (как аналоговыми, так и
цифровыми) контроллера робота и самого робота (его запястья).

Основные компоненты:
- RobotApi - входная точка для взаимодействия с роботом;
- coords - подмодуль с инструментами для работы с пользовательскими системами
    координат;
- tools - подмодуль с инструментами, предоставляемыми пакетом;
- types - подмодуль с публичными типами данных, которые используются в пакете;
- io - подмодуль с инструментами для взаимодействия с внешними устройствами
    через промышленные интерфейсы.

Примеры:
    >>> from API import RobotApi
    >>> robot = RobotApi("192.168.0.12")
    ... # Работа с роботом
    ...
    >>> from API import RobotApi
    >>> with RobotApi("192.168.0.12") as robot:
    ...     # Работа с роботом

    Подробные примеры использования компонентов модуля можно найти в
    документации соответствующего компонента или функции.
"""

# Далее реализован ленивый импорт компонентов пакета для быстрого импорта
# модуля в режиме совместимости с Импульсом
import sys
from typing import TYPE_CHECKING

from . import tools  # Для инициализации Impulse bridge (не убирать!)

# Остальные компоненты импортируем только при запросе
if TYPE_CHECKING or "pdoc" in sys.modules:
    from . import coords, io, source, tools, types
    from .rc_api import RobotApi

__all__ = ["RobotApi", "tools", "types", "coords", "io", "source"]


def __getattr__(name: str):
    if name == "RobotApi":
        from .rc_api import RobotApi as _RobotApi

        _RobotApi.__module__ = "API"
        return _RobotApi

    if name == "tools":
        from . import tools

        return tools

    if name == "types":
        from . import types

        return types

    if name == "coords":
        from . import coords

        return coords

    if name == "io":
        from . import io

        return io

    if name == "source":
        from . import source

        return source

    raise AttributeError(f"module 'API' has no attribute '{name}'")


def __dir__():
    """Позволяет IDE и autocomplete видеть все публичные имена."""
    return __all__
