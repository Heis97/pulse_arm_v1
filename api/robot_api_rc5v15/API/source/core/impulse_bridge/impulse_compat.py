from __future__ import annotations

from collections.abc import Generator
from contextlib import contextmanager
from typing import Any

from api.robot_api_rc5v15.API.source.core.connection_state import (
    is_in_impulse_compat_mode,
)

from .bridge import ImpulseBridge
from .tools import normalize_var_names
from .vars_proxy import ImpulseVarsProxy


def load_impulse_vars(
    *names: str | list[str] | tuple[str, ...],
    target: Any | None = None,
):
    """Загружает переменные, определённые в программе 'Пульс'.

    По умолчанию загружает переменные в глобальную область вызывающего фрейма.
    Если передан параметр `target`, данные загружаются непосредственно в него,
    что позволяет безопасно работать с переменными внутри функций, классов и
    в многопоточной среде.

    Эта функция предназначена для загрузки переменных при работе API в режиме
    совместимости с ПО 'Пульс' (запуск скрипта на ПО 'Пульс').

    Загружаются **только корректные имена переменных Python**:
        - являются допустимыми идентификаторами,
        - не являются ключевыми словами Python,
        - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

    Если аргументы `names` не переданы, загружаются все доступные
    переменные из 'Пульса', удовлетворяющие вышеуказанным условиям.

    Args:
        *names: Необязательный список имён переменных для загрузки.
                Может быть задан как:
                - отдельные строки: `load_impulse_vars("x", "y")`,
                - список: `load_impulse_vars(["x", "y"])`,
                - кортеж: `load_impulse_vars(("x", "y"))`.
                Если не указано — загружаются все допустимые переменные.
        target: Опциональный контейнер для загрузки значений. Поддерживаются:
                - `dict`
                - `types.SimpleNamespace`
                - пользовательские прокси-объекты (например, `ImpulseVarsProxy`)
                - любые объекты с методом `.update()` или поддержкой
                  атрибутивного присваивания.
                Если `None` (по умолчанию), переменные добавляются в
                `globals()` вызывающего модуля (в глобальную область видимости).

    Examples:
        >>> # Загрузить все переменные из 'Пульса'
        >>> load_impulse_vars()
        >>> print(x)  # если 'x' определена в 'Импульсе' и является валидным именем
        ... # 42

        >>> # Загрузить только указанные переменные
        >>> load_impulse_vars("tool_id", "speed")
        >>> print(tool_id)
        ... # 5

    Notes:
        - При `target=None` функция модифицирует `globals()`, поэтому её
          рекомендуется вызывать **на уровне модуля**, а не внутри функций.
        - Передача `target` решает ограничение Python на динамическую запись
          в `locals()` и делает код потокобезопасным.
        - Переменные, чьи имена не соответствуют правилам Python
          (например, `1var`, `class`, `_private`), игнорируются.
        - Функция работает **только в режиме совместимости с 'Пульсом'**.
    """
    if not is_in_impulse_compat_mode():
        print("Can't load vars - API not running in Impulse compatible mode")
        return
    normalized_names = normalize_var_names(*names)
    ImpulseBridge.load_vars(vars_names=normalized_names, target=target)


def save_impulse_vars(
    *names: str | list[str] | tuple[str, ...],
    target: Any | None = None,
):
    """Сохраняет изменения значений переменных из скрипта обратно в программу 'Пульс'.

    По умолчанию считывает значения из глобальной области вызывающего фрейма.
    Если передан `target`, данные сериализуются и отправляются из него.

    Эта функция предназначена для сохранения изменений при работе API в режиме
    совместимости с ПО 'Пульс' (запуск скрипта на ПО 'Пульс').

    Сохраняются **только корректные имена переменных Python**, которые были
    предварительно зарегистрированы как 'Пульс-переменные':
        - были **ранее загружены** методом `load_impulse_vars()`;
        - являются допустимыми идентификаторами,
        - не являются ключевыми словами Python,
        - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

    Если аргументы `names` не переданы, сохраняются все зарегистрированные
    переменные, значения которых были изменены в глобальном пространстве
    имён вызывающего скрипта.

    Args:
        *names: Необязательный список имён переменных для сохранения.
                Может быть задан как:
                - отдельные строки: `save_impulse_vars("x", "y")`,
                - список: `save_impulse_vars(["x", "y"])`,
                - кортеж: `save_impulse_vars(("x", "y"))`.
                Если не указано — сохраняются все зарегистрированные переменные.
        target: Источник данных для сохранения. Типы и требования совпадают с
                параметром `target` в `load_impulse_vars()`.
                Если `None`, используется `globals()` вызывающего фрейма.

    Examples:
        >>> # Загрузить переменные, изменить и сохранить все
        >>> load_impulse_vars()
        >>> x += 10
        >>> save_impulse_vars()
        ... # обновлённое значение 'x' будет отправлено в 'Пульс'

        >>> # Сохранить только указанные переменные
        >>> save_impulse_vars("tool_id", "speed")

    Notes:
        - При использовании `target` изменения в глобальной области **не
          отслеживаются**. Метод  считывает данные строго из переданного объекта.
        - Сохраняются **только зарегистрированные 'Пульс-переменные'**
          (полученные через `load_impulse_vars`).
        - Новые переменные, созданные вручную без регистрации, игнорируются.
        - Функция работает **только в режиме совместимости с 'Пульсом'**.
    """
    if not is_in_impulse_compat_mode():
        print("Can't save vars - API not running in Impulse compatible mode")
        return
    normalized_names = normalize_var_names(*names)
    ImpulseBridge.save_vars(vars_names=normalized_names, target=target)


def add_impulse_vars(*names: str | list[str] | tuple[str, ...]) -> bool:
    """**На данный момент функция недоступна.** Регистрирует имена переменных в качестве 'Пульс-переменных'.

    Эта функция предназначена для регистрации пользовательских переменных
    при работе API в режиме совместимости с ПО 'Пульс'.
    Зарегистрированные переменные будут сохраняться в программу 'Пульс'
    при последующем вызове `save_impulse_vars()`.

    Регистрируются **только корректные имена переменных Python**:
        - являются допустимыми идентификаторами,
        - не являются ключевыми словами Python,
        - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

    Некорректные имена игнорируются без ошибки.

    Args:
        *names: Имена переменных для регистрации.
                Может быть задан как:
                - отдельные строки: `add_impulse_vars("x", "y")`,
                - список: `add_impulse_vars(["x", "y"])`,
                - кортеж: `add_impulse_vars(("x", "y"))`.

    Returns:
        bool: True, если хотя бы одно имя было успешно добавлено;
              False, если все переданные имена некорректны или список пуст.

    Examples:
        >>> new_param = "test string"
        >>> config_flag = False
        >>> add_impulse_vars("new_param", "config_flag")
        >>> save_impulse_vars()  # значения будут отправлены в 'Пульс'

    Notes:
        - Переменные можно регистрировать **до их фактического определения**
          в коде — главное, чтобы они существовали в глобальном пространстве имён
          на момент вызова `save_impulse_vars()`.
        - Функция работает **только в режиме совместимости с 'Пульсом'**.
    """
    if not is_in_impulse_compat_mode():
        print("Can't add vars - API not running in Impulse compatible mode")
        return False

    normalized_names = normalize_var_names(*names)

    for name in normalized_names:
        if not ImpulseBridge.is_json_serializable(name):
            continue
        ImpulseBridge.add_impulse_var(name)
    return True


def send_error_to_impulse(message: str) -> bool:
    """Отправляет сообщение об ошибке в программу 'Пульс'.

    Эта функция предназначена для передачи текстовых сообщений об ошибках
    во внешнюю систему 'Пульс' при работе API в режиме совместимости
    с ПО 'Пульс' (запуск скрипта на ПО 'Пульс').

    Вызов этого метода вызовет модальное окно 'Пульса' и остановит выполнение
    запущенной в нем программы. Аналогичного поведения можно добиться явным
    вызовом исключения в коде с помощью `raise`.

    Args:
        message (str): Текстовое сообщение об ошибке для отправки в 'Пульс'.

    Returns:
        bool: True, если сообщение успешно отправлено;
              False, если API не в режиме совместимости или сообщение
              не может быть сериализовано в JSON.

    Examples:
        >>> if some_condition_failed:
        ...     send_error_to_impulse("Некорректное значение параметра tool_id")

    Notes:
        - Функция работает **только в режиме совместимости с 'Пульсом'**.
        - Перед отправкой выполняется проверка на JSON-сериализуемость строки.
        - В случае ошибки (например, недопустимые символы) функция молча
          возвращает False без исключения.
    """
    if not is_in_impulse_compat_mode():
        print(
            "Can't send error to impulse - API not running in Impulse compatible mode"
        )
        return False
    if not ImpulseBridge.is_json_serializable(message):
        return False
    ImpulseBridge.send_error(message)
    return True


@contextmanager
def impulse_vars(
    *names: str | list[str] | tuple[str, ...],
) -> Generator[ImpulseVarsProxy, None, None]:
    """Контекстный менеджер для удобной работы с переменными 'Пульса'.

    Эта функция предназначена для работы с переменными при запуске скрипта
    в режиме совместимости с ПО 'Пульс'. При входе в блок автоматически
    загружает указанные (или все доступные) переменные из 'Пульса',
    предоставляет прокси-объект для их использования и автоматически
    сохраняет все изменения при выходе из блока.

    Загружаются **только корректные имена переменных Python**:
        - являются допустимыми идентификаторами,
        - не являются ключевыми словами Python,
        - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

    Args:
        *names: Необязательный список имён переменных для загрузки.
                Может быть задан как:
                - отдельные строки: `impulse_vars("x", "y")`,
                - список: `impulse_vars(["x", "y"])`,
                - кортеж: `impulse_vars(("x", "y"))`.
                Если не указано — загружаются все допустимые переменные.

    Yields:
        ImpulseVarsProxy: Прокси-объект, через который можно читать и
                          изменять значения переменных 'Пульса'.

    Examples:
        >>> # Работа с существующими переменными
        >>> with impulse_vars() as vars:
        ...     print(vars.x)
        ...     vars.y = 100
        ... # Все изменения сохраняются автоматически при выходе из блока

    Notes:
        - Контекстный менеджер работает **только в режиме совместимости
          с 'Пульсом'**.
    """
    if not is_in_impulse_compat_mode():
        print(
            "Can't use impulse vars - API not running in Impulse compatible mode"
        )
        return
    normalized_names = normalize_var_names(*names)
    vars_proxy = ImpulseVarsProxy(auto_add_vars=False, initial_data=None)

    try:
        ImpulseBridge.load_vars(vars_names=normalized_names, target=vars_proxy)
        yield vars_proxy
    finally:
        ImpulseBridge.save_vars(vars_names=normalized_names, target=vars_proxy)
