from __future__ import annotations

import keyword
from collections.abc import ItemsView, KeysView, ValuesView
from typing import Any

from .bridge import ImpulseBridge


class ImpulseVarsProxy:
    """Прокси-объект для безопасной работы с переменными программы 'Импульс'.

    Этот класс предоставляет удобный интерфейс для чтения, изменения и регистрации
    переменных при работе API в режиме совместимости с ПО 'Импульс'
    (запуск скрипта на ПО 'Импульс').

    Все атрибуты прокси-объекта отражают значения переменных, загруженных
    из 'Импульса', или зарегистрированных пользователем. При выходе из контекстного
    менеджера `impulse_vars` все изменения автоматически сохраняются обратно
    в 'Импульс'.

    Зарегистрироваться могут **только корректные имена переменных Python**:
        - являются допустимыми идентификаторами,
        - не являются ключевыми словами Python,
        - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

    Значения переменных должны быть JSON-сериализуемыми.

    Notes:
        - Экземпляры этого класса создаются автоматически контекстным менеджером
          `impulse_vars` — прямое создание вручную не рекомендуется.
        - Класс является частью внутренней реализации API и предназначен
          для использования только в режиме совместимости с 'Импульсом'.
        - Чтение несуществующего атрибута вызовет `AttributeError`.
    """

    def __init__(
        self,
        auto_add_vars: bool,
        initial_data: dict[str, Any] | None,
    ):
        self._data = initial_data.copy() if initial_data else {}
        self._auto_add_vars = auto_add_vars

    def _add_var(self, var_name: str, value: Any) -> None:
        """**На данный момент функция недоступна.** Явно добавляет переменную в набор переменных, сохраняемых в программу 'Импульс'.

        Эта функция предназначена для регистрации новой переменной при работе
        в режиме совместимости с ПО 'Импульс'. Переменная будет автоматически
        сохранена при выходе из контекстного менеджера ``impulse_vars``.

        Регистрируются **только корректные имена переменных Python**:
            - являются допустимыми идентификаторами,
            - не являются ключевыми словами Python,
            - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

        Значение переменной должно быть JSON-сериализуемым.

        Args:
            var_name (str): Имя переменной для регистрации.
            value (Any): Значение переменной. Должно поддерживать сериализацию в JSON.

        Raises:
            ValueError: Если имя переменной не соответствует требованиям
                        к идентификаторам Python.

        Examples:
            >>> with impulse_vars(auto_add_vars=False) as vars:
            ...     vars._add_var("timeout", 30)
            ...     # 'timeout' будет сохранена в 'Импульс' при выходе из блока

        Notes:
            - Если переменная с таким именем уже существует, её значение будет
                обновлено.
        """
        if not self._is_valid_name(var_name):
            raise ValueError(f"Invalid variable name: '{var_name}'")

        self._data[var_name] = value

        ImpulseBridge.add_impulse_var(var_name)

    def _add_vars(self, **kwargs) -> None:
        """**На данный момент функция недоступна.** Добавляет несколько переменных в программу 'Импульс' одновременно.

        Эта функция предназначена для массовой регистрации переменных
        при работе в режиме совместимости с ПО 'Импульс'.
        Все переданные переменные будут сохранены при выходе из контекстного
        менеджера `impulse_vars`.

        Регистрируются **только корректные имена переменных Python**:
            - являются допустимыми идентификаторами,
            - не являются ключевыми словами Python,
            - не начинаются с символа подчёркивания `_` (т.е. публичные имена).

        Значения переменных должны быть JSON-сериализуемыми.

        Args:
            **kwargs: Пары `имя=значение` для регистрации.
                      Каждое имя должно соответствовать требованиям к
                      идентификаторам Python.

        Examples:
            >>> with impulse_vars(auto_add_vars=False) as vars:
            ...     vars._add_vars(x=42, y="hello", flag=True)
            ...     # Переменные 'x', 'y', 'flag' будут сохранены в 'Импульс'

        Notes:
            - Некорректные имена (например, `1var`, `class`, `_private`)
              вызовут исключение.
        """
        for var_name, value in kwargs.items():
            self._add_var(var_name, value)

    def has_var(self, var_name: str) -> bool:
        """
        Проверяет, зарегистрирована ли переменная с указанным именем в
        качестве 'Импульс-переменной'.

        Args:
            var_name (str): Имя переменной.

        Returns:
            bool: `True`, если переменная существует в наборе
                'Импульс-переменных', иначе `False`.

        Examples:
            >>> name = "abc"
            >>> result = vars.has_var(name)
            >>> if result:
            >>>     print(f"Переменная {name} есть в vars")
        """
        return var_name in ImpulseBridge.get_impulse_var_names()

    @property
    def var_names(self) -> set[str]:
        """
        Возвращает множество имён всех переменных, зарегистрированных как
        'Импульс-переменные'.

        Returns:
            Set[str]: Множество имён переменных.

        Examples:
            >>> print(vars.var_names)
            ... # {'x', 'y', 'counter'}
        """
        return ImpulseBridge.get_impulse_var_names().copy()

    def _is_valid_name(self, name: str) -> bool:
        """
        Проверяет, является ли строка допустимым публичным именем переменной в Python:
            - корректный идентификатор,
            - не ключевое слово,
            - не начинается с подчёркивания (публичное имя).
        """
        return (
            isinstance(name, str)
            and name.isidentifier()
            and not keyword.iskeyword(name)
            and not name.startswith("_")
        )

    def __getattr__(self, name: str) -> Any:
        if name in ImpulseBridge.get_impulse_var_names():
            if name in self._data:
                return self._data[name]
            raise AttributeError(
                f"Variable '{name}' is registered in Impulse but not loaded in this proxy"
            )
        raise AttributeError(
            f"'{name}' not found in Impulse variables (available: "
            f"{sorted(ImpulseBridge.get_impulse_var_names())})"
        )

    def __setattr__(self, name: str, value: Any) -> None:
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        if not self._is_valid_name(name):
            raise ValueError(f"Invalid variable name: '{name}'")

        if not self._auto_add_vars and not self.has_var(name):
            raise AttributeError(
                f"Cannot set '{name}': not registered and auto_add_vars=False. "
                f"Use impulse_vars(auto_add_vars=True) or add_impulse_vars()."
            )

        self._data[name] = value
        if self._auto_add_vars:
            ImpulseBridge.add_impulse_var(name)

    def __dir__(self):
        return sorted(ImpulseBridge.get_impulse_var_names())

    def __str__(self) -> str:
        """Возвращает строковое представление всех Impulse-переменных и их значений."""
        var_names = ImpulseBridge.get_impulse_var_names()
        if not var_names:
            return "ImpulseVarsProxy(<no variables>)"

        items = []
        for name in sorted(var_names):
            value = self._data.get(name, "<not loaded>")
            value_repr = repr(value)
            if len(value_repr) > 100:
                value_repr = value_repr[:97] + "..."
            items.append(f"{name} = {value_repr}")
        return "ImpulseVarsProxy(\n    " + "\n    ".join(items) + "\n)"

    def update(self, data: dict[str, Any]) -> None:
        """Пакетное обновление переменных (вызывается автоматически при load)."""
        for k, v in data.items():
            self.__setattr__(k, v)

    def to_dict(self) -> dict[str, Any]:
        """Возвращает копию данных для сериализации (вызывается автоматически при save)."""
        registered = ImpulseBridge.get_impulse_var_names()
        return {k: v for k, v in self._data.items() if k in registered}

    def __getitem__(self, name: str) -> Any:
        return self.__getattr__(name)

    def __setitem__(self, name: str, value: Any) -> None:
        self.__setattr__(name, value)

    def keys(self) -> KeysView[str]:
        return self._data.keys()

    def items(self) -> ItemsView[str, Any]:
        return self._data.items()

    def values(self) -> ValuesView[Any]:
        return self._data.values()

    def __contains__(self, name: str) -> bool:
        return name in self._data
