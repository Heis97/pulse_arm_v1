from __future__ import annotations

from dataclasses import dataclass

from ..enum_classes.rs_485_structures import Rs485Commands, Rs485ReturnCodes


@dataclass
class Rs485Frame:
    """Структура, представляющая один логический фрейм ответа от RS-485-сервиса робота.

    Инкапсулирует результат выполнения команды: тип операции, код возврата и, при наличии,
    полезные данные. Используется внутренними методами `RS485Protocol` для унификации
    обработки входящих сообщений.

    Attributes:
        command (Rs485Commands): Тип команды, на которую получен ответ.
        return_code (Rs485ReturnCodes): Результат выполнения операции.
            По умолчанию — `ok`.
        data (bytes): Полезная нагрузка (например, принятые данные от устройства).
            По умолчанию — пустой байтовый объект.
    """

    command: Rs485Commands
    return_code: Rs485ReturnCodes = Rs485ReturnCodes.ok
    data: bytes = b""

    @classmethod
    def from_response(cls, command_id: int, payload: bytes) -> Rs485Frame:
        """
        Создаёт экземпляр фрейма из 'сырого' ответа RS-485-сервиса.

        Метод парсит байтовый ответ, определяет тип команды и интерпретирует
        payload в зависимости от контекста:

        - Для команды `received_data` весь payload считается данными.
        - Для остальных команд:
            - если payload длиной 1 байт — он интерпретируется как код возврата;
            - иначе — сохраняется как `data`.

        Args:
            command_id (int): Числовой идентификатор команды из заголовка фрейма.
            payload (bytes): Тело ответа, полученное от сервиса.

        Returns:
            Экземпляр Rs485Frame.
        """
        try:
            command = Rs485Commands(command_id)
        except Exception:
            command = Rs485Commands.unknown

        # Определяем return_code
        if command == Rs485Commands.received_data:
            return cls(command=command, data=payload)
        else:
            if len(payload) == 1:
                try:
                    return_code = Rs485ReturnCodes(payload[0])
                except ValueError:
                    return_code = Rs485ReturnCodes.failure
                return cls(command=command, return_code=return_code)
            else:
                # Нестандартный payload — сохраняем как data
                return cls(command=command, data=payload)

    def __str__(self) -> str:
        return (
            f"Rs485Frame(command={self.command.name}, "
            f"return_code={self.return_code.name}, "
            f"data={self.data!r})"
        )

    def __repr__(self) -> str:
        return (
            f"Rs485Frame(command={self.command.name}, "
            f"return_code={self.return_code.name}, "
            f"data={self.data!r})"
        )


@dataclass(frozen=True)
class Response:
    """Стандартизированный ответ от RS-485 интерфейса.

    Представляет результат выполнения операции (`write`, `read`, `reset` и др.)
    в удобной, неизменяемой форме. Содержит код возврата и, при наличии,
    полученные данные.

    Attributes:
        return_code (Rs485ReturnCodes): Код результата операции.
            Значение `ok` означает успешное выполнение.
        raw_data (bytes): Полученные данные (если операция подразумевает передачу данных).
            Может быть пустым, если операция не возвращает полезную нагрузку
            (например, `reset` или `write`).
    """

    return_code: Rs485ReturnCodes
    raw_data: bytes

    @property
    def is_ok(self) -> bool:
        """Проверяет, завершилась ли операция успешно.

        Returns:
            bool: `True`, если `return_code == Rs485ReturnCodes.ok`.
        """
        return self.return_code == Rs485ReturnCodes.ok

    @property
    def has_error(self) -> bool:
        """Проверяет, произошла ли ошибка при выполнении операции.

        Returns:
            bool: `True`, если `return_code` указывает на ошибку.
        """
        return not self.is_ok

    @classmethod
    def from_frame(cls, frame: Rs485Frame) -> Response:
        """Создаёт публичный ответ на основе внутреннего фрейма (служебный метод).

        Используется адаптером `RS485Protocol` для преобразования
        низкоуровневого представления в интерфейс, понятный пользователю.

        Args:
            frame (Rs485Frame): Внутренний фрейм, полученный от сервиса.

        Returns:
            Response: Публичный объект ответа.
        """
        return Response(return_code=frame.return_code, raw_data=frame.data)

    def __str__(self) -> str:
        """
        Возвращает человекочитаемое представление ответа.
        """
        status = "OK" if self.is_ok else f"ERROR ({self.return_code.name})"
        data_repr = (
            self.raw_data
            if len(self.raw_data) <= 32
            else self.raw_data[:29] + b"..."
        )
        return f"Response({status}, data={data_repr!r})"

    def __repr__(self) -> str:
        """
        Возвращает однозначное представление объекта для отладки.
        """
        return f"Response(return_code={self.return_code.name}, raw_data={self.raw_data!r})"
