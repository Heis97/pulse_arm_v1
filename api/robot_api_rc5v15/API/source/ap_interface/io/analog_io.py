from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.tools import (
    sleep,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    SetOutputTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    AMPERAGE_VALUES_RANGE,
    AVAILABLE_AN_IN_INDEX_COUNT,
    AVAILABLE_AN_OUT_INDEX_COUNT,
    CHECK_FREQUENCY_SEC,
    REAL_WRIST_MAX_AN_IN,
    VOLTAGE_VALUES_RANGE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AnalogIndex,
    CompareSigns,
    PowerUnits,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class AnalogIO(LoggerMixin):
    """
    Класс для работы с аналоговыми входами/выходами контроллера робота.
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver
    _logger: Logger | None

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: Logger | None,
    ):
        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._connection_state = connection_state
        self._set_logger(logger)

    @handle_connection(available_in_read_only=True)
    def get_input(self, index: AnalogIndex) -> tuple[int, float]:
        """Возвращает текущее значение аналогового входа робота.

        Метод считывает мгновенное значение напряжения или силы тока
        с указанного аналогового входа. Входы 0–1 предназначены для измерения
        напряжения (в вольтах), входы 2–3 — для измерения тока (в миллиамперах).

        Метод доступен в режиме «read only».

        Args:
            index (AnalogIndex): Индекс аналогового входа. Допустимые значения:
                - 0, 1 — входы напряжения (0–10 В);
                - 2, 3 — входы тока (4–20 мА).

        Returns:
            Tuple[int, float]: Кортеж вида `(индекс, значение)`, где:
                - `индекс` — переданный номер входа;
                - `значение` — измеренное значение:
                    * для входов 0–1: напряжение в **вольтах (В)**;
                    * для входов 2–3: сила тока в **миллиамперах (мА)**.

        Examples:
            >>> # Считать напряжение с входа 0
            >>> idx, voltage = robot.io.analog.get_input(0)
            >>> print(f"Вход {idx}: {voltage:.2f} В")

            >>> # Считать ток с входа 2
            >>> idx, current = robot.io.analog.get_input(2)
            >>> print(f"Вход {idx}: {current:.1f} мА")

            >>> # Использовать в логике управления
            >>> _, pressure = robot.io.analog.get_input(2)  # давление через датчик 4-20 мА
            >>> if pressure < 5.0:
            ...     print("Низкое давление в системе!")
        """

        Validation.index(index, range(AVAILABLE_AN_IN_INDEX_COUNT))

        value = self._rtd_receiver.get_data().an_in_value[index]
        if index in (2, 3):
            value *= 1000
        return (index, value)

    @handle_connection(available_in_read_only=False)
    def set_output(
        self, index: AnalogIndex, value: float, units: PowerUnits
    ) -> bool:
        """Устанавливает значение аналогового выхода робота — напряжение или ток.

        Метод позволяет управлять внешними устройствами через аналоговые сигналы:
        например, задавать скорость привода (0–10 В), управлять клапаном (4–20 мА)
        или регулировать яркость лампы.

        Выходы 0–3 поддерживают **либо напряжение, либо ток**, в зависимости от
        аппаратной конфигурации и указанной единицы измерения.

        Args:
            index (int): Индекс аналогового выхода. Допустимый диапазон: `0`–`3`.
            value (float): Устанавливаемое значение:
                - при `units='V'`: напряжение в диапазоне **0.0 – 10.0 В**;
                - при `units='mA'`: ток в диапазоне **4.0 – 20.0 мА**.
            units (PowerUnits): Тип сигнала:
                - `'V'` — напряжение (вольты);
                - `'mA'` — сила тока (миллиамперы).

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Установить 7.5 В на выход 0 (управление частотным преобразователем)
            >>> robot.io.analog.set_output(0, 7.5, 'V')

            >>> # Подать 12 мА на выход 2 (управление пневмоклапаном)
            >>> robot.io.analog.set_output(2, 12.0, 'mA')

        Notes:
            - Значения вне указанных диапазонов будут проигнорированы или вызовут ошибку.
        """

        Validation.literal("power", units)
        Validation.index(index, range(AVAILABLE_AN_OUT_INDEX_COUNT))
        Validation.value(
            value,
            AMPERAGE_VALUES_RANGE if units == "mA" else VOLTAGE_VALUES_RANGE,
            "value",
        )
        set_output_template = SetOutputTemplate()
        set_output_template.set_analog_output(
            index=index, value=value, units=units
        )
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_OUTPUTS, set_output_template
        )

    @handle_connection(available_in_read_only=True)
    def wait_input(
        self,
        index: AnalogIndex,
        threshold_value: float,
        greater_or_less: CompareSigns,
        await_sec: int | float = -1,
    ) -> bool:
        """Ожидает, пока аналоговый вход не преодолеет заданное пороговое значение.

        Метод блокирует выполнение до тех пор, пока значение на указанном аналоговом
        входе не станет больше или меньше заданного порога (в зависимости от
        параметра `greater_or_less`). Поддерживается как ограниченное, так и
        неограниченное по времени ожидание.

        Метод доступен в режиме «read only».

        Args:
            index: Индекс аналогового входа (допустимые значения: 0–3).
                - Входы 0–1 соответствуют измерению напряжения (в вольтах, диапазон 0–10 В).
                - Входы 2–3 соответствуют измерению тока (в миллиамперах, диапазон 4–20 мА).
            threshold_value: Пороговое значение:
                - Для напряжения (входы 0–1): от 0.0 до 10.0 В.
                - Для тока (входы 2–3): от 4.0 до 20.0 мА.
            greater_or_less: Направление сравнения:
                - `'>'`: ожидать, пока значение не станет **строго больше** порога.
                - `'<'`: ожидать, пока значение не станет **строго меньше** порога.
            await_sec: Максимальное время ожидания в секундах.
                - `-1`: ожидание без ограничения по времени (по умолчанию).
                - `0`: проверка выполняется ровно один раз (без блокировки).
                - Положительное значение: максимальное время ожидания в секундах.

        Returns:
            bool:
                - `True`, если пороговое значение было преодолено в течение времени ожидания.
                - `False`, если произошёл тайм-аут (только если `await_sec >= 0`).

        Examples:
            >>> # Дождаться, пока напряжение на входе 0 превысит 5 В (макс. 10 сек)
            >>> robot.io.analog.wait_input(0, 5.0, '>', await_sec=10)

            >>> # Проверить однократно, опустилось ли значение тока на входе 2 ниже 8 мА
            >>> robot.io.analog.wait_input(2, 8.0, '<', await_sec=0)

            >>> # Бесконечно ждать, пока напряжение на входе 1 не упадёт ниже 1 В
            >>> robot.io.analog.wait_input(1, 1.0, '<')

        Notes:
            - Метод может блокировать выполнение программы, особенно при `await_sec = -1`.
            - Некорректный `index` или `greater_or_less` вызывают исключения
              `ArgIndexError` и `ArgComparisonError` соответственно.
        """

        Validation.index(index, range(AVAILABLE_AN_IN_INDEX_COUNT))
        Validation.value(
            threshold_value,
            VOLTAGE_VALUES_RANGE
            if index in range(REAL_WRIST_MAX_AN_IN)
            else AMPERAGE_VALUES_RANGE,
            "threshold_value",
        )
        Validation.literal("compare", greater_or_less)
        self._write_log("info", f"Waiting analog signal on {index} input...")
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    f"Failed to wait analog input {index} - connection was lost",
                )
                return False
            if greater_or_less == "<":
                if self.get_input(index)[1] <= threshold_value:
                    self._write_log(
                        "info",
                        (
                            f"Analog signal on {index}: "
                            f"{self.get_input(index)[1]} < "
                            f"{threshold_value}"
                        ),
                    )
                    return True
            elif greater_or_less == ">":
                if self.get_input(index)[1] >= threshold_value:
                    self._write_log(
                        "info",
                        (
                            f"Analog signal on {index}: "
                            f"{self.get_input(index)[1]} > "
                            f"{threshold_value}"
                        ),
                    )
                    return True
        self._write_log("info", f"Analog signal on {index} input timeout")
        return False
