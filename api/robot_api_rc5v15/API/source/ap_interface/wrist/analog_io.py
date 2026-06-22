from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    WristStateError,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.tools import sleep
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    SetWristInputOutputTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    WristMode,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    PowerUnitsCode,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    AMPERAGE_VALUES_RANGE,
    CHECK_FREQUENCY_SEC,
    REAL_WRIST_MAX_AN_IN,
    SET_WRIST_MODE_AWAIT_SEC,
    VOLTAGE_VALUES_RANGE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AnalogWristIndex,
    CompareSigns,
    PowerUnits,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class WristAnalogIO(LoggerMixin):
    """
    Класс для работы с аналоговыми входами платы запястья робота.

    Все методы требуют, чтобы плата запястья находилась в режиме 'analog_in'.
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver

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

    def _check_wrist_mode(self):
        mode = self._rtd_receiver.get_data().wrist_mode
        if mode != WristMode.analog_in:
            raise WristStateError(
                f"Can not use method when wrist mode is {WristMode(mode).name}"
            )

    @handle_connection(available_in_read_only=False)
    def configure_input(
        self, index: AnalogWristIndex, units: PowerUnits
    ) -> bool:
        """
        Настраивает единицы измерения для указанного аналогового входа.

        Метод отправляет команду контроллеру для переключения входа
        в режим измерения напряжения ('V') или тока ('mA').

        Args:
            index (AnalogWristIndex): Индекс аналогового входа (0 или 1).
            units (PowerUnits): Единицы измерения: 'V' (0–10 В) или 'mA' (4–20 мА).

        Returns:
            bool: True, если конфигурация успешно применена; False в случае таймаута.

        Raises:
            WristStateError: Если плата запястья не в режиме 'analog_in'.

        Examples:
            >>> # Сконфигурировать вход 0 на напряжение
            >>> result = robot.wrist.analog.configure_input(0, 'V')
            >>> if result:
            ...     print("Вход сконфигурирован на измерение напряжения")
        """
        self._check_wrist_mode()
        return self._set_input_units(index, units)

    @handle_connection(available_in_read_only=True)
    def get_input(
        self,
        index: AnalogWristIndex,
    ) -> tuple[float, PowerUnits]:
        """Считывает текущее аналоговое значение с указанного входа платы запястья.

        Метод возвращает мгновенное значение, уже преобразованное в соответствии
        с текущей конфигурацией входа (установленной через configure_input).

        Метод доступен в режиме «read only».

        Args:
            index (AnalogWristIndex): Индекс аналогового входа. Допустимые значения: `0`–`1`.

        Returns:
            Tuple[float, PowerUnits]:
                - Кортеж `(значение, единицы измерения)`, если чтение успешно:
                    - для `'V'`: значение в диапазоне **0.0 – 10.0 В**;
                    - для `'mA'`: значение в диапазоне **4.0 – 20.0 мА**.
        Raises:
            WristStateError: Если плата запястья не в режиме 'analog_in'.

        Examples:
            >>> value, units = robot.wrist.analog.get_input(0)
            >>> print(f"Вход 0: {value:.2f} {units}")

        Notes:
            - Метод не блокирует выполнение и возвращает мгновенное значение.
        """
        self._check_wrist_mode()
        value = self._rtd_receiver.get_data().wrist_an_in_value[index]
        units = self._rtd_receiver.get_data().wrist_an_in_curr_mode[index]

        units = PowerUnitsCode(units).name
        if units == "mA":
            value *= 1000

        return (value, units)

    @handle_connection(available_in_read_only=False)
    def get_input_in_units(
        self, index: AnalogWristIndex, units: PowerUnits
    ) -> float | None:
        """Считывает текущее аналоговое значение с указанного входа платы
        запястья в указанных единицах.

        Метод позволяет получить мгновенное значение с одного из аналоговых входов
        в указанных единицах — например, для считывания показаний датчиков давления,
        уровня, температуры или других устройств с аналоговым выходом.

        Args:
            index (AnalogWristIndex): Индекс аналогового входа. Допустимые значения: `0`–`1`.
            units (PowerUnits): Единицы измерения сигнала:
                - `'V'` — ожидается напряжение (вход должен быть подключен к источнику 0–10 В);
                - `'mA'` — ожидается ток (вход должен быть подключен к источнику 4–20 мА).

        Returns:
            Optional[float]:
                - Значение сигнала на входе:
                    - для `'V'`: значение в диапазоне **0.0 – 10.0 В**;
                    - для `'mA'`: значение в диапазоне **4.0 – 20.0 мА**.
                - `None`, если плата запястья отсутствует, вход недоступен или
                    произошла ошибка.
        Raises:
            WristStateError: Если плата запястья не в режиме 'analog_in'.

        Examples:
            >>> # Считать напряжение с входа 0
            >>> voltage = robot.wrist.analog.get_input_in_units(0, 'V')
            >>> if voltage:
            ...     print(f"Вход 0: {voltage:.2f} В")

            >>> # Считать ток с входа 1
            >>> current = robot.wrist.analog.get_input_in_units(1, 'mA') or 0.0
            >>> if current >= 12.0:
            ...     print("Сигнал выше порога")

        Notes:
            - Метод не блокирует выполнение и возвращает мгновенное значение.
        """
        if not self.configure_input(index, units):
            return None
        value, _ = self.get_input(index)
        return value

    def _set_input_units(
        self,
        index: AnalogWristIndex,
        units: PowerUnits,
        await_sec: int | float = SET_WRIST_MODE_AWAIT_SEC,
    ) -> bool:
        """
        Установить единицы измерения, снимаемые на 'index' аналоговом входе
        платы запястья. Тип устанавливаемого значения зависит от переменной
        units.

        Args:
            index: Индекс выхода (0-1).
            units: Единицы измерения. 'mA' - сила тока. 'V' - напряжение.
        Returns:
            True: В случае успешной отправки команды.
            False: В случае таймаута или неудачной отправки команды.
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not set input units, when wrist mode is {WristMode.off}"
            )
        Validation.literal("power", units)
        Validation.index(index, range(REAL_WRIST_MAX_AN_IN))

        if (
            self._rtd_receiver.get_data().wrist_an_in_curr_mode[index]
            == PowerUnitsCode[units]
        ):
            return True

        set_input_output_template = SetWristInputOutputTemplate()
        set_input_output_template.an_in_mask[index] = 1
        set_input_output_template.an_in_mode[index] = PowerUnitsCode[units]
        set_input_output_template.mux_mode = WristMode.analog_in
        res = self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_WRIST_IO,
            set_input_output_template,
        )
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error", "Failed to set input units - connection was lost"
                )
                return False
            if (
                self._rtd_receiver.get_data().wrist_an_in_curr_mode[index]
                == PowerUnitsCode[units]
            ):
                return res
        return False

    @handle_connection(available_in_read_only=True)
    def wait_input(
        self,
        index: AnalogWristIndex,
        threshold_value: float,
        greater_or_less: CompareSigns,
        await_sec: int | float = -1,
    ) -> bool:
        """Ожидает, пока аналоговый вход платы запястья не преодолеет заданное пороговое значение.

        Метод ожидает преодоления значения, уже преобразованного в соответствии
        с текущей конфигурацией входа (установленной через configure_input).

        Метод позволяет синхронизировать выполнение программы с внешними аналоговыми
        сигналами — например, дождаться, пока датчик давления (4–20 мА) не достигнет
        заданного уровня или пока управляющее напряжение (0–10 В) не упадёт ниже порога.

        Метод доступен в режиме «read only».

        Args:
            index (AnalogWristIndex): Индекс аналогового входа. Допустимые значения: `0`–`1`.
            threshold_value (float): Пороговое значение:
                - при `units='V'`: от **0.0 до 10.0 В**;
                - при `units='mA'`: от **4.0 до 20.0 мА**.
            greater_or_less (CompareSigns): Условие ожидания:
                - `'>'` — ждать, пока значение **станет больше** порога;
                - `'<'` — ждать, пока значение **станет меньше** порога.
            await_sec (int): Лимит времени ожидания в секундах:
                - `-1` — ожидание без ограничения (по умолчанию);
                - `0` — однократная проверка без блокировки;
                - положительное число — максимальное время ожидания в секундах.

        Returns:
            bool:
                - `True`, если пороговое значение было преодолено в течение указанного времени;
                - `False`, если произошёл тайм-аут (только при `await_sec >= 0`).

        Raises:
            WristStateError: Если плата запястья не в режиме 'analog_in'.
            ArgIndexError: Если `index` выходит за допустимый диапазон (0–1).
            ArgComparisonError: Если `greater_or_less` не равен `'>'` или `'<'`.

        Examples:
            >>> # Дождаться, пока напряжение на входе 0 превысит 7.5 В (макс. 10 сек)
            >>> robot.wrist.analog.configure_input(0, 'V')
            >>> if robot.wrist.analog.wait_input(0, 7.5, '>', await_sec=10):
            ...     print("Уровень напряжения достигнут")

            >>> # Бесконечно ждать, пока значение на входе не поднимется
            ... # выше 16 (мА или В в зависимости от конфигурации)
            >>> robot.wrist.analog.wait_input(0, 16.0, '>')

        Notes:
            - Метод блокирует выполнение программы.
        """
        self._check_wrist_mode()
        Validation.index(index, range(REAL_WRIST_MAX_AN_IN))
        Validation.value(
            threshold_value,
            (
                VOLTAGE_VALUES_RANGE
                if index in range(REAL_WRIST_MAX_AN_IN)
                else AMPERAGE_VALUES_RANGE
            ),
            "threshold_value",
        )
        Validation.literal("compare", greater_or_less)
        self._write_log(
            "info", f"Waiting analog signal on {index} wrist input..."
        )
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to wait wrist analog input - connection was lost",
                )
                return False
            actual_value, actual_units = self.get_input(index)
            if actual_value is None:
                continue
            if greater_or_less == "<":
                if actual_value <= threshold_value:
                    self._write_log(
                        "info",
                        (
                            f"Analog signal on wrist {index}: "
                            f"{actual_value} < "
                            f"{threshold_value} ({actual_units})"
                        ),
                    )
                    return True
            elif greater_or_less == ">":
                if actual_value >= threshold_value:
                    self._write_log(
                        "info",
                        (
                            f"Analog signal on wrist {index}: "
                            f"{actual_value} > "
                            f"{threshold_value} ({actual_units})"
                        ),
                    )
                    return True
        self._write_log(
            "warning", f"Analog signal on {index} wrist input timeout"
        )
        return False

    @handle_connection(available_in_read_only=False)
    def wait_input_in_units(
        self,
        index: AnalogWristIndex,
        threshold_value: float,
        units: PowerUnits,
        greater_or_less: CompareSigns,
        await_sec: int | float = -1,
    ) -> bool:
        """Ожидает, пока аналоговый вход платы запястья не преодолеет заданное
        пороговое значение в заданных единицах.

        Метод позволяет синхронизировать выполнение программы с внешними аналоговыми
        сигналами — например, дождаться, пока датчик давления (4–20 мА) не достигнет
        заданного уровня или пока управляющее напряжение (0–10 В) не упадёт ниже порога.

        Args:
            index (AnalogWristIndex): Индекс аналогового входа. Допустимые значения: `0`–`1`.
            threshold_value (float): Пороговое значение:
                - при `units='V'`: от **0.0 до 10.0 В**;
                - при `units='mA'`: от **4.0 до 20.0 мА**.
            units (PowerUnits): Тип измеряемого сигнала:
                - `'V'` — напряжение (вход должен быть подключён к источнику 0–10 В);
                - `'mA'` — сила тока (вход должен быть подключён к источнику 4–20 мА).
            greater_or_less (CompareSigns): Условие ожидания:
                - `'>'` — ждать, пока значение **станет больше** порога;
                - `'<'` — ждать, пока значение **станет меньше** порога.
            await_sec (int): Лимит времени ожидания в секундах:
                - `-1` — ожидание без ограничения (по умолчанию);
                - `0` — однократная проверка без блокировки;
                - положительное число — максимальное время ожидания в секундах.

        Returns:
            bool:
                - `True`, если пороговое значение было преодолено в течение указанного времени;
                - `False`, если произошёл тайм-аут (только при `await_sec >= 0`).

        Raises:
            WristStateError: Если плата запястья не в режиме 'analog_in'.
            ArgIndexError: Если `index` выходит за допустимый диапазон (0–1).
            ArgComparisonError: Если `greater_or_less` не равен `'>'` или `'<'`.

        Examples:
            >>> # Дождаться, пока напряжение на входе 0 превысит 7.5 В (макс. 10 сек)
            >>> if robot.wrist.analog.wait_input(0, 7.5, 'V', '>', await_sec=10):
            ...     print("Уровень напряжения достигнут")

            >>> # Бесконечно ждать, пока ток не поднимется выше 16 мА
            >>> robot.wrist.analog.wait_input(0, 16.0, 'mA', '>')

        Notes:
            - Метод блокирует выполнение программы.
        """
        self.configure_input(index, units)
        return self.wait_input(
            index=index,
            threshold_value=threshold_value,
            greater_or_less=greater_or_less,
            await_sec=await_sec,
        )
