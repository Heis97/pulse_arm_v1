from __future__ import annotations

from collections.abc import Iterable
from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.generic_error import ApiValueError
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
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.io_functions import (
    InputFunction,
    OutputFunction,
    SafetyInputFunctions,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    AVAILABLE_DIG_IN_INDEX_COUNT,
    AVAILABLE_DIG_OUT_INDEX_COUNT,
    BYTE_SIZE,
    CHECK_FREQUENCY_SEC,
    DIG_IN_INDEX_RANGE,
    DIG_SAFETY_IN_INDEX_RANGE,
    NO_FUNC_ANSWER_VALUE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    DigitalIndex,
    DigitalSafetyIndex,
    InputFunctionName,
    OutputFunctionName,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class DigitalIO(LoggerMixin):
    """
    Класс для работы с цифровыми входами/выходами контроллера робота.
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver
    _connection_state: ConnectionState

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

    def _get_input(self, index: int, inputs_range: Iterable[int]) -> bool:
        """
        Получить текущее двоичное значение на 'index' цифровом входе.

        Args:
            index: Индекс входа.
        Returns:
            bool: True — есть сигнал, False — нет сигнала.
        """
        Validation.index(index, inputs_range)
        byte_index = int(index // BYTE_SIZE)
        mask = 1 << (index % BYTE_SIZE)
        return self._rtd_receiver.get_data().dig_in[byte_index] & mask != 0

    @handle_connection(available_in_read_only=True)
    def get_input(self, index: DigitalIndex) -> bool:
        """Возвращает текущее состояние цифрового входа робота.

        Метод считывает двоичный сигнал («высокий»/«низкий») с указанного
        цифрового входа. Используется для мониторинга состояния внешних устройств:
        датчиков, кнопок, концевых выключателей и т.п.

        Метод доступен в режиме «read only».

        Args:
            index (DigitalIndex): Индекс цифрового входа. Допустимый диапазон:
                от `0` до `23` (всего 24 входа).

        Returns:
            bool:
                - `True` — на входе присутствует активный сигнал («высокий уровень»);
                - `False` — сигнал отсутствует («низкий уровень»).

        Examples:
            >>> # Проверить состояние датчика наличия детали (вход 5)
            >>> if robot.io.digital.get_input(5):
            ...     print("Деталь на месте")
            ... else:
            ...     print("Деталь отсутствует")

            >>> # Опрос нескольких входов
            >>> emergency_stop = robot.io.digital.get_input(0)
            >>> door_open = robot.io.digital.get_input(1)
            >>> if emergency_stop or door_open:
            ...     robot.motion.mode.set('hold')

        Notes:
            - Состояние возвращается **мгновенно** — метод не блокирует выполнение.
            - Для реакции на **изменения** состояния рекомендуется использовать
              `wait_input()` или `wait_any_input()` вместо опроса в цикле.
        """

        return self._get_input(index=index, inputs_range=DIG_IN_INDEX_RANGE)

    @handle_connection(available_in_read_only=True)
    def get_safety_input(self, index: DigitalSafetyIndex) -> bool:
        """Возвращает текущее состояние цифрового входа безопасности.

        Эти входы предназначены для подключения критически важных устройств
        безопасности: кнопок аварийной остановки, защитных дверей, световых завес,
        двухручных панелей и других компонентов системы безопасности.

        Метод доступен в режиме «read only».

        Args:
            index (DigitalSafetyIndex): Индекс входа безопасности.
                Допустимый диапазон: от `0` до `7` (всего 8 входов).
                Конкретное назначение каждого входа определяется схемой подключения
                и конфигурацией контроллера.

        Returns:
            bool:
                - `True` — на входе присутствует активный сигнал;
                - `False` — сигнал отсутствует.

        Examples:
            >>> # Проверить состояние защитной двери (вход 0)
            >>> if not robot.io.digital.get_safety_input(0):
            ...     print("Внимание: защитная дверь открыта!")
            ...     # Возможно, требуется остановить робота

            >>> # Проверить наличие сигнала с двуручной панели (вход 2)
            >>> if robot.io.digital.get_safety_input(2):
            ...     print("Двуручная панель активна — можно продолжить")

        Notes:
            - Логика сигнала **может быть инвертирована** в зависимости от
              конфигурации безопасности.
        """
        return self._get_input(
            index=index + AVAILABLE_DIG_IN_INDEX_COUNT,
            inputs_range=DIG_SAFETY_IN_INDEX_RANGE,
        )

    @handle_connection(available_in_read_only=False)
    def get_safety_input_functions(
        self,
    ) -> tuple[tuple[int, str], ...]:
        """Возвращает текущие назначения функций для цифровых входов безопасности.

        Каждый вход безопасности может быть привязан к определённой функции
        безопасности.
        Эти назначения определяют, как контроллер реагирует на изменение состояния
        входа (например, открытие двери или нажатие кнопки).

        Returns:
            Tuple[Tuple[int, str], ...]: Кортеж пар вида `(номер_входа, функция)`,
            где:
            - `номер_входа` (int) — индекс входа безопасности (0–7);
            - `функция` (str) — строковое имя функции безопасности.

        Examples:
            >>> # Получить все назначения функций
            >>> funcs = robot.io.digital.get_safety_input_functions()
            >>> for index, func_name in funcs:
            ...     print(f"Вход {index}: {func_name}")
        """
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_CBOX_GET_SFTY_INPUT_FUNC
        )
        result = []
        for index, value in enumerate(response):
            result.append((index, SafetyInputFunctions(value).name))
        return tuple(result)

    @handle_connection(available_in_read_only=True)
    def get_output(self, index: DigitalIndex) -> bool:
        """Возвращает текущее состояние цифрового выхода контроллера робота.

        Метод позволяет считать фактическое состояние. Метод доступен в режиме
        «read only».

        Args:
            index (DigitalIndex): Индекс цифрового выхода.
                Допустимый диапазон: от `0` до `23` (всего 24 выхода).

        Returns:
            bool:
                - `True` — на выходе активен сигнал («высокий уровень»);
                - `False` — сигнал отсутствует («низкий уровень»).

        Examples:
            >>> # Проверить, включён ли индикатор готовности (выход 5)
            >>> if robot.io.digital.get_output(5):
            ...     print("Робот готов к работе")
            ... else:
            ...     print("Ожидание инициализации")

            >>> # Логировать состояние нескольких выходов
            >>> vacuum_on = robot.io.digital.get_output(10)  # управление вакуумом
            >>> gripper_closed = robot.io.digital.get_output(11)  # сигнал захвата
            >>> print(f"Вакуум: {vacuum_on}, Захват: {gripper_closed}")

        Notes:
            - Для **управления** выходом используйте `set_output()`.
        """

        Validation.index(index, range(AVAILABLE_DIG_OUT_INDEX_COUNT))
        byte_index = int(index / BYTE_SIZE)
        mask = 1 << (index % BYTE_SIZE)
        return self._rtd_receiver.get_data().dig_out[byte_index] & mask != 0

    @handle_connection(available_in_read_only=False)
    def set_output(self, index: DigitalIndex, value: bool) -> bool:
        """Устанавливает состояние цифрового выхода робота.

        Метод отправляет команду на активацию или деактивацию указанного
        цифрового выхода. Используется для управления внешними устройствами:
        реле, клапанами, индикаторами, захватами, вакуумными системами и т.п.

        Args:
            index (DigitalIndex): Индекс цифрового выхода.
                Допустимый диапазон: от `0` до `23` (всего 24 выхода).
                Конкретная нумерация зависит от аппаратной конфигурации контроллера.
            value (bool): Желаемое состояние выхода:
                - `True` — активировать выход («высокий уровень»);
                - `False` — деактивировать выход («низкий уровень»).

        Returns:
            bool: True — если команда успешно отправлена контроллеру.

        Examples:
            >>> # Включить вакуумный захват (выход 10)
            >>> robot.io.digital.set_output(10, True)

            >>> # Выключить сигнальную лампу (выход 5)
            >>> robot.io.digital.set_output(5, False)

        Notes:
            - Успешный возврат `True` означает **только отправку команды**,
              а не подтверждение физического переключения выхода.
            - Для **чтения** текущего состояния используйте `get_output()`.
        """

        Validation.index(index, range(AVAILABLE_DIG_OUT_INDEX_COUNT))
        set_output_template = SetOutputTemplate()
        set_output_template.set_digital_output(index, value)
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_OUTPUTS, set_output_template
        )

    @handle_connection(available_in_read_only=False)
    def pulse_output(
        self,
        index: DigitalIndex,
        duration: float,
        active_high: bool = True,
    ) -> bool:
        """Генерирует одиночный импульс на цифровом выходе.

        Метод устанавливает выход в активное состояние, выдерживает заданную
        длительность, затем возвращает выход в неактивное состояние.

        **Метод блокирует вызывающий поток** на время `duration`.
        Для асинхронных сценариев используйте вызов в отдельном потоке.

        **Полярность импульса:**
        - `active_high=True` (по умолчанию): положительный импульс
          `0 --> 1 --> 0` (низкий --> высокий --> низкий);
        - `active_high=False`: отрицательный импульс
          `1 --> 0 --> 1` (высокий --> низкий --> высокий).

        Args:
            index (DigitalIndex): Индекс выхода (0–N).
            duration (float): Длительность импульса в секундах (≥ 0).
            active_high (bool, optional): Полярность активного состояния:
                - `True` — активный высокий уровень (положительный импульс);
                - `False` — активный низкий уровень (отрицательный импульс).

        Returns:
            bool: `True`, если обе команды (включение/выключение) успешно
                отправлены контроллеру; `False`, если хотя бы одна операция
                завершилась с ошибкой.

        Raises:
            ValueError: Если `duration` отрицательный.

        Examples:
            >>> # Положительный импульс 100 мс на выходе 3
            >>> success = robot.io.digital.pulse_output(3, duration=0.1)

            >>> # Отрицательный импульс 50 мс (активный низкий) на выходе 0
            >>> success = robot.io.digital.pulse_output(
            ...     0, duration=0.05, active_high=False
            ... )
        """
        if duration < 0:
            raise ApiValueError(
                f"Duration must be non-negative, got {duration}"
            )

        Validation.index(index, range(AVAILABLE_DIG_OUT_INDEX_COUNT))

        if not self.set_output(index, active_high):
            return False

        for _ in sleep(duration):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to pulse digital output - connection was lost",
                )
                return False

        return self.set_output(index, not active_high)

    @handle_connection(available_in_read_only=True)
    def wait_any_input(self, await_sec: int | float = -1) -> bool:
        """Блокирует выполнение до тех пор, пока не изменится состояние хотя бы одного цифрового входа.

        Метод полезен для сценариев, где важно отреагировать на **любое внешнее событие**,
        не привязываясь к конкретному датчику: например, ожидание прерывания от оператора,
        срабатывания любого из нескольких датчиков или сигнала тревоги.

        Метод доступен в режиме «read only».

        Args:
            await_sec (int, optional): Максимальное время ожидания в секундах:
                - `-1` — ожидание без ограничения по времени (по умолчанию);
                - `0` — неблокирующая проверка: вернуть результат немедленно;
                - `> 0` — ожидать не более указанного числа секунд.

        Returns:
            bool:
                - `True` — если произошло изменение на одном или нескольких
                  цифровых входах (0–23) в течение заданного времени;
                - `False` — если ни один вход не изменил состояние до истечения
                  таймаута (`await_sec >= 0`).

        Examples:
            >>> # Ждать любого сигнала от оператора (например, кнопки "Стоп")
            >>> if robot.io.digital.wait_any_input(await_sec=60):
            ...     print("Получен внешний сигнал — приостанавливаем операцию")
            ... else:
            ...     print("Таймаут: за 60 секунд ничего не произошло")

            >>> # Неблокирующая проверка активности
            >>> if robot.io.digital.wait_any_input(await_sec=0):
            ...     # Затем можно опросить все входы через get_input()
            ...     print("Обнаружено изменение на одном из входов")

        Notes:
            - Метод **не указывает, какой именно вход изменился** — только факт
              изменения. Для идентификации конкретного входа используйте
              `get_input()` после возврата `True`.
            - При `await_sec = -1` вызов может блокировать программу неограниченно —
              используйте с осторожностью в автоматизированных сценариях.
        """
        current_inputs = self._rtd_receiver.get_data().dig_in
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to wait any digital input - connection was lost",
                )
                return False
            if current_inputs != self._rtd_receiver.get_data().dig_in:
                return True
        return False

    @handle_connection(available_in_read_only=True)
    def wait_input(
        self,
        index: DigitalIndex,
        value: bool,
        await_sec: int | float = -1,
    ) -> bool:
        """Блокирует выполнение до тех пор, пока на цифровом входе не появится ожидаемое состояние.

        Метод используется для синхронизации с внешними событиями:
        например, ожидание нажатия кнопки, срабатывания датчика или открытия двери.
        Проверка выполняется с высокой частотой на стороне контроллера, что
        исключает пропуск коротких импульсов.

        Метод доступен в режиме «read only».

        Args:
            index (DigitalIndex): Индекс цифрового входа.
                Допустимый диапазон: от `0` до `23`.
            value (bool): Ожидаемое состояние входа:
                - `True` — ожидается активный сигнал («высокий уровень»);
                - `False` — ожидается отсутствие сигнала («низкий уровень»).
            await_sec (int, optional): Максимальное время ожидания в секундах:
                - `-1` — ожидание без ограничения по времени (по умолчанию);
                - `0` — неблокирующая проверка: вернуть результат немедленно;
                - `> 0` — ожидать не более указанного числа секунд.

        Returns:
            True: В случае получения ожидаемого сигнала.
            False: В случае таймаута (если await_sec >= 0).

        Examples:
            >>> # Дождаться нажатия кнопки «Пуск» (вход 5)
            >>> if robot.io.digital.wait_input(5, value=True):
            ...     print("Кнопка нажата — запускаем операцию")
            ... else:
            ...     print("Таймаут: кнопка не нажата")

            >>> # Неблокирующая проверка наличия детали (вход 10)
            >>> if robot.io.digital.wait_input(10, value=True, await_sec=0):
            ...     robot.motion.move_to_home_pose()
            ... else:
            ...     print("Деталь отсутствует")

            >>> # Ждать открытия двери не более 30 секунд
            >>> door_open = robot.io.digital.wait_input(1, value=False, await_sec=30)
            >>> if not door_open:
            ...     raise RuntimeError("Дверь не открыта вовремя")

        Notes:
            - При `await_sec = -1` вызов может блокировать программу неограниченно —
              используйте с осторожностью в автоматизированных системах.
            - Для ожидания **любого изменения** на любом входе используйте
              `wait_any_input()`.
        """

        Validation.index(index, range(AVAILABLE_DIG_IN_INDEX_COUNT))
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    f"Failed to wait digital input {index} - connection was lost",
                )
                return False
            if self.get_input(index) == bool(value):
                return True
        return False

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set_input_function(
        self, index: DigitalIndex, function: InputFunctionName
    ) -> bool:
        """Назначает действие, которое будет автоматически выполнено при активации цифрового входа.

        Эта функция позволяет превратить любой цифровой вход (0–23) в управляющую
        кнопку: например, «Пуск», «Пауза», «Возврат домой» и т.д. Действие
        срабатывает при переходе входа в активное состояние.

        Args:
            index (DigitalIndex): Индекс цифрового входа. Диапазон: `0`–`23`.
            function (InputFunctionName): Назначаемое действие. Допустимые значения:
                - `'no_func'` — отключить реакцию на вход (по умолчанию);
                - `'move'` — начать выполнение траектории из буфера;
                - `'hold'` — немедленно остановить робота и очистить буфер точек;
                - `'pause'` — приостановить движение без очистки буфера;
                - `'zero_gravity'` — включить режим свободного перемещения (Free Drive);
                - `'run'` — включить робота (снять тормоза);
                - `'move_to_home'` — инициировать возврат в домашнюю позицию.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Назначить вход 0 как кнопку "Пауза"
            >>> robot.io.digital.set_input_function(0, 'pause')

            >>> # Назначить вход 1 как кнопку "Возврат домой"
            >>> robot.io.digital.set_input_function(1, 'move_to_home')

            >>> # Отключить реакцию на вход 5
            >>> robot.io.digital.set_input_function(5, 'no_func')

        Notes:
            - После перезагрузки контроллера назначения **могут сбрасываться** —
              убедитесь, что они восстанавливаются при старте приложения.
        """

        Validation.literal("in_f", function)
        Validation.index(index, range(AVAILABLE_DIG_IN_INDEX_COUNT))
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_DIG_INPUT_FUNC,
            index,
            InputFunction[function].value,
        )

    @handle_connection(available_in_read_only=False)
    def get_input_functions(
        self, index: DigitalIndex | None = None
    ) -> tuple[tuple[int, str], ...] | tuple[int, str]:
        """Возвращает назначенную функцию для одного или всех цифровых входов.

        Метод позволяет проверить, какое действие привязано к конкретному входу,
        или получить полную карту назначений для всех входов (0–23).

        Получить установленное действие на 'index' цифровом входе. При вызове
        без аргумента вернет все индексы цифровых входов и
        установленные на них действия.

        Args:
            index (DigitalIndex, optional): Индекс цифрового входа (0–23).
                Если не указан, возвращаются данные по всем входам.

        Returns:
            Tuple[Tuple[int, str], ...] | Tuple[int, str]:

                - Если `index` задан: кортеж `(индекс, функция)`, например:
                  `(0, 'pause')`.
                - Если `index=None`: кортеж кортежей вида `(индекс, функция)`
                  для всех входов, которым назначена **непустая функция**
                  (функция `'no_func'` обычно исключается из списка).

            Возможные значения функции:

                - 'no_func' — вход неактивен;
                - 'move' — запуск движения по траектории;
                - 'hold' — аварийная остановка с очисткой буфера;
                - 'pause' — приостановка без очистки буфера;
                - 'zero_gravity' — режим свободного перемещения;
                - 'run' — включение робота (снятие тормозов);
                - 'move_to_home' — возврат в домашнюю позицию.

        Examples:
            >>> # Получить функцию для входа 0
            >>> idx, func = robot.io.digital.get_input_functions(0)
            >>> print(f"Вход {idx}: {func}")

            >>> # Получить все назначенные функции
            >>> all_funcs = robot.io.digital.get_input_functions()
            >>> for idx, func in all_funcs:
            ...     print(f"Вход {idx}: {func}")
        """

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_DIG_INPUT_FUNC
        )
        if index is None:
            result = []
            for ind, value in enumerate(
                response[:AVAILABLE_DIG_IN_INDEX_COUNT]
            ):
                result.append((ind, InputFunction(value).name))
            return tuple(result)
        Validation.index(index, range(AVAILABLE_DIG_IN_INDEX_COUNT))
        return index, InputFunction(response[index]).name

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set_output_function(
        self, index: DigitalIndex, function: OutputFunctionName
    ) -> bool:
        """Назначает автоматическое поведение цифровому выходу в зависимости от состояния робота.

        Вместо ручного управления через `set_output()`, выход может
        автоматически отражать статус робота: движение, ошибка, предупреждение и т.д.
        Это позволяет подключать индикаторы, реле или систему верхнего уровня
        без необходимости постоянного опроса состояния.

        Args:
            index: Индекс цифрового выхода (0 — 23).
            function: Устанавливаемое действие на цифровом выходе.
                - 'no_func' — выход не управляется автоматически (по умолчанию);
                - 'no_move_signal_false' — выход = `0`, когда робот **остановлен**;
                - 'no_move_signal_true' — выход = `1`, когда робот **остановлен**;
                - 'move_status_signal_true_false' — выход = `1` при **движении**,
                `0` — при остановке;
                - 'run_signal_true' — выход = 1, когда контроллер в состоянии `'run'`
                (тормоза сняты, робот готов к движению);
                - 'warning_signal_true' — выход = 1 при активном **предупреждении**
                (например, `'protective_stop'`);
                - 'error_signal_true' — выход = 1 при **ошибке** (например,
                'fault', 'violation').

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Назначить выход 0 как индикатор движения
            >>> robot.io.digital.set_output_function(0, 'move_status_signal_true_false')

            >>> # Назначить выход 1 как аварийный сигнал
            >>> robot.io.digital.set_output_function(1, 'error_signal_true')

            >>> # Отключить автоматическое управление выходом 5
            >>> robot.io.digital.set_output_function(5, 'no_func')
        """

        Validation.index(index, range(AVAILABLE_DIG_OUT_INDEX_COUNT))
        Validation.literal("out_f", function)
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_DIG_OUTPUT_FUNC,
            index,
            OutputFunction[function].value,
        )

    @handle_connection(available_in_read_only=False)
    def get_output_functions(
        self, index: DigitalIndex | None = None
    ) -> tuple[tuple[int, str], ...] | tuple[int, str]:
        """Возвращает назначенную автоматическую функцию для одного или всех цифровых выходов.

        Метод позволяет проверить, какое поведение привязано к конкретному выходу,
        или получить полную карту автоматических назначений для всех выходов (0–23).

        Args:
            index (DigitalIndex, optional): Индекс цифрового выхода (0–23).
                Если не указан, возвращаются данные по всем выходам.

        Returns:
            Tuple[Tuple[int, str], ...] | Tuple[int, str]:
                - Если `index` задан: кортеж `(индекс, функция)`, например:
                  `(0, 'move_status_signal_true_false')`.
                - Если `index=None`: кортеж кортежей `(индекс, функция)` для всех
                  выходов с активными автоматическими функциями
                  (выходы с `'no_func'` обычно исключаются из списка).

            Возможные значения функции:
                'no_func' - отсутствие действия на цифровом выходе.
                'no_move_signal_false' - при остановленном роботе значение
                    устанавливается в 0.
                'no_move_signal_true' - при остановленном роботе значение
                    устанавливается в 1.
                'move_status_signal_true_false' - при остановленном роботе
                    значение устанавливается в 0, при движении - 1.
                'run_signal_true' - робот при состоянии RUN выдает 1.
                'warning_signal_true' - выдает при предупреждении 1 на цифровой
                    выход.
                'error_signal_true' - выдает при ошибке 1 на цифровой выход.

        Examples:
            >>> # Получить функцию для выхода 0
            >>> idx, func = robot.io.digital.get_output_functions(0)
            >>> print(f"Выход {idx}: {func}")

            >>> # Получить все активные автоматические назначения
            >>> all_funcs = robot.io.digital.get_output_functions()
            >>> for idx, func in all_funcs:
            ...     print(f"Выход {idx}: {func}")
        """

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_DIG_OUTPUT_FUNC
        )
        if index is None:
            result = []
            for ind, value in enumerate(
                response[:AVAILABLE_DIG_OUT_INDEX_COUNT]
            ):
                if value != NO_FUNC_ANSWER_VALUE:
                    result.append((ind, OutputFunction(value).name))
            return tuple(result)
        Validation.index(index, range(AVAILABLE_DIG_OUT_INDEX_COUNT))
        return index, OutputFunction(response[index]).name
