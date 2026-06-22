from __future__ import annotations

from collections.abc import Iterable
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
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.io_functions import (
    InputFunction,
    OutputFunction,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    WristMode,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    AVAILABLE_DIG_OUT_INDEX_COUNT,
    BYTE_SIZE,
    CHECK_FREQUENCY_SEC,
    NO_FUNC_ANSWER_VALUE,
    REAL_WRIST_MAX_AN_IN,
    REAL_WRIST_MAX_DIG_OUT,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    DigitalIndex,
    DigitalWristIndex,
    InputFunctionName,
    OutputFunctionName,
    WristInputActivationType,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class WristDigitalIO(LoggerMixin):
    """
    Класс для работы с цифровыми входами/выходами платы запястья робота.
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

    def _get_input(self, index: int, inputs_range: Iterable) -> bool:
        """
        Получить текущее двоичное значение на 'index' цифровом входе.

        Args:
            index: Индекс входа.
        Returns:
            bool: True — есть сигнал, False — нет сигнала.
        """
        Validation.index(index, inputs_range)
        mask = 1 << (index % BYTE_SIZE)
        return int(self._rtd_receiver.get_data().wrist_dig_in) & mask != 0

    @handle_connection(available_in_read_only=True)
    def get_input(self, index: DigitalWristIndex) -> bool:
        """Получает текущее состояние цифрового входа на плате запястья робота.

        Метод позволяет считать бинарный сигнал (вкл/выкл) с одного из цифровых
        входов платы запястья — например, для определения состояния концевика,
        кнопки или датчика присутствия.
        Метод доступен в режиме «read only».

        Args:
            index (DigitalWristIndex): Индекс цифрового входа. Допустимые значения: `0`–`1`.

        Returns:
            bool:
                - `True` — на входе присутствует активный сигнал (логическая «1»);
                - `False` — сигнал отсутствует (логический «0»).

        Examples:
            >>> # Считать состояние цифрового входа 0
            >>> if robot.wrist.digital.get_input(0):
            ...     print("Концевик сработал")

            >>> # Проверить оба цифровых входа
            >>> input_0 = robot.wrist.digital.get_input(0)
            >>> input_1 = robot.wrist.digital.get_input(1)
            >>> print(f"Входы: [0]={input_0}, [1]={input_1}")

        Notes:
            - Метод выполняется мгновенно и не блокирует выполнение программы.
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not get input value, when wrist mode is {WristMode.off}"
            )
        return self._get_input(
            index=index,
            inputs_range=range(REAL_WRIST_MAX_AN_IN),
        )

    @handle_connection(available_in_read_only=True)
    def get_output(self, index: DigitalWristIndex) -> bool:
        """Получает текущее состояние цифрового выхода на плате запястья робота.

        Метод позволяет узнать, включён или выключен заданный цифровой выход —
        например, чтобы проверить, активировано ли внешнее реле, светодиод или
        управляющий сигнал для исполнительного устройства.
        Метод доступен в режиме «read only».

        Args:
            index (DigitalWristIndex): Индекс цифрового выхода. Допустимые значения: `0`–`1`.

        Returns:
            bool:
                - `True` — на выходе установлен активный сигнал (логическая «1»);
                - `False` — сигнал отключён (логический «0»).

        Examples:
            >>> # Проверить состояние цифрового выхода 0
            >>> if robot.wrist.digital.get_output(0):
            ...     print("Реле на выходе 0 включено")

            >>> # Считать оба выхода для логгирования
            >>> out0 = robot.wrist.digital.get_output(0)
            >>> out1 = robot.wrist.digital.get_output(1)
            >>> print(f"Выходы: [0]={out0}, [1]={out1}")

        Notes:
            - Для изменения состояния выхода используйте метод `set_output`.

        Получить текущее двоичное значение на 'index' цифровом выходе платы
        запястья.

        Args:
            index: Индекс выхода (0-1).
        Returns:
            bool: True — есть сигнал, False — нет сигнала.
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not get output value, when wrist mode is {WristMode.off}"
            )
        Validation.index(
            index,
            range(REAL_WRIST_MAX_DIG_OUT),
        )
        mask = 1 << (index % BYTE_SIZE)
        return int(self._rtd_receiver.get_data().wrist_dig_out) & mask != 0

    @handle_connection(available_in_read_only=False)
    def set_output(self, index: DigitalWristIndex, value: bool) -> bool:
        """Устанавливает логическое состояние цифрового выхода на плате запястья робота.

        Метод позволяет управлять внешними устройствами через цифровые выходы:
        например, включать реле, светодиоды, соленоиды или подавать управляющий
        сигнал на другую электронику.

        Args:
            index (DigitalWristIndex): Индекс цифрового выхода. Допустимые значения: `0`–`1`.
            value (bool): Новое состояние выхода:
                - `True` — установить активный сигнал (логическая «1»);
                - `False` — отключить сигнал (логический «0»).

        Returns:
            bool: `True`, если команда успешно отправлена на плату запястья.

        Examples:
            >>> # Включить реле на цифровом выходе 0
            >>> robot.wrist.digital.set_output(0, True)

            >>> # Выключить светодиод на выходе 1
            >>> robot.wrist.digital.set_output(1, False)

            >>> # Переключить состояние выхода на противоположное
            >>> current = robot.wrist.digital.get_output(0)
            >>> robot.wrist.digital.set_output(0, not current)

        Notes:
            - Возвращаемое значение подтверждает лишь отправку команды, а не факт
              физического переключения на клемме.
            - Убедитесь, что подключённая нагрузка совместима с электрическими
              характеристиками выходов платы запястья (напряжение, ток, тип сигнала).
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not set output value, when wrist mode is {WristMode.off}"
            )
        Validation.index(
            index,
            range(REAL_WRIST_MAX_DIG_OUT),
        )
        set_output_template = SetWristInputOutputTemplate()
        dig_out_mask = set_output_template.dig_out_mask
        dig_out_mask = 1 << (index % BYTE_SIZE)
        set_output_template.dig_out_mask = dig_out_mask
        dig_out = set_output_template.dig_out
        dig_out = (1 if value else 0) << (index % BYTE_SIZE)
        set_output_template.dig_out = dig_out
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_WRIST_IO, set_output_template
        )

    @handle_connection(available_in_read_only=False)
    def pulse_output(
        self,
        index: DigitalWristIndex,
        duration: float,
        active_high: bool = True,
    ) -> bool:
        """Генерирует одиночный импульс на цифровом выходе запястья.

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
            index (DigitalWristIndex): Индекс выхода (0–N).
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
            >>> # Положительный импульс 100 мс на выходе 1
            >>> success = robot.wrist.digital.pulse_output(1, duration=0.1)

            >>> # Отрицательный импульс 50 мс (активный низкий) на выходе 0
            >>> success = robot.wrist.digital.pulse_output(
            ...     0, duration=0.05, active_high=False
            ... )
        """
        if duration < 0:
            raise ValueError(f"Duration must be non-negative, got {duration}")

        Validation.index(index, range(AVAILABLE_DIG_OUT_INDEX_COUNT))

        if not self.set_output(index, active_high):
            return False

        for _ in sleep(duration):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to pulse wrist digital output - connection was lost",
                )
                return False

        return self.set_output(index, not active_high)

    @handle_connection(available_in_read_only=True)
    def wait_any_input(self, await_sec: int | float = -1) -> bool:
        """Ожидает изменения состояния **любого** из цифровых входов платы запястья.

        Метод полезен, когда важно отреагировать на событие от любого подключённого
        цифрового датчика или кнопки, не привязываясь к конкретному входу — например,
        для реализации универсального триггера или обнаружения неожиданного сигнала.
        Метод доступен в режиме «read only».

        Args:
            await_sec (int): Максимальное время ожидания в секундах:
                - `-1` — ожидание без ограничения (по умолчанию);
                - `0` — однократная проверка: вернёт `True`, если **хотя бы один** вход
                  изменил состояние с момента последнего опроса (или имеет активный сигнал);
                - положительное число — лимит времени ожидания в секундах.

        Returns:
            bool:
                - `True`, если произошло изменение состояния на **одном или нескольких**
                  цифровых входах (0 или 1) в течение указанного времени;
                - `False`, если ни один вход не изменил состояние до истечения тайм-аута
                  (только при `await_sec >= 0`).

        Examples:
            >>> # Дождаться любого сигнала от подключённых датчиков (макс. 10 сек)
            >>> if robot.wrist.digital.wait_any_input(await_sec=10):
            ...     print("Сработал хотя бы один датчик!")
            ... else:
            ...     print("Нет активности на цифровых входах")

            >>> # Ждать неограниченно, пока что-то не произойдёт
            >>> robot.wrist.digital.wait_any_input()  # блокирует выполнение

        Notes:
            - Метод не указывает, **какой именно** вход изменил состояние — для этого
              требуется дополнительный опрос через `get_input`.
            - При `await_sec = -1` выполнение программы может быть заблокировано надолго.
            - Метод работает только с цифровыми входами платы запястья (входы 0 и 1).
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not wait input value, when wrist mode is {WristMode.off}"
            )
        current_inputs = self._rtd_receiver.get_data().dig_in
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to wait any wrist digit input - connection was lost",
                )
                return False
            if current_inputs != self._rtd_receiver.get_data().dig_in:
                return True
        return False

    @handle_connection(available_in_read_only=True)
    def wait_input(
        self,
        index: DigitalWristIndex,
        value: bool,
        await_sec: int | float = -1,
    ) -> bool:
        """Ожидает появления заданного логического уровня на цифровом входе платы запястья.

        Метод блокирует выполнение до тех пор, пока указанный цифровой вход не примет
        требуемое состояние — например, дождаться нажатия кнопки, срабатывания
        датчика или отключения сигнала присутствия.
        Метод доступен в режиме «read only».

        Args:
            index (DigitalWristIndex): Индекс цифрового входа. Допустимые значения: `0`–`1`.
            value (bool): Ожидаемое состояние входа:
                - `True` — дождаться появления активного сигнала;
                - `False` — дождаться исчезновения сигнала.
            await_sec (int): Максимальное время ожидания в секундах:
                - `-1` — ожидание без ограничения (по умолчанию);
                - `0` — однократная проверка без блокировки;
                - положительное число — лимит времени ожидания в секундах.

        Returns:
            bool:
                - `True`, если вход принял ожидаемое состояние в течение заданного времени;
                - `False`, если произошёл тайм-аут (только при `await_sec >= 0`).

        Examples:
            >>> # Дождаться, пока на входе 0 появится сигнал (макс. 5 сек)
            >>> if robot.wrist.digital.wait_input(0, True, await_sec=5):
            ...     print("Датчик сработал!")
            ... else:
            ...     print("Тайм-аут: датчик не активировался")

            >>> # Проверить однократно, отключён ли вход 1
            >>> is_off = robot.wrist.digital.wait_input(1, False, await_sec=0)

            >>> # Бесконечно ждать, пока пользователь не нажмёт кнопку (вход 0 = True)
            >>> robot.wrist.digital.wait_input(0, True)

        Notes:
            - При `await_sec = -1` метод может блокировать выполнение программы на неопределённое время.
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not wait input value, when wrist mode is {WristMode.off}"
            )
        Validation.index(index, range(REAL_WRIST_MAX_AN_IN))
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    f"Failed to wait wrist digital input {index} - connection was lost",
                )
                return False
            if self.get_input(index) == bool(value):
                return True
        return False

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set_input_function(
        self,
        index: DigitalWristIndex,
        function: InputFunctionName,
    ) -> bool:
        """Назначает функцию, выполняемую при активации цифрового входа платы запястья.

        Метод позволяет связать физический цифровой вход (например, кнопку или
        датчик) с определённым действием робота — например, запуском программы,
        переходом в режим свободного перемещения или остановкой движения.

        Args:
            index (DigitalWristIndex): Индекс цифрового входа. Допустимые значения: `0`–`1`.
            function (InputFunctionName): Функция, привязываемая к входу. Возможные значения:
                - `'no_func'` — вход не вызывает никакого действия;
                - `'move'` — запуск движения по запланированным точкам;
                - `'hold'` — немедленная остановка робота и очистка буфера траектории;
                - `'pause'` — остановка без очистки буфера (возобновление возможно);
                - `'zero_gravity'` — включение режима FREE DRIVE (свободное перемещение);
                - `'run'` — включение робота (переход в рабочее состояние);
                - `'move_to_home'` — возврат робота в домашнюю позицию.

        Returns:
            bool: `True`, если команда успешно отправлена и функция назначена.

        Examples:
            >>> # Назначить кнопке на входе 0 запуск движения
            >>> robot.wrist.digital.set_input_function(0, 'move')

            >>> # Отключить все действия на входе 1
            >>> robot.wrist.digital.set_input_function(1, 'no_func')

            >>> # Связать вход 0 с режимом FREE DRIVE
            >>> robot.wrist.digital.set_input_function(0, 'zero_gravity')

        Notes:
            - Назначение функции не отменяет возможность читать текущее состояние входа
              через `get_input`.
            - Изменения сохраняются до следующего переназначения или перезагрузки платы.
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not set input function, when wrist mode is {WristMode.off}"
            )
        Validation.literal("in_f", function)
        Validation.index(index, range(REAL_WRIST_MAX_AN_IN))
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_WRIST_DIG_INPUT_FUNC,
            index,
            InputFunction[function].value,
        )

    @handle_connection(available_in_read_only=False)
    def get_input_functions(
        self, index: DigitalWristIndex | None = None
    ) -> tuple[tuple[int, str], ...] | tuple[int, str]:
        """Получает функцию, назначенную на один или все цифровые входы платы запястья.

        Метод позволяет проверить, какие действия привязаны к цифровым входам:
        как для конкретного входа, так и для всех сразу. Это полезно при отладке,
        восстановлении конфигурации или проверке текущего поведения устройства.

        Args:
            index (Optional[DigitalWristIndex]): Индекс цифрового входа. Допустимые значения: `0`–`1`.
                Если не указан (`None`), возвращаются функции для **всех** цифровых входов.

        Returns:
            Union[Tuple[int, str], Tuple[Tuple[int, str], ...]]:
                - Если указан `index`: кортеж вида `(индекс, функция)`.
                - Если `index=None`: кортеж из кортежей вида `((0, функция), (1, функция))`.

            Возможные значения функции:
                - `'no_func'` — вход не вызывает никакого действия;
                - `'move'` — запуск движения по точкам;
                - `'hold'` — остановка и очистка буфера траектории;
                - `'pause'` — остановка без очистки буфера;
                - `'zero_gravity'` — включение режима FREE DRIVE;
                - `'run'` — включение робота;
                - `'move_to_home'` — возврат в домашнюю позицию.

        Examples:
            >>> # Получить функцию, назначенную на вход 0
            >>> idx, func = robot.wrist.digital.get_input_functions(0)
            >>> print(f"Вход {idx} вызывает: {func}")

            >>> # Получить функции всех цифровых входов
            >>> all_funcs = robot.wrist.digital.get_input_functions()
            >>> for idx, func in all_funcs:
            ...     print(f"Вход {idx}: {func}")

            >>> # Проверить, включён ли FREE DRIVE на любом входе
            >>> if any(func == 'zero_gravity' for _, func in robot.wrist.digital.get_input_functions()):
            ...     print("Режим FREE DRIVE доступен через цифровой вход")
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not get output function, when wrist mode is {WristMode.off}"
            )
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_DIG_INPUT_FUNC
        )
        if index is None:
            result = []
            for ind, value in enumerate(response[:REAL_WRIST_MAX_AN_IN]):
                result.append((ind, InputFunction(value).name))
            return tuple(result)
        Validation.index(index, range(REAL_WRIST_MAX_AN_IN))
        return index, InputFunction(response[index]).name

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set_active_output(
        self,
        wrist_index: DigitalWristIndex,
        output_index: DigitalIndex,
        activation_type: WristInputActivationType,
    ) -> bool:
        """Настраивает управление цифровым выходом контроллера через цифровой вход платы запястья.

        Метод позволяет связать физический вход на плате запястья (например, кнопку)
        с любым из цифровых выходов основного контроллера (0–23), чтобы управлять
        внешними устройствами (реле, световыми индикаторами, пневмоклапанами и т.п.)
        напрямую через аппаратную логику — без участия основной программы.

        Args:
            wrist_index (DigitalWristIndex): Индекс цифрового входа на плате запястья.
                Допустимые значения: `0`–`1`.
            output_index (DigitalIndex): Индекс цифрового выхода **контроллера**,
                который будет управляться. Допустимые значения: `0`–`23`.
            activation_type (WristInputActivationType): Тип реакции на изменение входа:
                - `'hold'` — выход активен **только пока удерживается** сигнал на входе
                  платы запястья;
                - `'trigger'` — выход переключается **однократно** при изменении
                  состояния входа (фронта сигнала).

        Returns:
            bool: `True`, если команда успешно отправлена и связь настроена.

        Examples:
            >>> # При удержании кнопки на входе 0 платы запястья включать выход 5 контроллера
            >>> robot.wrist.digital.set_active_output(0, 5, 'hold')

            >>> # По нажатию кнопки на входе 1 платы запястья однократно сработать на выход 12
            >>> robot.wrist.digital.set_active_output(1, 12, 'trigger')

        Notes:
            - Один вход платы запястья может управлять **только одним** выходом контроллера
              за раз (повторный вызов перезаписывает предыдущую привязку).
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not set hold active function, when wrist mode is {WristMode.off}"
            )
        Validation.index(
            wrist_index,
            range(REAL_WRIST_MAX_AN_IN),
        )
        Validation.index(
            output_index,
            range(AVAILABLE_DIG_OUT_INDEX_COUNT),
        )
        Validation.literal("activation_type", activation_type)
        if activation_type == "hold":
            function = InputFunction.ifunc_hold_io_dig_output
        elif activation_type == "trigger":
            function = InputFunction.ifunc_trigger_io_dig_output
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_WRIST_DIG_INPUT_FUNC,
            wrist_index,
            function + output_index,
        )

    @handle_connection(available_in_read_only=False)
    def set_output_function(
        self,
        index: DigitalWristIndex,
        function: OutputFunctionName,
    ) -> bool:
        """Назначает автоматическое поведение цифрового выхода платы запястья в зависимости от состояния робота.

        Метод позволяет связать цифровой выход с внутренними событиями системы —
        например, автоматически включать индикатор при ошибке, сигнализировать
        о движении или управлять внешним оборудованием в зависимости от режима работы.

        Args:
            index (DigitalWristIndex): Индекс цифрового выхода. Допустимые значения: `0`–`1`.
            function (OutputFunctionName): Функция, определяющая логику управления выходом. Возможные значения:
                - `'no_func'` — выход не управляется автоматически (можно использовать вручную);
                - `'no_move_signal_false'` — при остановке робота выход устанавливается в `0`;
                - `'no_move_signal_true'` — при остановке робота выход устанавливается в `1`;
                - `'move_status_signal_true_false'` — `0` при остановке, `1` при движении;
                - `'run_signal_true'` — выход устанавливается в `1`, когда робот находится в состоянии `RUN`;
                - `'warning_signal_true'` — выход устанавливается в `1` при возникновении предупреждения;
                - `'error_signal_true'` — выход устанавливается в `1` при возникновении ошибки.

        Returns:
            bool: `True`, если команда успешно отправлена и функция назначена.

        Examples:
            >>> # На выходе 0 индицировать ошибки: 1 — ошибка, 0 — всё в порядке
            >>> robot.wrist.digital.set_output_function(0, 'error_signal_true')

            >>> # На выходе 1 отображать статус движения: 1 — движется, 0 — стоит
            >>> robot.wrist.digital.set_output_function(1, 'move_status_signal_true_false')

            >>> # Отключить автоматическое управление выходом 0
            >>> robot.wrist.digital.set_output_function(0, 'no_func')
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not set output function, when wrist mode is {WristMode.off}"
            )
        Validation.index(
            index,
            range(REAL_WRIST_MAX_DIG_OUT),
        )
        Validation.literal("out_f", function)
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_WRIST_DIG_OUTPUT_FUNC,
            index,
            OutputFunction[function].value,
        )

    @handle_connection(available_in_read_only=False)
    def get_output_functions(
        self, index: DigitalWristIndex | None = None
    ) -> tuple[tuple[int, str], ...] | tuple[int, str]:
        """Получает функцию, назначенную на один или все цифровые выходы платы запястья.

        Метод позволяет узнать, какое автоматическое поведение привязано к цифровым
        выходам.

        Args:
            index (Optional[DigitalWristIndex]): Индекс цифрового выхода. Допустимые значения: `0`–`1`.
                Если не указан (`None`), возвращаются функции для **всех** цифровых выходов.

        Returns:
            Union[Tuple[int, str], Tuple[Tuple[int, str], ...]]:
                - Если указан `index`: кортеж вида `(индекс, функция)`.
                - Если `index=None`: кортеж из кортежей вида `((0, функция), (1, функция))`.

            Возможные значения функции:
                - `'no_func'` — выход не управляется автоматически;
                - `'no_move_signal_false'` — при остановке робота выход = `0`;
                - `'no_move_signal_true'` — при остановке робота выход = `1`;
                - `'move_status_signal_true_false'` — `0` при остановке, `1` при движении;
                - `'run_signal_true'` — выход = `1`, когда робот в состоянии `RUN`;
                - `'warning_signal_true'` — выход = `1` при предупреждении;
                - `'error_signal_true'` — выход = `1` при ошибке.

        Examples:
            >>> # Получить функцию, назначенную на выход 0
            >>> idx, func = robot.wrist.digital.get_output_functions(0)
            >>> print(f"Выход {idx} настроен как: {func}")

            >>> # Получить конфигурацию всех цифровых выходов
            >>> all_funcs = robot.wrist.digital.get_output_functions()
            >>> for idx, func in all_funcs:
            ...     print(f"Выход {idx}: {func}")

            >>> # Проверить, есть ли выход, сигнализирующий об ошибках
            >>> if any(func == 'error_signal_true' for _, func in robot.wrist.digital.get_output_functions()):
            ...     print("Обнаружен выход, связанный с ошибками")
        """
        if self._rtd_receiver.get_data().wrist_mode == WristMode.off:
            raise WristStateError(
                f"Can not get output function, when wrist mode is {WristMode.off}"
            )
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_DIG_OUTPUT_FUNC
        )
        if index is None:
            result = []
            for ind, value in enumerate(response[:REAL_WRIST_MAX_DIG_OUT]):
                if value != NO_FUNC_ANSWER_VALUE:
                    result.append((ind, OutputFunction(value).name))
            return tuple(result)
        Validation.index(
            index,
            range(REAL_WRIST_MAX_DIG_OUT),
        )
        return index, OutputFunction(response[index]).name
