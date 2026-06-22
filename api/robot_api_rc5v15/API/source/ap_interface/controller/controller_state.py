from __future__ import annotations

import time
from collections.abc import Callable
from logging import Logger

from api.robot_api_rc5v15.API.source.ap_interface.motion import Motion
from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    FunctionTimeOutError,
    RobotCalibrationPositionError,
    SavedPositionError,
)
from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.tools import sleep
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.service_types import (
    JointAngleDiscrepancy,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    InComingControllerState,
    OutComingControllerState,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    CHECK_FREQUENCY_SEC,
    JNT_CONFIRM_POS_THRESHOLD,
    SET_CTRLR_STATE_AWAIT_SEC,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import ControllerStateName


class ControllerState(LoggerMixin):
    """
    Класс для работы с текущими состояниями контроллера.
    Доступные состояния контроллера:

        - 'idle' — Начальное состояние контроллера.
        - 'off' — Контроллер выключен.
        - 'stby' — Промежуточное состояние.
        - 'on' — Контроллер включен. Тормоза активированы.
            Если робот в состоянии 'run', ставит робота на тормоза.
            Если робот в состоянии 'off', включает робота, не снимая его с
            тормозов.
        - 'run' — Контроллер включен. Тормоза деактивированы.
        - 'calibration' — Вычисление смещения (для сохранения позиции робота
            после перезагрузки).
        - 'failure' — Фатальная ошибка контроллера.
        - 'force_exit' — Принудительное завершение работы ядра.

    При использовании 'on' и 'off', все имеющиеся целевые точки в памяти робота
        удаляются.
    """

    _CALLBACK_PRE_CALL_PAUSE: float = 0.2  # sec

    _controller: Controller
    _rtd_receiver: RTDReceiver
    _motion: Motion

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        motion: Motion,
        connection_state: ConnectionState,
        logger: Logger | None,
    ) -> None:
        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._motion = motion
        self._connection_state = connection_state
        self._set_logger(logger)
        self._position_confirmed = False
        self._confirm_position_callback: (
            Callable[[tuple[JointAngleDiscrepancy, ...]], bool] | None
        ) = None
        self._confirm_position_callback_was_called: bool = False

    @handle_connection(available_in_read_only=True)
    def get(self) -> str:
        """Возвращает текущее состояние контроллера робота.

        Состояние отражает текущий режим работы ядра управления и определяет,
        какие операции доступны в данный момент. Метод доступен в режиме
        «read only».

        Возможные значения:

            - 'idle': Начальное состояние контроллера (после запуска, до включения).
            - 'off': Контроллер выключен, питание на манипулятор не подано.
            - 'stby': Промежуточное состояние — подано питание на манипулятор,
              но тормоза активированы.
            - 'on': Контроллер включён, тормоза активированы. Готов к запуску движения.
            - 'run': Контроллер включён, тормоза деактивированы. Выполняются движения.
            - 'calibration': Процесс калибровки или вычисления смещения для сохранения
              позиции после перезагрузки.
            - 'failure': Фатальная ошибка контроллера, требующая вмешательства.
            - 'force_exit': Принудительное завершение работы ядра управления.

        Returns:
            str: Текущее состояние контроллера. Значение всегда одно из перечисленных выше.

        Examples:
            >>> state = robot.controller.state.get()
            >>> if state == 'run':
            ...     print("Робот в активном состоянии")
            ... elif state == 'off':
            ...     print("Робот выключен")
        """

        return InComingControllerState(
            int(self._rtd_receiver.get_data().state)
        ).name

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set(
        self,
        state: ControllerStateName,
        await_sec: int | float = SET_CTRLR_STATE_AWAIT_SEC,
    ) -> bool:
        """Устанавливает новое состояние контроллера робота.

        Метод отправляет команду на перевод контроллера в указанное состояние
        и, при необходимости, ожидает подтверждения выполнения.

        Args:
            state (ControllerStateName): Целевое состояние контроллера.
                Допустимые значения: 'on', 'off', 'run'.
                - 'on': включить робота, активировать тормоза;
                - 'off': выключить робота, деактивировать питание;
                - 'run': запустить робота (деактивировать тормоза).
            await_sec (int, optional): Максимальное время ожидания подтверждения
                перехода в целевое состояние (в секундах).
                - `-1` — ожидание без ограничения по времени;
                - `0` — отправить команду и немедленно вернуть управление
                  (без ожидания завершения);
                - `> 0` — ожидать не более указанного числа секунд.

        Returns:
            bool:  True: В случае успешной отправки команды.
                   False: В случае таймаута (если await_sec >= 0).

        Examples:
            >>> # Включить робота и дождаться готовности
            >>> if robot.controller.state.set('on'):
            ...     print("Робот включён")

        Notes:
            - При переходе в состояния 'on' или 'off' **все загруженные целевые точки
              удаляются из памяти робота**.
            - Состояния 'idle', 'stby', 'calibration', 'failure', 'force_exit'
              **не могут быть установлены вручную** — они управляются ядром.
        """
        init_state = self.get()
        Validation.literal("cs", state)
        if state not in (
            InComingControllerState.run.name,
            InComingControllerState.calibration.name,
        ):
            self._position_confirmed = False

        res = self._controller.send_command(
            ControllerCommands.CTRLR_COMS_POWER,
            OutComingControllerState[state].value,
        )
        if self._wait_for_state(state, await_sec, CHECK_FREQUENCY_SEC):
            return res
        elif self._confirm_position_callback_was_called:
            self._confirm_position_callback_was_called = False
            if self._wait_for_state(state, await_sec, CHECK_FREQUENCY_SEC):
                return res
        raise FunctionTimeOutError(
            f"Controller state ({init_state}->{state})", await_sec
        )

    def _wait_for_state(
        self,
        state: ControllerStateName,
        await_sec: int | float,
        frequency: float = CHECK_FREQUENCY_SEC,
    ) -> bool:
        for _ in sleep(
            await_sec=await_sec,
            frequency=frequency,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to wait controller state - connection was lost",
                )
                return False

            current_state = self.get()

            if current_state == state:
                return True

            if state in (
                InComingControllerState.run.name,
                InComingControllerState.calibration.name,
            ):
                if not self._position_confirmed:
                    self._write_log("debug", "Confirming robot position")
                    if not self._confirm_position():
                        self._compare_position()
                    else:
                        self._write_log(
                            "debug", "Robot position was confirmed"
                        )

        return False

    def set_confirm_position_callback(
        self,
        callback: Callable[[tuple[JointAngleDiscrepancy, ...]], bool]
        | None = None,
    ) -> bool:
        """Устанавливает обработчик для подтверждения расхождения позиции робота.

        Вызывается, когда контроллер обнаруживает несоответствие между
        последним сохранённым положением сочленений и их текущим фактическим
        положением (например, после сбоя питания). Подтверждение необходимо
        для возобновления работы.

        Метод **доступен в режиме read-only**, но само подтверждение
        может быть выполнено **только при подключении в управляющем режиме**.

        Если `callback=None` (по умолчанию), используется стандартный
        терминальный обработчик, запрашивающий подтверждение через `input()`.

        Args:
            callback (Callable[[Tuple[JointAngleDiscrepancy, ...]], bool] or None, optional):
                Функция, принимающая кортеж объектов `JointAngleDiscrepancy`
                и возвращающая `True`, если позиция подтверждена, в противном
                случае `False`. Если `None` — восстанавливается стандартный
                обработчик с запросом подтверждения через консоль.

        Returns:
            True: В случае успешной установки обработчика.

        Examples:
            >>> # Использовать стандартный терминальный обработчик
            >>> robot.controller.state.set_confirm_position_callback()

            >>> # Автоматическое подтверждение при малых расхождениях
            >>> def auto_confirm(data):
            ...     return all(abs(j.actual_position - j.saved_position) <= 5.0
            ...                for j in data)
            >>> robot.controller.state.set_confirm_position_callback(auto_confirm)

        Notes:
            - Обработчик вызывается **синхронно и блокирующе** — управление
              не возвращается, пока функция не вернёт результат.
            - API **не накладывает таймаут** на выполнение обработчика.
              Бесконечное ожидание в callback приведёт к зависанию всей программы.
            - Объект `JointAngleDiscrepancy` содержит:
                * `joint_number`: номер сочленения (от основания),
                * `allowed_discrepancy`: допустимое расхождение (в градусах),
                * `actual_position`: текущее положение привода (°),
                * `saved_position`: сохранённая позиция (°).
            - Рекомендуется использовать автоматическое подтверждение
              только в контролируемых условиях (например, при частых
              кратковременных отключениях питания) и с разумным порогом
              допуска (≤ 5–10°).
        """
        self._confirm_position_callback = (
            callback or self._confirm_position_default_callback
        )
        self._write_log(
            "warning", "Confirm position callback was set, it may not be save"
        )
        return True

    def _confirm_position(self) -> bool:
        try:
            confirm = self._controller.receive_command(
                ControllerCommands.CTRLR_COMS_CONFIRM_POSITION,
            )
            if isinstance(confirm, tuple):
                if len(confirm) < 1 or confirm[0] == 0:
                    self._position_confirmed = True
                    return True
                return False
            self._position_confirmed = True
            return True
        except Exception as e:
            self._write_log("error", f"Failed to confirm position, error: {e}")
            return False

    def _compare_position(self) -> bool:
        self._write_log("debug", "Comparing robot position")
        incorrect = []
        actual_position = self._motion.get_actual_position(
            orientation_units="deg", position_format="joints"
        )
        saved_position = self._motion.get_last_saved_position(
            orientation_units="deg", position_format="joints"
        )

        if saved_position is None:
            raise SavedPositionError("Failed to get robot last saved position")

        for index, values in enumerate(zip(saved_position, actual_position)):
            if abs(values[0] - values[1]) > JNT_CONFIRM_POS_THRESHOLD:
                incorrect.append(
                    JointAngleDiscrepancy(
                        joint_number=index,
                        allowed_discrepancy=JNT_CONFIRM_POS_THRESHOLD,
                        actual_position=values[1],
                        saved_position=values[0],
                    )
                )
        if len(incorrect) > 0:
            if self._confirm_position_callback is not None:
                self._confirm_position_callback_was_called = True
                time.sleep(self._CALLBACK_PRE_CALL_PAUSE)
                self._write_log(
                    "info", "Calling confirm position callback function"
                )
                if self._confirm_position_callback(tuple(incorrect)):
                    self._write_log("info", "Position was confirmed")
                    return self._controller.send_command(
                        ControllerCommands.CTRLR_COMS_CONFIRM_POSITION
                    )
                self._write_log("warning", "Position has not been confirmed")
            raise RobotCalibrationPositionError(tuple(incorrect))
        self._write_log("debug", "Robot position was validated")
        return True

    def _confirm_position_default_callback(
        self, data: tuple[JointAngleDiscrepancy, ...]
    ) -> bool:
        """
        Стандартный обработчик для события необходимости подтверждения положения.
        """
        self._write_log("warning", "Manual position confirm required.")
        print()
        print(
            "Are you sure that the actual position of the\nrobot corresponds to the specified one?"
        )
        print()

        header = (
            f"{'Joint':>5} | {'Actual position':>15} | {'Saved position':>14}"
        )
        separator = "-" * len(header)
        print(header)
        print(separator)

        for joint in data:
            row = f"{joint.joint_number:>5} | {joint.actual_position:>14.2f}° | {joint.saved_position:>13.2f}°"
            print(row)

        print()
        answer = input("Enter [yes/no] (default [no]): ").strip().lower()
        print()
        if answer in ("yes", "y"):
            return True
        return False
