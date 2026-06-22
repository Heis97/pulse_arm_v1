from __future__ import annotations

from logging import Logger
from struct import pack
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    FunctionTimeOutError,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.tools import sleep
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
    PauseModes,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    InComingMotionMode,
    MotionWarning,
    OutComingMotionMode,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    BUFFER_CHECK_TIMEOUT,
    CHECK_FREQUENCY_SEC,
    EMPTY_BYTES,
    OMM_ENABLE_DISABLE_PACK_FORMAT,
    SET_MOTION_MODE_AWAIT_SEC,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import MotionModeName

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class MotionMode(LoggerMixin):
    """
    Класс для работы с текущими режимами движения робота.
    В режиме 'hold' недоступен режим 'pause'.

    Доступные режимы движения:

        'hold' — Удержание позиции. (Сброс траектории).
        'pause' — Удержание позиции. (Сохранение траектории).
        'move' — Начать/продолжить выполнение заданной траектории.
        'move_adv' — Начать/продолжить выполнение заданной траектории с
            использованием режима движения 'Advanced'.
        'zero_gravity' — Режим FreeDrive (ручное управление).
            (Сброс траектории).
        'jog' — Декартовый джоггинг (джоггинг ЦТИ). (Сброс траектории).
        'joint_jog' — Моторный джоггинг (джоггинг каждым мотором в
            отдельности). (Сброс траектории).
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
    ) -> None:
        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._connection_state = connection_state
        self._set_logger(logger)

    @handle_connection(available_in_read_only=True)
    def get(self) -> str:
        """Возвращает текущий режим движения робота.

        Режим определяет, как контроллер обрабатывает команды движения и буфер траекторий.
        Метод доступен в режиме «read only».

        Возможные значения:

            - 'hold': Удержание текущей позиции с **сбросом всей траектории**.
              Режим используется для безопасной остановки.
            - 'pause': Удержание позиции **с сохранением траектории**.
              Позволяет возобновить выполнение с текущей точки.
              Недоступен, если текущий режим — 'hold'.
            - 'move': Выполнение траектории, заданной через классы 'Joint' и 'Linear'.
            - 'move_adv': Выполнение траектории, заданной через класс 'Advanced'.
            - 'realtime': Включено управления в реальном времени.
            - 'freedrive': Режим свободного перемещения (ручное управление с компенсацией гравитации).
              Сбрасывает буфер траекторий.
            - 'jog': Ручное управление TCP в декартовой системе координат.
              Сбрасывает буфер траекторий.
            - 'joint_jog': Ручное управление отдельными сочленениями.
              Сбрасывает буфер траекторий.
            Получить текущий режим движения робота.

        Returns:
            str: Текущий режим движения. Значение всегда одно из перечисленных выше.

        Examples:
            >>> mode = robot.motion.mode.get()
            >>> if mode == 'pause':
            ...     robot.motion.mode.set('move')  # возобновить
            ... elif mode in ('hold', 'freedrive'):
            ...     print("Траектория была сброшена")

        Notes:
            - Перевод в любой режим, кроме 'pause', **автоматически удаляет**
              все точки из буфера ядра управления.
            - Режим 'pause' может быть установлен **только из режимов,
              поддерживающих возобновление** (например, 'move' или 'move_adv').
        """

        return InComingMotionMode(
            self._rtd_receiver.get_data().motion_mode
        ).name

    @handle_connection(available_in_read_only=True)
    def check_warning_status(self) -> str:
        """Проверяет наличие предупреждений, связанных с текущим состоянием движения робота.

        Метод особенно полезен при диагностике причины, по которой робот перешёл
        в режим `pause` — например, из-за коллизии или превышения усилия.
        Возвращает категорию предупреждения, если таковая активна.

        Метод доступен в режиме «read only».

        Returns:
            str: Текущий статус предупреждения. Возможные значения:

                - `'protective_stop'`: Движение остановлено из-за внешних условий,
                  например, столкновения с объектом в рабочей зоне или превышения
                  допустимого усилия.
                - `'self_collision'`: Движение остановлено, так как запрошенная
                  траектория привела бы к столкновению частей робота между собой.
                - `'no_warning'`: Активных предупреждений нет; режим `pause` может
                  быть вызван пользователем или другим управляющим сигналом.

        Examples:
            >>> mode = robot.motion.mode.get()
            >>> if mode == 'pause':
            ...     warning = robot.motion.mode.check_warning_status()
            ...     if warning == 'protective_stop':
            ...         print("Проверьте, нет ли препятствий в зоне робота")
            ...     elif warning == 'self_collision':
            ...         print("Невозможная траектория: риск самостолкновения")
        """
        return MotionWarning(
            int(self._rtd_receiver.get_data().state_flags)
        ).name

    @handle_connection(available_in_read_only=False)
    def set(
        self,
        mode: MotionModeName,
        await_sec: int | float = SET_MOTION_MODE_AWAIT_SEC,
    ) -> bool:
        """Устанавливает новый режим движения робота.

        Смена режима влияет на обработку траекторий и состояние контроллера:
        - Переход в любой режим, **кроме 'pause'**, приводит к **полному сбросу**
          буфера целевых точек.
        - Режим 'pause' сохраняет текущую траекторию и позволяет возобновить
          выполнение позже.

        Args:
            mode (MotionModeName): Целевой режим движения. Допустимые значения:
                - `'move'` — выполнение траекторий из `LinearMotion` и `JointMotion`;
                - `'move_adv'` — выполнение траекторий из `AdvancedMotion`;
                - `'pause'` — приостановка с сохранением траектории;
                - `'hold'` — остановка с полным сбросом траектории;
                - `'realtime'` — включение управления в реальном времени.
            await_sec (int, optional): Время ожидания подтверждения перехода:
                - `-1` — ожидание без ограничения по времени (по умолчанию);
                - `0` — неблокирующий вызов: отправить команду и немедленно вернуться;
                - `> 0` — ожидать не более указанного числа секунд.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Приостановить движение с возможностью возобновления
            >>> robot.motion.mode.set('pause')

            >>> # Возобновить выполнение (в зависимости от типа траектории)
            >>> robot.motion.mode.set('move')      # для Linear/Joint
            >>> robot.motion.mode.set('move_adv')  # для Advanced

            >>> # Остановка с очисткой буфера
            >>> robot.motion.mode.set('hold')

        Notes:
            - Перевод в любой режим, кроме 'pause', **автоматически удаляет**
              все точки из буфера ядра управления.
            - Режим 'pause' может быть установлен **только из режимов,
              поддерживающих возобновление** (например, 'move' или 'move_adv').
        """

        Validation.literal("mm", mode)
        if mode == InComingMotionMode.realtime.name:
            return self._enable_realtime_mode(await_sec)

        points_in_buffer = 0
        for _ in sleep(
            await_sec=BUFFER_CHECK_TIMEOUT,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            points_in_buffer = self._rtd_receiver.get_data().buff_fill
            if points_in_buffer >= 1:
                break

        self._write_log(
            "debug",
            f"Waypoint in queue before motion mode '{mode}' "
            f"setting: {points_in_buffer}",
        )

        if (
            mode == InComingMotionMode.move.name
            or mode == InComingMotionMode.move_adv.name
        ):
            if points_in_buffer <= 0:
                self._write_log(
                    "warning",
                    f"Waypoint queue is empty. Can not process "
                    f"{mode} motion mode",
                )
                return False

        if self.get() == InComingMotionMode.pause.name:
            self._controller.send_command(
                ControllerCommands.CTRLR_COMS_PAUSE, PauseModes.disable
            )
        return self._enable_mode(mode, await_sec)

    @handle_connection(available_in_read_only=False)
    def _enable_mode(
        self,
        mode: MotionModeName,
        await_sec: int | float = SET_MOTION_MODE_AWAIT_SEC,
    ) -> bool:
        payload = (
            pack(OMM_ENABLE_DISABLE_PACK_FORMAT, PauseModes.enable)
            if mode == InComingMotionMode.pause.name
            else EMPTY_BYTES
        )
        self._controller.send(OutComingMotionMode[mode].value, payload)
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error", "Failed to set motion mode - connection was lost"
                )
                return False
            actual_mode = self.get()
            if actual_mode == mode:
                return True
            if (
                self._rtd_receiver.get_data().buff_fill <= 0
                and actual_mode == InComingMotionMode.hold.name
            ):
                self._write_log("warning", "All waypoints have been executed.")
                return True

        raise FunctionTimeOutError("Motion mode", await_sec)

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def _enable_realtime_mode(
        self,
        await_sec: int | float = SET_MOTION_MODE_AWAIT_SEC,
    ) -> bool:
        self._controller.send_command(
            ControllerCommands.CTRLR_COMS_ENABLE_REALTIME
        )
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to enable realtime mode - connection was lost",
                )
                return False
            if self.get() == InComingMotionMode.realtime.name:
                return True

        raise FunctionTimeOutError("Realtime motion mode", await_sec)
