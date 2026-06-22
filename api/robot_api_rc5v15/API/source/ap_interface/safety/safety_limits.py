from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.network import (
    Controller,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.data_classes.service_types import (
    JointsLimitsParams,
    SafetyLimitsParams,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    InComingControllerState,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    JOINTS_COUNT,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class SafetyLimits(LoggerMixin):
    """
    Класс для работы с пределами безопасности робота.
    """

    _controller: Controller

    def __init__(
        self,
        rtd_receiver: RTDReceiver,
        controller: Controller,
        connection_state: ConnectionState,
        logger: logging.Logger | None = None,
    ) -> None:
        self._rtd_receiver = rtd_receiver
        self._controller = controller
        self._connection_state = connection_state
        self._set_logger(logger)

    @handle_connection(available_in_read_only=False)
    def get(self) -> tuple[SafetyLimitsParams, SafetyLimitsParams]:
        """Возвращает параметры безопасности для двух режимов работы робота.

        Метод запрашивает у контроллера полный набор параметров, определяющих
        безопасное поведение робота. Результат разделён на два независимых
        набора параметров:

        - **Normal mode** - стандартный режим работы с полными скоростями,
            точностью и динамикой. Используется при штатном выполнении задач.
        - **Reduced mode** - ограниченной режим. Активируется при срабатывании
            внешних защитных контуров.

        Метод требует полного подключения к контроллеру (недоступен в режиме
        read-only).

        Returns:
            Tuple[SafetyLimitsParams, SafetyLimitsParams]: Кортеж из двух объектов
            `(normal, reduced)`, где каждый содержит:

                - `max_tcp_velocity` (float): Максимальная линейная скорость
                  центра инструмента (TCP), м/с.
                - `max_joints_velocity` (Tuple[float, ...]): Максимальные угловые
                  скорости сочленений, рад/с. Порядок: от основания (J1) к фланцу (J6).
                - `joints_limits` (JointsLimitsParams): Ограничения по положению:
                    * `low_limits` - минимальные углы, рад;
                    * `high_limits` - максимальные углы, рад.
                - `joint_following_error` (float): Допустимая ошибка слежения
                  по положению сочленения, рад. Превышение может указывать на
                  столкновение или потерю шагов.
                - `cartesian_translation_following_error` (float): Допустимая
                  ошибка позиционирования центра инструмента в декартовом
                  пространстве, мм.
                - `cartesian_rotation_following_error` (float): Допустимая
                  ошибка ориентации центра инструмента в декартовом пространстве,
                  рад.
                - `max_stop_time` (float): Максимальное время полной остановки
                  после получения команды аварийного останова, с.

        Examples:
            >>> # Получить лимиты для обоих режимов
            >>> normal, reduced = robot.safety.limits.get()

            >>> # Проверить, что скорость в reduced mode не превышает 0.25 м/с
            >>> assert reduced.max_tcp_velocity <= 0.25

            >>> # Получить диапазон допустимых углов для J4 в нормальном режиме
            >>> j4_min, j4_max = normal.joints_limits.low_limits[3], normal.joints_limits.high_limits[3]
        """

        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_SFTY_LIMITS
        )
        mid = len(limits) // 2
        normal = limits[:mid]
        reduced = limits[mid:]
        normal_limits = self._process_limits(normal)
        reduced_limits = self._process_limits(reduced)
        return (normal_limits, reduced_limits)

    @handle_connection(available_in_read_only=False)
    def set(
        self,
        normal_limits: SafetyLimitsParams | None = None,
        reduced_limits: SafetyLimitsParams | None = None,
    ) -> bool:
        """Устанавливает новые параметры безопасности для нормального и/или
        ограниченного режимов.

        **Внимание: метод изменяет критически важные параметры безопасности.**
        Некорректные значения могут привести к травмам, повреждению оборудования
        или нарушению работы защитных контуров. Используйте только при полном
        понимании физических ограничений робота и после согласования с инженером
        по безопасности.

        **Требования к вызову:**
        - Контроллер должен находиться в состоянии **OFF** . Вызов в любом
            другом состоянии будет отклонён.
        - Изменения вступают в силу немедленно после успешного применения и
            сохраняются в энергонезависимой памяти контроллера.

        Если передан только один из параметров (`normal_limits` или `reduced_limits`),
        второй автоматически сохраняется без изменений.

        Args:
            normal_limits (SafetyLimitsParams, optional): Новые лимиты для
                нормального режима работы. Если `None` — текущие значения
                сохраняются без изменений.
            reduced_limits (SafetyLimitsParams, optional): Новые лимиты для
                режима пониженной безопасности. Если `None` — текущие значения
                сохраняются без изменений.

        Returns:
            bool: `True`, если параметры успешно применены контроллером;
                `False`, если произошла ошибка передачи, валидации или
                контроллер отклонил новые значения.

        Raises:
            ValueError: Если переданные значения выходят за аппаратно допустимые
                пределы (проверка выполняется на стороне контроллера).

        Examples:
            >>> # Установить новые лимиты только для reduced mode
            >>> new_reduced = SafetyLimitsParams(
            ...     max_tcp_velocity=0.1,  # м/с
            ...     max_joints_velocity=(0.5,) * 6,  # рад/с
            ...     joints_limits=JointsLimitsParams(
            ...         low_limits=(-2.0,) * 6,
            ...         high_limits=(2.0,) * 6,
            ...     ),
            ...     joint_following_error=0.01,
            ...     cartesian_translation_following_error=1.0,
            ...     cartesian_rotation_following_error=0.05,
            ...     max_stop_time=0.5,
            ... )
            >>> success = robot.safety.limits.set(reduced_limits=new_reduced)
            >>> if not success:
            ...     print("Не удалось применить лимиты")

            >>> # Обновить только скорость TCP в нормальном режиме,
            >>> # остальные параметры сохранить
            >>> current_normal, _ = robot.safety.limits.get()
            >>> current_normal.max_tcp_velocity = 1.5
            >>> robot.safety.limits.set(normal_limits=current_normal)

            >>> # Полная замена обоих режимов
            >>> robot.safety.limits.set(
            ...     normal_limits=new_normal,
            ...     reduced_limits=new_reduced
            ... )
        """

        if (
            InComingControllerState(int(self._rtd_receiver.get_data().state))
            != InComingControllerState.off
        ):
            self._write_log(
                "warning",
                "It is forbidden to change the safety limits when "
                "controller state is not 'off'",
            )
            return False

        self._write_log(
            "warning",
            "It is not safe to change the safety limits. "
            "You have to be aware of what you are doing.",
        )
        old_normal_limits, old_reduced_limits = None, None
        if normal_limits is None or reduced_limits is None:
            old_normal_limits, old_reduced_limits = self.get()

        normal_limits = normal_limits or old_normal_limits
        reduced_limits = reduced_limits or old_reduced_limits

        success = self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_SFTY_LIMITS,
            normal_limits,
            reduced_limits,
        )

        if success:
            self._write_log(
                "info",
                f"Normal safety limits was set to {normal_limits}\n"
                f"Reduced safety limits was set to {reduced_limits}",
            )
        else:
            self._write_log("warning", "Failed to apply new safety limits")

        return success

    @staticmethod
    def _process_limits(limits: tuple[float, ...]) -> SafetyLimitsParams:
        tcp_vel_max = limits[0]
        joint_vel_max = limits[1 : JOINTS_COUNT + 1]
        joint_position_low_limits = limits[
            JOINTS_COUNT + 1 : 2 * JOINTS_COUNT + 1
        ]
        joint_position_high_limits = limits[
            2 * JOINTS_COUNT + 1 : 3 * JOINTS_COUNT + 1
        ]
        joint_fol_err = limits[3 * JOINTS_COUNT + 1]
        cart_fol_err_t = limits[3 * JOINTS_COUNT + 2]
        cart_fol_err_r = limits[3 * JOINTS_COUNT + 3]
        stop_time = limits[3 * JOINTS_COUNT + 4]
        joint_params = JointsLimitsParams(
            low_limits=tuple(joint_position_low_limits),
            high_limits=tuple(joint_position_high_limits),
        )
        limits_params = SafetyLimitsParams(
            max_tcp_velocity=tcp_vel_max,
            max_joints_velocity=tuple(joint_vel_max),
            joints_limits=joint_params,
            joint_following_error=joint_fol_err,
            cartesian_translation_following_error=cart_fol_err_t,
            cartesian_rotation_following_error=cart_fol_err_r,
            max_stop_time=stop_time,
        )
        return limits_params
