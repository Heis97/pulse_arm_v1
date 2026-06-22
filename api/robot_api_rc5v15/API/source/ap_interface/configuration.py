from __future__ import annotations

from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.network import (
    Controller,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    radians_to_degrees,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    JOINTS_COUNT,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller


class Configuration(LoggerMixin):
    """
    Класс для работы с конфигурацией робота.
    """

    _controller: Controller

    def __init__(
        self,
        controller: Controller,
        connection_state: ConnectionState,
    ) -> None:
        self._controller = controller
        self._connection_state = connection_state

    @handle_connection(available_in_read_only=False)
    def get_joints_position_limits(
        self,
        units: AngleUnits | None = None,
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Возвращает программные ограничения углов поворота для всех шести сочленений робота.

        Метод запрашивает у контроллера допустимые минимальные и максимальные углы для
        каждого сустава. Эти значения определяются конструктивными ограничениями кинематики
        и текущими настройками программных лимитов. Результат отражает
        **технически допустимый диапазон** перемещения по каждой оси.

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Args:
            units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.

        Returns:
            Tuple[Tuple[float, ...], Tuple[float, ...]]: Кортеж из двух элементов
            `(min_limits, max_limits)`, где:
                - `min_limits` — кортеж из 6 минимально допустимых углов
                  `(J1_min, J2_min, J3_min, J4_min, J5_min, J6_min)`;
                - `max_limits` — кортеж из 6 максимально допустимых углов
                  `(J1_max, J2_max, J3_max, J4_max, J5_max, J6_max)`.
                Все значения указаны в порядке от основания (J1) к фланцу (J6).

        Examples:
            >>> # Получить ограничения в градусах (по умолчанию)
            >>> min_limits, max_limits = robot.configuration.get_joints_position_limits()
            >>> print(f"Мин. углы: {min_limits}")
            >>> print(f"Макс. углы: {max_limits}")

            >>> # Получить ограничения в радианах и проверить лимит для J3
            >>> limits_rad = robot.configuration.get_joints_position_limits(units='rad')
            >>> j3_min, j3_max = limits_rad[0][2], limits_rad[1][2]
        """

        units = units or MOTION_SETUP.units

        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_JOINT_POS_LIMITS
        )
        if units == "deg":
            limits = radians_to_degrees(limits)
        return (
            tuple(limits[:JOINTS_COUNT]),
            tuple(limits[JOINTS_COUNT : 2 * JOINTS_COUNT]),
        )

    @handle_connection(available_in_read_only=False)
    def get_joints_velocity_limits(
        self,
        units: AngleUnits | None = None,
    ) -> tuple[float, ...]:
        """Возвращает максимально допустимые угловые скорости для всех сочленений робота.

        Метод запрашивает у контроллера ограничения по скорости для каждой оси.
        Эти значения определяются динамическими характеристиками приводов,
        текущими настройками безопасности и аппаратными лимитами. Результат
        отражает **максимально допустимую скорость** перемещения по каждой оси,
        что критично для безопасного планирования траекторий.

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Args:
            units (AngleUnits, optional): Единицы измерения угловой скорости:
                - `'deg'` — градусы в секунду (по умолчанию);
                - `'rad'` — радианы в секунду.

        Returns:
            Tuple[float, ...]: Кортеж из 6 чисел:
                `(V1_max, V2_max, ..., V6_max)`, где `Vi_max` — максимальная
                допустимая угловая скорость i-го сочленения от основания к
                фланцу, в указанных единицах.

        Examples:
            >>> # Получить лимиты скоростей в градусах в секунду
            >>> vel_limits = robot.configuration.get_joints_velocity_limits()
            >>> print(f"Макс. скорости: {vel_limits}")

            >>> # Получить лимиты в радианах в секунду и проверить ось J5
            >>> limits_rad = robot.configuration.get_joints_velocity_limits(units='rad')
            >>> j5_max_vel = limits_rad[4]
        """

        units = units or MOTION_SETUP.units

        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_JOINT_VEL_LIMITS
        )
        if units == "deg":
            limits = radians_to_degrees(limits)
        return tuple(limits[:JOINTS_COUNT])

    @handle_connection(available_in_read_only=False)
    def get_joints_current_limits(self) -> tuple[float, ...]:
        """Возвращает максимально допустимые токи для всех сочленений робота.

        Метод запрашивает у контроллера текущие ограничения по току для каждого
        привода. Эти значения определяют максимальный крутящий момент, который
        могут развивать моторы, и используются системой безопасности для
        предотвращения перегрузок, перегрева обмоток и превышения допустимых
        механических нагрузок. Результат отражает **аппаратные и программные
        лимиты тока** на момент вызова.

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Returns:
            Tuple[float, ...]: Кортеж из 6 чисел:
                `(I1_max, I2_max, I3_max, I4_max, I5_max, I6_max)`, где `Ii_max` —
                максимальный допустимый ток i-го сочленения от основания к фланцу,
                в амперах (А).

        Examples:
            >>> # Получить лимиты токов для всех осей
            >>> current_limits = robot.configuration.get_joints_current_limits()
            >>> print(f"Макс. токи: {current_limits}")

            >>> # Проверить лимит тока для базового сочленения (J1)
            >>> j1_max_current = robot.configuration.get_joints_current_limits()[0]
        """
        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_JOINT_CURRENT_LIM
        )
        return tuple(limits[:JOINTS_COUNT])

    @handle_connection(available_in_read_only=False)
    def get_joints_nominal_torque(self) -> tuple[float, ...]:
        """Возвращает номинальные крутящие моменты для всех сочленений робота.

        Номинальный момент — это максимальный крутящий момент, который привод может
        развивать непрерывно в течение длительного времени без перегрева обмоток и
        превышения допустимых тепловых нагрузок. Эти значения используются для безопасного
        долгосрочного планирования траекторий и оценки постоянной рабочей нагрузки.

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Returns:
            Tuple[float, ...]: Кортеж из 6 чисел:
                `(T1_nom, T2_nom, T3_nom, T4_nom, T5_nom, T6_nom)`, где `Ti_nom` —
                номинальный момент i-го сочленения от основания к фланцу,
                в ньютон-метрах (Н·м).

        Examples:
            >>> # Получить номинальные моменты для оценки постоянной нагрузки
            >>> nominal_torques = robot.configuration.get_joints_nominal_torque()
            >>> print(f"Номинальные моменты: {nominal_torques}")

            >>> # Проверить, не превысит ли движение лимит J3
            >>> if required_torque > nominal_torques[2]:
            ...     print("Движение требует пикового режима")
        """
        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_JOINT_NOMINAL_TRQ
        )
        return tuple(limits[:JOINTS_COUNT])

    @handle_connection(available_in_read_only=False)
    def get_joints_torque_limits(self) -> tuple[float, ...]:
        """Возвращает максимальные (пиковые) крутящие моменты для всех сочленений робота.

        Максимальный момент — это предельное значение крутящего момента, которое привод
        способен развивать кратковременно (до срабатывания токовой или тепловой защиты).
        Этот параметр критичен для динамичных движений, резких ускорений/торможений
        и преодоления пиковых внешних нагрузок.

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Returns:
            Tuple[float, ...]: Кортеж из 6 чисел:
                `(T1_max, T2_max, T3_max, T4_max, T5_max, T6_max)`, где `Ti_max` —
                максимальный кратковременно допустимый момент i-го сочленения от
                основания к фланцу, в ньютон-метрах (Н·м).

        Examples:
            >>> # Получить пиковые лимиты для расчёта ускорений
            >>> peak_torques = robot.configuration.get_joints_torque_limits()
            >>> print(f"Пиковые моменты: {peak_torques}")

            >>> # Сравнить номинальный и пиковый момент для оси J5
            >>> nom = robot.configuration.get_joints_nominal_torque()[4]
            >>> peak = robot.configuration.get_joints_torque_limits()[4]
            >>> print(f"Запас пиковой мощности J5: {(peak / nom - 1) * 100:.1f}%")
        """
        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_JOINT_MAX_TRQ
        )
        return tuple(limits[:JOINTS_COUNT])

    @handle_connection(available_in_read_only=False)
    def get_max_payload(self) -> float:
        """Получает максимально допустимую массу полезной нагрузки робота.

        Метод возвращает максимально допустимую массу полезной нагрузки робота
        согласно его паспортным данным.

        Returns:
            float: масса полезной нагрузки в килограммах (float).

        Examples:
            >>> max_payload = robot.payload.get_max()
            ... print(
            ...     f"Максимально допустимая масса полезной нагрузки: {max_payload} кг"
            ... )

        Notes:
            - Значение возвращается в **килограммах**.
        """
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_MAX_PAYLOAD
        )
        return response[0]

    @handle_connection(available_in_read_only=False)
    def get_analog_out_limits(self) -> tuple[float, float]:
        """Возвращает аппаратные ограничения аналоговых выходов контроллера.

        Метод запрашивает у контроллера допустимые пределы напряжения и тока
        для аналоговых выходных каналов. Эти значения определяются
        характеристиками ЦАП и выходных цепей контроллера. Результат отражает
        **безопасный рабочий диапазон** для подключения внешних исполнительных
        устройств (пропорциональных клапанов, сервоприводов, датчиков).

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Returns:
            Tuple[float, float]: Кортеж из двух значений:
                `(voltage_limit, current_limit)`, где:
                - `voltage_limit` — максимальное допустимое напряжение, в вольтах (В);
                - `current_limit` — максимальный допустимый ток нагрузки, в миллиамперах (мА).

        Examples:
            >>> # Получить лимиты аналогового выхода
            >>> v_lim, i_lim = robot.configuration.get_analog_out_limits()
            >>> print(f"Макс. напряжение: {v_lim} В, макс. ток: {i_lim} мА")

            >>> # Проверить совместимость с внешним клапаном
            >>> if valve_nominal_current > i_lim:
            ...     raise RuntimeError("Требуется внешний усилитель тока")
        """
        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_AN_OUT_LIMITS
        )
        return tuple(limits[:2])

    @handle_connection(available_in_read_only=False)
    def get_analog_in_limits(self) -> tuple[float, float]:
        """Возвращает аппаратные ограничения аналоговых входов контроллера.

        Метод запрашивает у контроллера допустимые пределы напряжения и тока
        для аналоговых входных каналов. Эти значения определяются входным
        импедансом АЦП и встроенными цепями защиты. Результат отражает
        **технически допустимый диапазон** входных сигналов без риска
        повреждения входных цепей или искажения оцифровки.

        Метод требует полного подключения к контроллеру (недоступен в режиме read-only).

        Returns:
            Tuple[float, float]: Кортеж из двух значений:
                `(voltage_limit, current_limit)`, где:
                - `voltage_limit` — максимальное допустимое входное напряжение, в вольтах (В);
                - `current_limit` — максимальный допустимый входной ток, в миллиамперах (мА).

        Examples:
            >>> # Получить лимиты аналогового входа
            >>> v_lim, i_lim = robot.configuration.get_analog_in_limits()
            >>> print(f"Допустимый вход: до {v_lim} В, до {i_lim} мА")

            >>> # Валидация сигнала датчика перед подключением
            >>> if sensor_max_voltage > v_lim:
            ...     print("Внимание: требуется делитель напряжения или преобразователь")
        """
        limits = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_AN_IN_LIMITS
        )
        return tuple(limits[:2])
