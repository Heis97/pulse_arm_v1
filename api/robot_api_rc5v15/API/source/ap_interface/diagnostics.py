from __future__ import annotations

from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import RTDReceiver


class Diagnostics:
    """
    Класс для работы с диагностической информацией робота.
    """

    def __init__(
        self,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
    ) -> None:
        self._rtd_receiver = rtd_receiver
        self._connection_state = connection_state

    # Controller
    @handle_connection(available_in_read_only=True)
    def get_controller_temperature(self) -> float:
        """Возвращает текущую температуру контроллера робота.

        Температура измеряется в градусах Цельсия. Метод доступен
        в режиме «read only».

        Returns:
            float: Температура контроллера в градусах Цельсия.

        Examples:
            >>> temp = robot.diagnostics.get_controller_temperature()
            >>> print(f"Температура контроллера: {temp:.1f} °C")
            ... # Температура контроллера: 42.3 °C

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().cbox_temp

    # Robot
    @handle_connection(available_in_read_only=True)
    def get_robot_voltage(self) -> float:
        """Возвращает текущее напряжение питания робота.

        Напряжение измеряется в вольтах. Метод доступен в режиме «read only».

        Returns:
            float: Напряжение питания робота в вольтах.

        Examples:
            >>> voltage = robot.diagnostics.get_robot_voltage()
            >>> print(f"Напряжение питания: {voltage:.2f} В")
            ... # Напряжение питания: 47.85 В

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().arm_voltage

    @handle_connection(available_in_read_only=True)
    def get_robot_current(self) -> float:
        """Возвращает текущий ток потребления робота.

        Ток измеряется в амперах. Метод доступен в режиме «read only».

        Returns:
            float: Ток потребления робота в амперах.

        Examples:
            >>> current = robot.diagnostics.get_robot_current()
            >>> print(f"Ток потребления: {current:.2f} А")
            ... # Ток потребления: 3.45 А

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().arm_current

    # IO
    @handle_connection(available_in_read_only=True)
    def get_io_current(self) -> float:
        """Возвращает ток потребления модулей ввода-вывода (I/O).

        Ток измеряется в амперах. Метод доступен в режиме «read only».

        Returns:
            float: Ток потребления модулей ввода-вывода в амперах.

        Examples:
            >>> io_current = robot.diagnostics.get_io_current()
            >>> print(f"Ток I/O: {io_current:.2f} А")
            ... # Ток I/O: 0.75 А

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().io_current

    # Tool
    @handle_connection(available_in_read_only=True)
    def get_tool_current(self) -> float:
        """Возвращает ток потребления инструмента.

        Ток измеряется в амперах. Метод доступен в режиме «read only».

        Returns:
            float: Ток потребления инструмента в амперах.

        Examples:
            >>> tool_current = robot.diagnostics.get_tool_current()
            >>> print(f"Ток инструмента: {tool_current:.2f} А")
            ... # Ток инструмента: 1.20 А

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().tool_power_current

    # Joints
    @handle_connection(available_in_read_only=True)
    def get_joints_motor_temperatures(self) -> tuple[float, ...]:
        """Возвращает температуры двигателей сочленений робота.

        Температура каждого двигателя измеряется в градусах Цельсия.
        Метод доступен в режиме «read only».

        Returns:
            Tuple[float, ...]: Кортеж с температурами двигателей сочленений
            в градусах Цельсия. Порядок значений соответствует порядку
            сочленений от основания к захвату.

        Examples:
            >>> temps = robot.diagnostics.get_joints_motor_temperatures()
            >>> print(f"Температуры двигателей: {[f'{t:.1f}' for t in temps]} °C")
            ... # Температуры двигателей: ['32.5', '33.1', '34.0', '32.8', '33.5', '32.9'] °C

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().joint_temp_motor

    @handle_connection(available_in_read_only=True)
    def get_joints_controller_temperatures(self) -> tuple[float, ...]:
        """Возвращает температуры контроллеров сочленений робота.

        Температура каждого контроллера измеряется в градусах Цельсия.
        Метод доступен в режиме «read only».

        Returns:
            Tuple[float, ...]: Кортеж с температурами контроллеров сочленений
            в градусах Цельсия. Порядок значений соответствует порядку
            сочленений от основания к захвату.

        Examples:
            >>> temps = robot.diagnostics.get_joints_controller_temperatures()
            >>> print(f"Температуры контроллеров: {[f'{t:.1f}' for t in temps]} °C")
            ... # Температуры контроллеров: ['50.2', '51.0', '50.8', '50.9', '52.5', '55.0'] °C

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().joint_temp_board

    @handle_connection(available_in_read_only=True)
    def get_joints_currents(self) -> tuple[float, ...]:
        """Возвращает токи двигателей сочленений робота.

        Ток каждого двигателя измеряется в амперах. Метод доступен в
        режиме «read only».

        Returns:
            Tuple[float, ...]: Кортеж с токами двигателей сочленений в амперах.
            Порядок значений соответствует порядку сочленений от основания к
            захвату.

        Examples:
            >>> currents = robot.diagnostics.get_joints_currents()
            >>> print(f"Токи двигателей: {[f'{i:.2f}' for i in currents]} А")
            ... # Токи двигателей: ['0.45', '0.62', '0.38', '0.51', '0.49', '0.40'] А

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().joint_amp

    @handle_connection(available_in_read_only=True)
    def get_joints_voltages(self) -> tuple[float, ...]:
        """Возвращает напряжения двигателей сочленений робота.

        Напряжение каждого двигателя измеряется в вольтах. Метод доступен в режиме «read only».

        Returns:
            Tuple[float, ...]: Кортеж с напряжениями двигателей сочленений в
            вольтах. Порядок значений соответствует порядку сочленений от
            основания к захвату.

        Examples:
            >>> voltages = robot.diagnostics.get_joints_voltages()
            >>> print(f"Напряжения двигателей: {[f'{v:.2f}' for v in voltages]} В")
            ... # Напряжения двигателей: ['47.80', '47.85', '47.78', '47.82', '47.84', '47.81'] В

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().joint_volt

    @handle_connection(available_in_read_only=True)
    def get_joints_torques(self) -> tuple[float, ...]:
        """Возвращает моменты двигателей сочленений робота.

        Момент каждого двигателя измеряется в Ньютон-метрах (Н·м).
        Метод доступен в режиме «read only».

        Returns:
            Tuple[float, ...]: Кортеж с моментами двигателей сочленений в Н·м.
            Порядок значений соответствует порядку сочленений от основания к
            захвату.

        Examples:
            >>> torques = robot.diagnostics.get_joints_torques()
            >>> print(f"Моменты двигателей: {[f'{t:.2f}' for t in torques]} Н·м")
            ... # Моменты двигателей: ['2.35', '1.80', '0.95', '0.40', '0.25', '0.10'] Н·м

        Notes:
            - Рекомендуется вызывать метод с частотой не более 100 Гц (не чаще
                0.01 с).
        """
        return self._rtd_receiver.get_data().act_trq
