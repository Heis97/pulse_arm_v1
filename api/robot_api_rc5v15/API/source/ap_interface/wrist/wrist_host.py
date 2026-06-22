from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.ap_interface.wrist.analog_io import (
    WristAnalogIO,
)
from api.robot_api_rc5v15.API.source.ap_interface.wrist.digital_io import (
    WristDigitalIO,
)
from api.robot_api_rc5v15.API.source.ap_interface.wrist.rs_485 import WristRS485
from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    FunctionTimeOutError,
    WristStateError,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.tools import (
    sleep,
)
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
from api.robot_api_rc5v15.API.source.models.constants import (
    CHECK_FREQUENCY_SEC,
    SET_WRIST_MODE_AWAIT_SEC,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import WristModeName

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class Wrist(LoggerMixin):
    """
    Класс для работы с платой запястья робота.
    Доступные состояния платы запястья:
        'off' — Плата запястья отсутствует.
        'rs485' — Плата запястья найдена и работает в режиме rs485 (обмен
            данными по протоколу modbus).
        'analog_in' — Плата запястья найдена и находится в режиме работы
            с аналоговыми входами.
        'nc' — Плата запястья найдена но не сконфигурирована для работы с
            внешними устройствами.
        'gnd' — Плата запястья найдена и находится в режиме общей 'земли'
    """

    _digital: WristDigitalIO
    """Подкласс для работы с цифровыми входами/выходами."""
    _analog: WristAnalogIO
    """Подкласс для работы с аналоговыми входами/выходами."""
    _rs_485: WristRS485
    """Подкласс для работы с RS-485."""
    _rtd_receiver: RTDReceiver
    _controller: Controller

    def __init__(
        self,
        host: str,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: Logger | None,
    ) -> None:
        self._host = host
        self._rtd_receiver = rtd_receiver
        self._controller = controller
        self._connection_state = connection_state
        self._set_logger(logger)
        self._digital = WristDigitalIO(
            controller=controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
            logger=logger,
        )
        self._analog = WristAnalogIO(
            controller=controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
            logger=logger,
        )
        self._rs_485 = WristRS485(host=host, logger=logger)

    @property
    def digital(self) -> WristDigitalIO:
        """
        Цифровые выходы доступны всегда и не зависят от режима работы
        платы запястья.
        """
        return self._digital

    @property
    def analog(self) -> WristAnalogIO:
        """
        Аналоговые входы доступны только в режиме работы платы запястья
        'analog_in'.
        """
        current_mode = self.get_mode()
        if current_mode != "analog_in":
            raise WristStateError(
                f"Analog inputs only available in 'analog_in' wrist mode. "
                f"Current wrist mode '{current_mode}'"
            )
        return self._analog

    @property
    def rs_485(self) -> WristRS485:
        """
        Интерфейс RS-485 для работы с совместимыми устройствами, доступен в
        режиме работы платы запястья 'rs485'.
        """
        current_mode = self.get_mode()
        if current_mode != "rs485":
            raise WristStateError(
                f"RS-485 interface is only available in 'rs485' wrist mode. "
                f"Current wrist mode '{current_mode}'"
            )
        return self._rs_485

    @handle_connection(available_in_read_only=True)
    def get_mode(self) -> WristModeName:
        """Получает текущее состояние платы запястья робота.

        Метод возвращает режим, в котором в данный момент работает плата запястья.
        Метод доступен в режиме «read only»

        Returns:
            str: Текущее состояние платы запястья. Возможные значения:
                - 'off' — плата запястья отсутствует;
                - 'rs485' — плата подключена и работает в режиме Modbus RTU по RS-485;
                - 'analog_in' — плата подключена и настроена на работу с аналоговыми входами;
                - 'nc' — плата подключена, но не сконфигурирована для работы с внешними устройствами;
                - 'gnd' — плата подключена и находится в режиме общей «земли»

        Examples:
            >>> state = robot.wrist.get_mode()
            >>> if state == 'analog_in':
            ...     print("Плата запястья готова к чтению аналоговых сигналов")
            >>> elif state == 'off':
            ...     print("Плата запястья не выключена")
        """
        return WristMode(int(self._rtd_receiver.get_data().wrist_mode)).name

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set_mode(
        self,
        mode: WristModeName,
        await_sec: int | float = SET_WRIST_MODE_AWAIT_SEC,
    ) -> bool:
        """Устанавливает рабочий режим платы запястья робота.

        Метод позволяет переключить плату запястья в требуемое состояние:
        отключить её, перевести в режим аналоговых входов, настроить для обмена
        по RS-485 или оставить в нейтральном состоянии.

        Args:
            mode (WristModeName): Целевой режим платы. Допустимые значения:
                - `'off'` — отключить плату запястья;
                - `'rs485'` — включить режим RS-485 (и совместимости с
                    Modbus RTU на основе RS-485);
                - `'analog_in'` — настроить плату для работы с аналоговыми входами;
                - `'nc'` — оставить плату подключённой, но неактивной;
                - `'gnd'` — перевести плату в режим общей «земли»
            await_sec (int): Максимальное время ожидания подтверждения смены режима (в секундах):
                - `-1` — ожидание без ограничения;
                - `0` — проверка выполняется один раз без блокировки;
                - положительное значение — лимит времени ожидания в секундах
                  (по умолчанию используется 60).

        Returns:
            bool:
                - `True`, если режим успешно установлен и подтверждён платой;
                - `False`, если смена режима не подтверждена в течение указанного времени.

        Examples:
            >>> # Перевести плату запястья в режим аналоговых входов с таймаутом 5 сек
            >>> robot.wrist.set_mode('analog_in', await_sec=5)

            >>> # Отключить плату запястья (используется значение по умолчанию для await_sec)
            >>> robot.wrist.set_mode('off')

            >>> # Попытаться установить режим RS-485
            >>> success = robot.wrist.set_mode('rs485', await_sec=10)
            >>> if not success:
            ...     print("Не удалось перевести плату в режим RS-485")
        """
        Validation.literal("Wm", mode)
        set_input_output_template = SetWristInputOutputTemplate()
        set_input_output_template.mux_mode = WristMode[mode].value
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
                    "error", "Failed to set wrist mode - connection was lost"
                )
                return False
            if self.get_mode() == WristMode[mode].name:
                return res
        raise FunctionTimeOutError("Wrist mode", await_sec)
