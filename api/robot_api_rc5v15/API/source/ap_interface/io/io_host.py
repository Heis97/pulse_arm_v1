from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
)

from .analog_io import AnalogIO
from .digital_io import DigitalIO

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class IO:
    """
    Класс для работы с входами/выходами контроллера робота.
    """

    digital: DigitalIO
    """Подкласс для работы с цифровыми входами/выходами."""
    analog: AnalogIO
    """Подкласс для работы с аналоговыми входами/выходами."""

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: Logger | None,
    ) -> None:
        self._connection_state = connection_state
        self.digital = DigitalIO(
            controller=controller,
            rtd_receiver=rtd_receiver,
            connection_state=connection_state,
            logger=logger,
        )
        self.analog = AnalogIO(
            controller=controller,
            rtd_receiver=rtd_receiver,
            connection_state=connection_state,
            logger=logger,
        )
