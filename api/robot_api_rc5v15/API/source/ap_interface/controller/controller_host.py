from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
)

from .controller_gravity import ControllerGravity
from .controller_state import ControllerState

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.ap_interface.motion import Motion
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class RobotController:
    """
    Класс для работы с входами/выходами контроллера робота.
    """

    state: ControllerState
    """Подкласс для работы с состоянием контроллера."""
    gravity: ControllerGravity
    """Подкласс для работы с параметрами ориентации робота."""

    def __init__(
        self,
        motion: Motion,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: Logger | None,
    ) -> None:
        self._connection_state = connection_state
        self.state = ControllerState(
            controller=controller,
            rtd_receiver=rtd_receiver,
            motion=motion,
            connection_state=connection_state,
            logger=logger,
        )
        self.gravity = ControllerGravity(
            controller=controller,
            connection_state=connection_state,
            logger=logger,
        )
