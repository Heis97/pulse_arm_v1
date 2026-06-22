from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
)

from .safety_limits import SafetyLimits
from .safety_status import SafetyStatus

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller, RTDReceiver


class Safety:
    """
    Класс для работы с входами/выходами контроллера робота.
    """

    status: SafetyStatus
    """Подкласс для работы со статусами безопасности."""
    limits: SafetyLimits
    """Подкласс для работы с пределами безопасности."""

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: Logger | None,
    ) -> None:
        self._connection_state = connection_state
        self.status = SafetyStatus(
            rtd_receiver=rtd_receiver,
            connection_state=connection_state,
            logger=logger,
        )
        self.limits = SafetyLimits(
            rtd_receiver=rtd_receiver,
            controller=controller,
            connection_state=connection_state,
            logger=logger,
        )
