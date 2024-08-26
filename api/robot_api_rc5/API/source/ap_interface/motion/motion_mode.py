from __future__ import annotations
from struct import pack
from typing import TYPE_CHECKING

from source.core.exceptions.data_validation_error.argument_error import (
    validation
)
from source.core.exceptions.data_validation_error.generic_error import (
    FunctionTimeOutError
)
from source.features.tools import sleep
from source.models.classes.enum_classes.state_classes import (
    InComingMotionMode as Imm, OutComingMotionMode as Omm
)
from source.models.constants import (
    CHECK_FREQUENCY_SEC, EMPTY_BYTES, ENABLE, DISABLE,
    OMM_ENABLE_DISABLE_PACK_FORMAT, SET_MOTION_MODE_AWAIT_SEC
)
from source.models.type_aliases import MotionMode_

if TYPE_CHECKING:
    from logging import Logger

    from source.core.network.rtd_receiver_socket import RTDReceiver
    from source.core.network.controller_socket import Controller


validate_literal = validation.validate_literal


class MotionMode:
    """
    Класс для работы с текущими режимами движения робота.
    В режиме 'hold' недоступен режим 'pause'.

    Доступные режимы движения:
        'hold' — Удержание позиции. (Сброс траектории).
        'pause' — Удержание позиции. (Сохранение траектории).
        'move' — Начать/продолжить выполнение заданной траектории.
        'zero_gravity' — Режим FreeDrive (ручное управление).
            (Сброс траектории).
        'jog' — Декартовый джоггинг (джоггинг ЦТИ). (Сброс траектории).
        'joint_jog' — Моторный джоггинг (джоггинг каждым мотором в
            отдельности). (Сброс траектории).
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver
    _logger: Logger

    def __init__(
        self, controller: Controller, rtd_receiver: RTDReceiver, logger: Logger
    ) -> None:

        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._logger = logger

    def get(self) -> str:
        """
        Получить текущий режим движения робота.

        Returns:
            str: Режим движения робота —
                ('hold', 'pause', 'move', 'zero_gravity', 'jog', 'joint_jog')
        """

        return Imm(self._rtd_receiver.rt_data.motion_mode).name

    def set(
        self,
        mode: MotionMode_,
        await_sec: int = SET_MOTION_MODE_AWAIT_SEC
    ) -> bool:
        """
        Установить новый режим движения робота.

        Args:
            mode: Тип движения робота — ('move', 'pause', 'hold').
            await_sec: Лимит времени ожидания (c).
                -1 — безлимитное ожидание.
                0 — одна итерация цикла ожидания (эквивалентно разовому
                    условию).

        Returns:
            True: В случае успешной отправки команды.
        """

        validate_literal('mm', mode)
        if mode == Imm.move.name:
            if self._rtd_receiver.rt_data.buff_fill <= 0:
                self._logger.warning(
                    f'Waypoint queue is empty. Can not process '
                    f'{mode} motion mode'
                )
                return False
            if self.get() == Imm.pause.name:
                self._controller.send(
                    Omm.pause.value,
                    pack(OMM_ENABLE_DISABLE_PACK_FORMAT, DISABLE)
                )
        payload = (
            (
                pack(OMM_ENABLE_DISABLE_PACK_FORMAT, ENABLE)
                if mode == Imm.pause.name else EMPTY_BYTES
            )
        )
        self._controller.send(Omm[mode].value, payload)
        for _ in sleep(await_sec=await_sec, frequency=CHECK_FREQUENCY_SEC):
            if self.get() == mode:
                return True
        raise FunctionTimeOutError('Motion mode', await_sec)
