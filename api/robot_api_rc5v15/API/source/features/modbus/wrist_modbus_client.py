from __future__ import annotations

import logging

from api.robot_api_rc5v15.API.source.ap_interface.wrist.rs_485 import WristRS485
from api.robot_api_rc5v15.API.source.core.exceptions.generic_error import ApiValueError

from .base_modbus_client import BaseModbusRS485Client


class WristModbusRS485Client(BaseModbusRS485Client):
    """Готовый Modbus RTU-клиент для работы с устройствами через RS-485 порт запястья робота.

    Предоставляет удобный способ взаимодействия с Modbus-устройствами, подключёнными
    к встроенному RS-485-интерфейсу платы запястья. Клиент автоматически создаёт
    и управляет транспортным слоем (`WristRS485`).

    Поддерживает все стандартные операции Modbus RTU: чтение/запись регистров,
    работа с дискретными входами/выходами и т.д.

    Examples:
        >>> client = WristModbusRS485Client(host="192.168.1.50")
        >>> client.connect()
        ... regs = client.read_holding_registers(address=0x01)
        >>> client.close()

        >>> with WristModbusRS485Client(host="192.168.1.50") as client:
        ...     regs = client.read_holding_registers(address=0x01)

        >>> client = WristModbusRS485Client(wrist_rs_485=robot.wrist.rs_485)
        >>> client.connect()
        ... regs = client.read_holding_registers(address=0x01)
        >>> client.close()
    """

    def __init__(
        self,
        *,
        host: str | None = None,
        timeout: float = 1.0,
        wrist_rs_485: WristRS485 | None = None,
        logger: logging.Logger | None = None,
        log_rs485: bool = False,
    ):
        """Инициализирует Modbus-клиент для RS-485 порта запястья.

        Допускает два режима создания:
        1. **Автоматический** — по IP-адресу контроллера (параметр `host`);
        2. **Ручной** — с передачей уже настроенного экземпляра `WristRS485`.

        Args:
            host (Optional[str]): IP-адрес контроллера робота.
                Обязателен, если не указан `wrist_rs_485`.
            timeout (float): Таймаут подключения к контроллеру (в секундах).
                Используется только при автоматическом создании транспорта.
            wrist_rs_485 (Optional[WristRS485]): Готовый экземпляр RS-485 интерфейса.
            logger (Optional[logging.Logger]): Логгер для Modbus-клиента.
            log_rs485 (bool): Включить логирование на уровне RS-485 транспорта.
                Если `True`, переданный `logger` также используется в `WristRS485`.

        Raises:
            ValueError: Если указаны одновременно `host` и `wrist_rs_485`,
                или если не задан ни один из них.

        Notes:
            - Параметры передаются **только по имени** (keyword-only) для ясности.
        """
        if wrist_rs_485 is not None:
            if host is not None:
                raise ApiValueError(
                    "Cannot specify both 'host' and 'wrist_rs_485' while "
                    "creating 'WristModbusRS485Client' object"
                )
            wrist_rs_485 = wrist_rs_485
        else:
            if host is None:
                raise ApiValueError(
                    "One of the 'host' or 'wrist_rs_485' parameters must be "
                    "set when creating the 'WristModbusRS485Client' object."
                )
            wrist_rs_485 = WristRS485(
                host=host,
                timeout=timeout,
                logger=logger if log_rs485 else None,
            )

        super().__init__(rs_485_protocol=wrist_rs_485, logger=logger)
