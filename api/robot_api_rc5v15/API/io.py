"""
Модуль для взаимодействия с внешними устройствами через промышленные интерфейсы.

Публичный API:
    WristRS485: Управление встроенным RS-485-портом запястья робота.
    WristModbusRS485Client: Готовый Modbus RTU-клиент для устройств на запястье.
    BaseModbusRS485Client: Базовый класс для кастомных Modbus-реализаций.

Примеры:
>>> from API import io
>>> from API.io import WristModbusRS485Client
"""

from .source.ap_interface.wrist.rs_485 import WristRS485
from .source.features.modbus.base_modbus_client import BaseModbusRS485Client
from .source.features.modbus.wrist_modbus_client import WristModbusRS485Client

__all__ = ["WristRS485", "WristModbusRS485Client", "BaseModbusRS485Client"]
