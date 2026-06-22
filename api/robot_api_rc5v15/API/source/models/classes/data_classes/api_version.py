from __future__ import annotations

from dataclasses import dataclass, field

from api.robot_api_rc5v15.API.source.models.classes.data_classes.service_types import DhModelParams


@dataclass
class Version:
    """
    Версия API. Создается из версии ПО ядра, протокола ядра и версии ПО API.

    Attributes:
        proto_version: Версия протокола ядра.
        core_version: Версия ПО ядра робота.
        api_version: Версия ПО API.
    """

    proto_version: int = 0x02000600
    core_version: str = "1.5.1"
    api_version: int = 4

    def get_full_version(self):
        return (
            f"{self.core_version}.{hex(self.proto_version)}/{self.api_version}"
        )


@dataclass
class RobotInfo:
    """
    Дата-класс с технической информацией о текущей модели робота и ПО.

    Attributes:
        robot_model: Имя модели робота;
        client_version: Полная версия клиента;
        dh_model: DH параметры робота.
    """

    robot_model: str = "rc10"
    client_version: str = Version().get_full_version()
    dh_model: DhModelParams = field(default_factory=DhModelParams)
