from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Tuple, cast

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    GRAVITY_VECTOR_COUNT,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller


class ControllerGravity(LoggerMixin):
    """
    Класс для управления ориентацией контроллера через задание вектора
    гравитации. Изменяет физическое положение/ориентацию контроллера путем
    имитации заданного вектора гравитационного воздействия.
    """

    _controller: Controller
    _connection_state: ConnectionState

    def __init__(
        self,
        controller: Controller,
        connection_state: ConnectionState,
        logger: logging.Logger | None,
    ) -> None:
        self._controller = controller
        self._connection_state = connection_state
        self._set_logger(logger)

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set(self, gravity_vector: tuple[float, float, float]) -> bool:
        """Устанавливает вектор гравитации для коррекции ориентации контроллера.

        Метод задаёт направление имитируемого гравитационного воздействия,
        используемого системой управления для адаптации к физической ориентации
        робота (например, при установке на наклонной поверхности или вверх ногами).

        Вектор **должен быть предварительно нормирован** пользователем:
        его длина должна соответствовать ускорению свободного падения ||v|| ≈ 1.0.
        API не выполняет автоматическую нормировку.

        Args:
            gravity_vector: Вектор гравитации,в формате (X, Y, Z),
                где (X, Y, Z) — ориентация вектора, нормированная на g≈9.81 м/с².
        Returns:
            True: В случае успешной отправки команды, иначе False

        Examples:
            >>> # Стандартная ориентация (в единицах g)
            >>> robot.controller.gravity.set((0.0, 0.0, -1.0))

            >>> # Робот установлен "вверх ногами"
            >>> robot.controller.gravity.set((0.0, 0.0, 1.0))

            >>> # Наклон на 45 градусов вперёд
            >>> import math
            >>> angle = math.radians(45)
            >>> robot.controller.gravity.set((0.0, -math.sin(angle), -math.cos(angle)))

        Notes:
            - Убедитесь, что вектор **нормирован** перед передачей — иначе
              поведение робота может быть непредсказуемым.
            - Неправильная ориентация гравитации может привести к ошибкам
              позиционирования или срабатыванию защиты по усилию.
        """

        Validation.length(
            gravity_vector, GRAVITY_VECTOR_COUNT, "gravity_vector"
        )
        inverted_vector = (
            -gravity_vector[0],
            -gravity_vector[1],
            -gravity_vector[2],
        )
        self._write_log(
            "debug", f"Setting robot gravity vector as {gravity_vector}"
        )
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_GRAVITY,
            inverted_vector,
        )

    @handle_connection(available_in_read_only=False)
    def get(self) -> tuple[float, float, float] | None:
        """Возвращает текущий активный вектор гравитации, применяемый к контроллеру.

        Вектор отражает имитацию гравитационного воздействия, используемую
        для коррекции ориентации контроллера. Значения нормированы относительно
        стандартного ускорения свободного падения: g = -9.81 м/с².
        Например, значение `(0.0, 0.0, -1.0)` соответствует стандартной
        ориентации с гравитацией, направленной вниз по оси Z.

        Returns:
            Tuple[float, float, float] | None: Трёхмерный вектор гравитации
                в формате (X, Y, Z), выраженный в **единицах g** (безразмерный
                вектор, где 1.0 ≈ 9.81 м/с²).
                Пример: (0.0, 0.0, -1.0) — стандартная ориентация.
                Возвращает `None`, если получить заданный вектор гравитации не удалось.

        Examples:
            >>> gravity = robot.controller.gravity.get()
            >>> if gravity is not None:
            ...     x, y, z = gravity
            ...     print(f"Вектор гравитации: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            ... else:
            ...     print("Не удалось получить вектор гравитации")

        Notes:
            - Возвращаемый вектор **безразмерный**: он показывает направление
              и относительную величину гравитации в единицах стандартного g.
            - Ось Z обычно направлена вниз при стандартной установке робота.
        """

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_GRAVITY
        )
        inverted_response = (
            -response[0],
            -response[1],
            -response[2],
        )
        return cast(Tuple[float, float, float], inverted_response)
