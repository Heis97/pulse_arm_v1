from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.exceptions.data_error.generic_error import (
    CalibrateTcpError,
)
from api.robot_api_rc5v15.API.source.features.decorators import transforms_coordinates
from api.robot_api_rc5v15.API.source.features.mathematics.coordinate_system import (
    convert_position_orientation,
)
from api.robot_api_rc5v15.API.source.features.validation import Validation
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import MOTION_SETUP
from api.robot_api_rc5v15.API.source.models.constants import POSITION_ORIENTATION_LENGTH
from api.robot_api_rc5v15.API.source.models.type_aliases import AngleUnits, PositionOrientation

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.ap_interface.motion.coordinate_system import (
        CoordinateSystem,
    )


@dataclass
class TcpCalibrationResult:
    """Результат калибровки центра инструмента (TCP) методом наименьших квадратов.

    Этот класс содержит как рассчитанные геометрические параметры, так и метрики
    качества калибровки, позволяющие оценить достоверность результата.

    Attributes:
        tcp: Смещение центра инструмента относительно центра фланца робота.
            Формат: `(X, Y, Z, Rx, Ry, Rz)`.
            ВАЖНО: Метод 4 точек определяет **только позицию** (X, Y, Z).
            Значения `(Rx, Ry, Rz)` являются фиктивными (заполняются из
            `default_tcp_orientation`, по умолчанию `0.0, 0.0, 0.0`) и не несут
            геометрической информации о повороте инструмента.
        target_point: Координаты калибровочной точки (физического острия),
            вокруг которой производился подвод. Формат: `(X, Y, Z, Rx, Ry, Rz)`.
            Выражена в той системе координат, в которая была передана (в СК
            основания, если не была передана)
        errors: Список абсолютных погрешностей (невязок) в метрах для каждой
            из входных точек. Показывает, насколько далеко рассчитанная
            сфера отклоняется от идеальной в каждой конкретной позе.
        rmse: Среднеквадратичная ошибка (Root Mean Square Error) в метрах.
            Главный индикатор качества калибровки.
        max_error: Максимальная абсолютная погрешность среди всех точек в метрах.
            Полезна для выявления "выбросов".
        units: Единицы измерения углов, использованные при расчете ('deg' или 'rad').
    """

    tcp: PositionOrientation
    target_point: PositionOrientation
    errors: list[float]
    rmse: float
    max_error: float
    orientation_units: AngleUnits


@transforms_coordinates(system_param="coordinate_system")
def calibrate_tcp(
    poses: list[PositionOrientation],
    orientation_units: AngleUnits | None = None,
    coordinate_system: CoordinateSystem | None = None,
    default_tcp_orientation: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> TcpCalibrationResult:
    """Вычисляет смещение TCP (Tool Center Point) методом наименьших квадратов.

    Алгоритм реализует классический "метод 4 точек" (метод сферы): острие инструмента
    подводится к одной физической точке пространства с 4 или более разными ориентациями
    фланца. На основе кинематического уравнения `P_target = P_flange + R_flange * P_tcp`
    решается переопределенная система линейных уравнений.

    Математическая модель инвариантна к выбору системы координат: расчет выполняется
    непосредственно в тех координатах, в которых предоставлены входные `poses`.

    ОГРАНИЧЕНИЯ МЕТОДА:
    Данный метод определяет **ТОЛЬКО позицию** (X, Y, Z) центра инструмента.
    Он физически не может определить его ориентацию (Rx, Ry, Rz). Ориентация
    должна быть задана самостоятельно и может быть передана через параметр
    `default_tcp_orientation` для добавления к рассчитанной позиции.

    Args:
        poses: Список поз фланца робота. Каждая поза должна содержать 6 значений:
            `[X, Y, Z, Rx, Ry, Rz]`. Координаты позиции должны передаваться в
            метрах и градусах/радианах. Требуется минимум 4 позы с максимально
            различными ориентациями.
        orientation_units: Единицы измерения углов во входных данных
            ('deg' или 'rad'). Если `None`, используются единицы по умолчанию.
        coordinate_system: (Опционально) Пользовательская система координат,
            относительно которой заданы входные `poses`. Если указана,
            результирующая `target_point` будет преобразована и выражена в
            этой локальной СК.
        default_tcp_orientation: Известная ориентация инструмента `(Rx, Ry, Rz)`.
            По умолчанию `(0.0, 0.0, 0.0)`.

    Returns:
        OptimizationResult: Объект с рассчитанным TCP, целевой точкой и
            метриками погрешности.

    Examples:
        >>> # Калибровка в базовой системе координат
        >>> poses_base = [
        ...     [0.474, -0.357, 0.205, 160.893, -1.179, -169.567],
        ...     [0.276, -0.467, 0.061, 102.712, 7.897, 130.757],
        ...     [0.431, -0.277, 0.209, 171.306, 13.945, 82.891],
        ...     [0.602, -0.319, 0.142, -141.302, 5.615, 91.071],
        ... ]
        >>> result = calibrate_tcp(poses_base, orientation_units='deg')
        >>> print(f"TCP: {result.tcp}, Target (Base): {result.target_point}")

        >>> # Калибровка относительно пользовательской СК (например, стола)
        >>> result_local = calibrate_tcp(
        ...     poses_local,
        ...     orientation_units='deg',
        ...     coordinate_system=table_cs
        ... )
        >>> print(f"Target (Local): {result_local.target_point}")
    """

    import numpy as np
    from scipy.spatial.transform import Rotation

    n_points = len(poses)
    Validation.value(n_points, (4, None), "number of poses")
    for i, pose in enumerate(poses, 1):
        Validation.length(
            pose,
            POSITION_ORIENTATION_LENGTH,
            f"length of {i} pose not equal {POSITION_ORIENTATION_LENGTH}",
        )

    orientation_units = orientation_units or MOTION_SETUP.units
    is_degrees = orientation_units == "deg"
    seq = "xyz"

    # A * X = B
    # X = [tcp_x, tcp_y, tcp_z, target_x, target_y, target_z]^T
    A_mat = np.zeros((3 * n_points, 6))
    B_mat = np.zeros((3 * n_points, 1))
    I_mat = np.eye(3)

    for i, pose in enumerate(poses):
        x, y, z, rx, ry, rz = pose
        P_i = np.array([x, y, z])

        rot_matrix = Rotation.from_euler(
            seq, [rx, ry, rz], degrees=is_degrees
        ).as_matrix()

        row_idx = 3 * i
        A_mat[row_idx : row_idx + 3, 0:3] = rot_matrix
        A_mat[row_idx : row_idx + 3, 3:6] = -I_mat
        B_mat[row_idx : row_idx + 3, 0] = -P_i

    X_mat: np.ndarray
    X_mat, residuals, rank, s = np.linalg.lstsq(A_mat, B_mat, rcond=None)

    P_tcp = X_mat[0:3].flatten()
    P_target = X_mat[3:6].flatten()

    errors = []
    for i, pose in enumerate(poses):
        x, y, z, rx, ry, rz = pose
        P_i = np.array([x, y, z])

        rot_matrix = Rotation.from_euler(
            seq, [rx, ry, rz], degrees=is_degrees
        ).as_matrix()

        calculated_target = P_i + rot_matrix @ P_tcp
        error = float(np.linalg.norm(calculated_target - P_target))
        errors.append(error)

    rmse = float(np.sqrt(np.mean(np.array(errors) ** 2)))
    max_error = float(np.max(errors))

    if rank < 6:
        raise CalibrateTcpError("orientations are to similar")

    tcp_6d = tuple(P_tcp.tolist() + list(default_tcp_orientation))
    target_point_6d = tuple(P_target.tolist() + [0.0, 0.0, 0.0])

    if coordinate_system is not None:
        target_point_6d = convert_position_orientation(
            coordinate_system=coordinate_system,
            position_orientation=target_point_6d,
            orientation_units=orientation_units,
        )

    return TcpCalibrationResult(
        tcp=tcp_6d,
        target_point=target_point_6d,
        errors=errors,
        rmse=rmse,
        max_error=max_error,
        orientation_units=orientation_units,
    )
