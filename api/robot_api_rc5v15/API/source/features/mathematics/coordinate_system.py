from __future__ import annotations

from typing import TYPE_CHECKING, cast

from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    CalculatePlaneError,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    CoordinateSystemInfoType,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    ORIENTATION_SLICE,
    POSITION_SLICE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    PositionOrientation,
)

from .unit_convert import radians_to_degrees

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.ap_interface.motion import CoordinateSystem


def calculate_plane_from_points(
    pO: list[float] | tuple[float, float, float],
    pX: list[float] | tuple[float, float, float],
    pY: list[float] | tuple[float, float, float],
    orientation_units: AngleUnits | None = None,
) -> PositionOrientation:
    """Вычисляет позицию и ориентацию пользовательской системы координат по трём точкам.

    На основе трёх точек в пространстве строится локальная система координат:
    - `pO` — начало системы (origin);
    - вектор `pX - pO` задаёт направление **оси X**;
    - вектор `pY - pO` участвует в построении **оси Y**;
    - ось **Z** вычисляется как векторное произведение X × Y.

    Результат — позиция и ориентация этой системы в глобальной СК основания робота,
    в формате `(X, Y, Z, Rx, Ry, Rz)`.

    Эта функция **не требует подключения к роботу** — она выполняет чисто
    геометрические вычисления.

    Args:
        pO (List[float] | Tuple[float, float, float]): Точка начала координат плоскости
            в формате `(x, y, z)` в метрах.
        pX (List[float] | Tuple[float, float, float]): Точка, задающая направление оси X,
            в формате `(x, y, z)` в метрах. Должна отличаться от `pO`.
        pY (List[float] | Tuple[float, float, float]): Точка, лежащая в плоскости XY,
            в формате `(x, y, z)` в метрах. Не должна лежать на прямой `pO–pX`.
        orientation_units (AngleUnits, optional): Единицы измерения углов в результате:
            - `'deg'` — градусы (по умолчанию);
            - `'rad'` — радианы.

    Returns:
        PositionOrientation: Позиция и ориентация системы координат в формате
        `(X, Y, Z, Rx, Ry, Rz)`, где:
        - `(X, Y, Z)` — координаты точки `pO` в метрах;
        - `(Rx, Ry, Rz)` — углы поворота в указанных единицах,
          представляющие ориентацию осей X, Y, Z.

    Examples:
        >>> # Определить СК по трём точкам на поверхности стола
        >>> from API.coords import calculate_plane_from_points
        >>> origin = (0.3, 0.0, 0.8)      # угол стола
        >>> x_point = (0.4, 0.0, 0.8)     # 10 см вдоль края
        >>> y_point = (0.3, 0.1, 0.8)     # 10 см поперёк
        >>> pose = calculate_plane_from_points(origin, x_point, y_point)
        >>> print(f"СК стола: {pose}")

        >>> # Использовать для создания пользовательской СК
        >>> from API.coords import CoordinateSystem
        >>> table_cs = CoordinateSystem(pose)

    Notes:
        - Точки `pO`, `pX`, `pY` **должны быть неколлинеарны**, иначе ось Z
          не определится (функция может вернуть некорректную ориентацию).
    """
    # Ленивый импорт тяжелых библиотек при необходимости
    import numpy as np
    from scipy.spatial.transform import Rotation

    orientation_units = orientation_units or MOTION_SETUP.units
    point_O = np.array(pO)
    point_X = np.array(pX)
    point_Y = np.array(pY)
    if (
        np.array_equal(point_O, point_X)
        or np.array_equal(point_O, point_Y)
        or np.array_equal(point_X, point_Y)
    ):
        raise CalculatePlaneError(
            "Two or more points defining the direction of the coordinate axes "
            "of the plane coincide, the coordinates of the points must "
            "be unique."
        )
    vecX = point_X - point_O
    vecY = point_Y - point_O
    vecX_norm = vecX / np.linalg.norm(vecX)
    vecY_norm = vecY / np.linalg.norm(vecY)
    vecZ = np.cross(vecX_norm, vecY_norm)
    if np.linalg.norm(vecZ) == 0:
        raise CalculatePlaneError(
            f"The vectors {vecX} and {vecY} are collinear."
        )
    vecZ_norm = vecZ / np.linalg.norm(vecZ)
    rotation_matrix = np.array(
        [
            vecX_norm,
            np.cross(vecZ_norm, vecX_norm),
            vecZ_norm,
        ]
    ).T
    rotation = Rotation.from_matrix(rotation_matrix)
    rpy = rotation.as_euler("xyz", degrees=orientation_units == "deg")
    return point_O.tolist() + rpy.tolist()


def convert_position_orientation(
    coordinate_system: CoordinateSystem,
    position_orientation: PositionOrientation,
    orientation_units: AngleUnits | None = None,
    to_local: bool = False,
) -> PositionOrientation:
    """Преобразует позицию и ориентацию между глобальной и пользовательской системой координат.

    Функция выполняет двунаправленное преобразование:
    - если `to_local=True`: из **глобальной СК (основания робота)**
       в **локальную СК** (заданную объектом `coordinate_system`);
    - если `to_local=False` (по умолчанию): из **локальной СК**
       в **глобальную СК**.

    Это чисто вычислительная функция — **не требует подключения к роботу**.

    Args:
        coordinate_system: Выбранная система координат.
        position_orientation: Конвертируемые позиция и ориентация в единицах
            измерения выбранной системы координат.
        orientation_units: Переданные единицы измерения. По-умолчанию градусы.
            'deg' — градусы.
            'rad' — радианы.
        to_local: Флаг переключения для конвертации из глобальной системы
            координат (основание робота) в локальную (пользовательскую).

    Returns:
        PositionOrientation: Преобразованная позиция и ориентация в том же формате
        `(X, Y, Z, Rx, Ry, Rz)`, в указанных единицах измерения.

    Examples:
        >>> # Создать пользовательскую СК (смещение + поворот)
        >>> from API.coords import CoordinateSystem, convert_position_orientation,
        >>> user_cs = CoordinateSystem((0.5, 0.0, 0.0, 0.0, 0.0, 90.0))

        >>> # Преобразовать точку из локальной СК в глобальную (по умолчанию)
        >>> local_point = (0.1, 0.0, 0.0, 0, 0, 0)  # 10 см вперёд от детали
        >>> global_point = convert_position_orientation(user_cs, local_point)
        >>> print(f"Глобальная позиция: {global_point}")

        >>> # Преобразовать текущую позицию робота в локальную СК
        >>> current_global = robot.motion.linear.get_actual_position()
        >>> current_local = convert_position_orientation(
        ...     user_cs,
        ...     current_global,
        ...     to_local=True
        ... )
        >>> print(f"Позиция относительно детали: {current_local}")

    Notes:
        - Углы должны быть в **тех же единицах**, что указаны в `orientation_units`.
        - Функция **не проверяет достижимость** результирующей позиции —
          это чисто геометрическое преобразование.
    """
    # Ленивый импорт тяжелых библиотек при необходимости
    import numpy as np
    from scipy.spatial.transform import Rotation

    def get_transformation_parameters(
        coordinate_system: CoordinateSystem,
    ) -> tuple[np.ndarray, Rotation]:
        """
        Вычисление вектора трансляции и матрицы поворота.

        Args:
            coordinate_system: Выбранная система координат.
        Returns:
            tuple: Вектор перемещения и Матрица поворота.
        """

        coordinate_system_position_orientation = np.array(
            coordinate_system.get(
                CoordinateSystemInfoType.POSITION_ORIENTATION
            )
        )
        return (
            np.array(coordinate_system_position_orientation[POSITION_SLICE]),
            Rotation.from_euler(
                "xyz",
                coordinate_system_position_orientation[ORIENTATION_SLICE],
                degrees=coordinate_system.get(
                    CoordinateSystemInfoType.ORIENTATION_UNITS
                )
                == "deg",
            ),
        )

    orientation_units = orientation_units or MOTION_SETUP.units
    position = np.array(position_orientation[POSITION_SLICE])
    r = Rotation.from_euler(
        "xyz",
        position_orientation[ORIENTATION_SLICE],
        degrees=orientation_units == "deg",
    )
    translation, rotation = get_transformation_parameters(coordinate_system)
    if to_local:
        position_transformed = rotation.inv().apply(position - translation)
        orientation_transformed = (rotation.inv() * r).as_euler(
            "xyz", degrees=orientation_units == "deg"
        )
    else:
        position_transformed = rotation.apply(position) + translation
        orientation_transformed = (rotation * r).as_euler(
            "xyz", degrees=orientation_units == "deg"
        )
    return cast(
        PositionOrientation,
        position_transformed.tolist() + orientation_transformed.tolist(),
    )


def convert_velocity(
    coordinate_system: CoordinateSystem,
    velocity: PositionOrientation,
    orientation_units: AngleUnits | None = None,
    to_local: bool = False,
) -> PositionOrientation:
    """Преобразует линейную и угловую скорости между глобальной и пользовательской СК.

    В отличие от позиций, скорости преобразуются только поворотом:
    - v_local = R⁻¹ · v_global  (если to_local=True)
    - v_global = R · v_local    (если to_local=False)

    Трансляция (смещение СК) на скорости не влияет.

    Args:
        coordinate_system: Пользовательская система координат.
        velocity: Текущие скорости (vx, vy, vz, wx, wy, wz).
            Линейная часть в м/с, угловая в рад/с.
        orientation_units: Единицы для угловой скорости: 'deg' или 'rad'.
        to_local: Если True — переводит из глобальной в локальную СК.

    Returns:
        PositionOrientation: Преобразованные скорости в формате
        (vx, vy, vz, wx, wy, wz).
    """

    import numpy as np
    from scipy.spatial.transform import Rotation

    orientation_units = orientation_units or MOTION_SETUP.units

    vel_linear = np.array(velocity[POSITION_SLICE])  # [vx, vy, vz]
    vel_angular = np.array(velocity[ORIENTATION_SLICE])  # [wx, wy, wz]

    cs_pose = coordinate_system.with_units("rad").get(
        CoordinateSystemInfoType.POSITION_ORIENTATION
    )

    # R_user_to_base
    r = Rotation.from_euler(
        "xyz",
        cs_pose[ORIENTATION_SLICE],
        degrees=False,
    )

    if to_local:
        # v_local = r⁻¹ · v_global
        R_inv = r.inv()
        vel_linear_out = R_inv.apply(vel_linear)
        vel_angular_out = R_inv.apply(vel_angular)
    else:
        # v_global = r · v_local
        vel_linear_out = r.apply(vel_linear)
        vel_angular_out = r.apply(vel_angular)

    if orientation_units == "deg":
        vel_angular_out = radians_to_degrees(vel_angular_out)

    return cast(
        PositionOrientation,
        vel_linear_out.tolist()
        + (
            vel_angular_out
            if isinstance(vel_angular_out, list)
            else vel_angular_out.tolist()
        ),
    )


def offset_coordinate_system(
    base_pose: PositionOrientation,
    base_units: AngleUnits,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
) -> PositionOrientation:
    """
    Вычисляет новую позу системы координат, смещённую относительно базовой.

    Смещение задаётся в локальной системе координат базовой СК.

    Args:
        base_pose: Поза базовой СК в формате (x, y, z, rx, ry, rz).
        base_units: Единицы измерения углов базовой СК ('deg' или 'rad').
        dx, dy, dz: Смещение вдоль локальных осей X, Y, Z (в метрах).

    Returns:
        PositionOrientation: Новая поза в тех же единицах, что и базовая СК.
    """
    # Ленивый импорт тяжелых библиотек при необходимости
    import numpy as np
    from scipy.spatial.transform import Rotation

    base_pos = np.array(base_pose[POSITION_SLICE])
    base_rot = Rotation.from_euler(
        "xyz", base_pose[ORIENTATION_SLICE], degrees=base_units == "deg"
    )

    local_offset = np.array([dx, dy, dz])
    global_offset = base_rot.apply(local_offset)

    new_pos = base_pos + global_offset
    return (*new_pos.tolist(), *base_pose[ORIENTATION_SLICE])


def rotate_coordinate_system(
    base_pose: PositionOrientation,
    base_units: AngleUnits,
    drx: float = 0.0,
    dry: float = 0.0,
    drz: float = 0.0,
    input_units: AngleUnits | None = None,
) -> PositionOrientation:
    """
    Вычисляет новую позу системы координат, повёрнутую относительно базовой.

    Поворот выполняется в локальной системе координат базовой СК.
    Входные углы могут быть заданы в градусах или радианах.

    Args:
        base_pose: Поза базовой СК в формате (x, y, z, rx, ry, rz).
        base_units: Единицы измерения углов базовой СК ('deg' или 'rad').
        drx, dry, drz: Углы поворота вокруг локальных осей X, Y, Z.
        input_units: Единицы измерения входных углов.
                     Если None — используются единицы базовой СК (`base_units`).

    Returns:
        PositionOrientation: Новая поза в тех же единицах, что и базовая СК.
    """
    # Ленивый импорт тяжелых библиотек при необходимости
    from scipy.spatial.transform import Rotation

    if input_units is None:
        input_units = base_units

    delta_rot = Rotation.from_euler(
        "xyz", [drx, dry, drz], degrees=input_units == "deg"
    )

    base_rot = Rotation.from_euler(
        "xyz", base_pose[ORIENTATION_SLICE], degrees=base_units == "deg"
    )

    # Локальный поворот: R_new = R_base * R_delta
    new_rot = base_rot * delta_rot

    # Сохраняем в единицах базовой СК
    new_orientation = new_rot.as_euler("xyz", degrees=base_units == "deg")

    return cast(
        PositionOrientation,
        (
            *base_pose[POSITION_SLICE],
            *new_orientation.tolist(),
        ),
    )


def align_coordinate_system_with_vector(
    base_pose: PositionOrientation,
    base_units: AngleUnits,
    target: tuple[float, float, float],
    up_vector: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> PositionOrientation:
    """Вычисляет позу новой системы координат, выровненной по направлению к цели.

    Новая система координат имеет то же начало, что и базовая, но её оси
    ориентированы так, чтобы:
    - **ось X** была направлена от начала базовой СК к точке `target`;
    - **ось Z** была как можно ближе к указанному `up_vector` (вектору "вверх").

    Args:
        base_pose (PositionOrientation): Поза базовой СК в формате (x, y, z, rx, ry, rz).
        base_units (AngleUnits): Единицы измерения углов базовой СК ('deg' или 'rad').
        target (Tuple[float, float, float]): Точка в **глобальных координатах**,
            на которую должна быть направлена ось X новой СК.
        up_vector (Tuple[float, float, float], optional): Желаемое направление
            оси Z ("вверх"). По умолчанию (0, 0, 1).

    Returns:
        PositionOrientation: Поза новой системы координат в тех же единицах,
        что и базовая СК.

    Raises:
        ValueError: Если цель совпадает с началом или up_vector нулевой/вырожденный.
    """
    # Ленивый импорт тяжелых библиотек при необходимости
    import numpy as np
    from scipy.spatial.transform import Rotation

    origin = np.array(base_pose[:3])
    np_target = np.array(target)
    np_up_vector = np.array(up_vector)

    direction = np_target - origin
    norm_dir = np.linalg.norm(direction)
    if norm_dir < 1e-12:
        raise ValueError(
            "Target point must be different from the origin of the coordinate system"
        )

    x_axis = direction / norm_dir

    up_norm = np.linalg.norm(np_up_vector)
    if up_norm < 1e-12:
        raise ValueError("Up vector must be non-zero")

    np_up_vector = np_up_vector / up_norm

    if abs(np.dot(x_axis, np_up_vector)) > 0.999:
        if abs(x_axis[0]) < 0.9:
            temp = np.array([1.0, 0.0, 0.0])
        else:
            temp = np.array([0.0, 1.0, 0.0])
        np_up_vector = np.cross(x_axis, temp)
        np_up_vector = np_up_vector / np.linalg.norm(np_up_vector)

    z_axis = np_up_vector - np.dot(np_up_vector, x_axis) * x_axis
    z_axis = z_axis / np.linalg.norm(z_axis)

    y_axis = np.cross(z_axis, x_axis)

    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
    rotation = Rotation.from_matrix(rotation_matrix)

    new_orientation = rotation.as_euler("xyz", degrees=base_units == "deg")
    return cast(
        PositionOrientation, (*origin.tolist(), *new_orientation.tolist())
    )
