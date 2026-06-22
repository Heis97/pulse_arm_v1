from __future__ import annotations

import math
import threading
from collections.abc import Generator
from contextlib import contextmanager
from typing import Literal, overload

from api.robot_api_rc5v15.API.source.core.exceptions.generic_error import (
    ApiRuntimeError,
    ApiValueError,
)
from api.robot_api_rc5v15.API.source.features.mathematics.coordinate_system import (
    align_coordinate_system_with_vector,
    offset_coordinate_system,
    rotate_coordinate_system,
)
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    degrees_to_radians,
    radians_to_degrees,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    CoordinateSystemInfoType,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    POSITION_ORIENTATION_LENGTH,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    CoordinateSystemProperty,
    PositionOrientation,
)


class CoordinateSystem:
    """
    Класс для работы с координатными системами робота.
    """

    _position_orientation: PositionOrientation
    _orientation_units: AngleUnits
    _in_context: bool
    _local = threading.local()

    def __init__(
        self,
        position_orientation: PositionOrientation,
        orientation_units: AngleUnits | None = None,
        normalize_angles: bool = True,
    ) -> None:
        """Создаёт пользовательскую систему координат (СК), заданную относительно основания робота.

        Пользовательская СК определяется своей нулевой точкой и ориентацией
        в глобальной системе координат робота. После создания её можно
        использовать для:
        - задания точек движения в локальных координатах,
        - преобразования позиций между системами,
        - упрощения логики управления (например, относительно детали или инструмента).

        Этот класс **не требует подключения к роботу** — он является чисто
        вычислительным инструментом API.

        Args:
            position_orientation (PositionOrientation): Позиция и ориентация
                начала пользовательской СК в формате '(X, Y, Z, Rx, Ry, Rz)', где:
                - '(X, Y, Z)' — координаты в метрах;
                - '(Rx, Ry, Rz)' — углы поворота в указанных единицах.
                Задаётся в **глобальной системе координат основания робота**.
            orientation_units (AngleUnits, optional): Единицы измерения углов:
                - 'deg' — градусы (по умолчанию);
                - 'rad' — радианы.
            normalize_angles (bool, optional): Нормализовать углы в диапазоне
                [-180, 180] / [-pi, pi] в зависимости от единиц измерения углов.
                По умолчанию True.

        Examples:
            >>> # Создать СК, смещённую на 0.5 м по X и повёрнутую вокруг Z на 90°
            >>> from API.coords import CoordinateSystem
            >>> local_cs = CoordinateSystem(
            ...     position_orientation=(0.5, 0.0, 0.0, 0.0, 0.0, 90.0),
            ...     orientation_units='deg'
            ... )

        Notes:
            - Все преобразования позиций между СК выполняются локально на стороне клиента.
        """
        self._in_context: bool = False

        orientation_units = orientation_units or MOTION_SETUP.units
        if normalize_angles:
            position_orientation = self._normalize_angles(
                position_orientation, orientation_units
            )
        self._set_coordinate_system(position_orientation, orientation_units)

    def set(
        self,
        position_orientation: PositionOrientation,
        orientation_units: AngleUnits | None = None,
        normalize_angles: bool = True,
    ) -> None:
        """Обновляет параметры существующей пользовательской системы координат.

        Позволяет динамически изменить положение и ориентацию СК без создания
        нового объекта. Новые значения задаются относительно глобальной системы
        координат основания робота.

        Этот метод **не требует подключения к роботу** — он работает локально
        в памяти приложения, но не работает внутри контекста **in_frame**
        для того же объекта.

        Args:
            position_orientation (PositionOrientation): Новое положение начала СК
                в формате '(X, Y, Z, Rx, Ry, Rz)', где:
                - '(X, Y, Z)' — координаты в метрах;
                - '(Rx, Ry, Rz)' — углы поворота в указанных единицах.
                Задаётся в **глобальной системе координат основания робота**.
            orientation_units (AngleUnits, optional): Единицы измерения углов:
                - 'deg' — градусы (по умолчанию);
                - 'rad' — радианы.
            normalize_angles (bool, optional): Нормализовать углы в диапазоне
                [-180, 180] / [-pi, pi] в зависимости от единиц измерения углов.
                По умолчанию True.

        Examples:
            >>> # Изначально СК задана относительно первой детали
            >>> from API.coords import CoordinateSystem
            >>> cs = CoordinateSystem((0.3, 0.0, 0.2, 0, 0, 0))
            >>> # После замены детали — обновить СК
            >>> cs.set((0.35, 0.02, 0.21, 0, 0, 2.5))  # небольшой сдвиг и поворот

        Notes:
            - Обновление СК **не влияет на уже добавленные в буфер точки** — они
              остаются в той системе, в которой были заданы.
        """

        orientation_units = orientation_units or MOTION_SETUP.units
        if normalize_angles:
            position_orientation = self._normalize_angles(
                position_orientation, orientation_units
            )
        self._set_coordinate_system(position_orientation, orientation_units)

    @overload
    def get(
        self,
        info_type: Literal[
            CoordinateSystemInfoType.POSITION_ORIENTATION, "pose"
        ],
    ) -> PositionOrientation: ...

    @overload
    def get(
        self,
        info_type: Literal[
            CoordinateSystemInfoType.ORIENTATION_UNITS, "units"
        ],
    ) -> AngleUnits: ...

    def get(
        self, info_type: CoordinateSystemInfoType | CoordinateSystemProperty
    ) -> PositionOrientation | AngleUnits:
        """Возвращает запрошенную информацию о пользовательской системе координат.

        Метод позволяет получить либо полную позицию и ориентацию начала СК,
        либо только единицы измерения углов, в зависимости от переданного типа.

        Этот метод **не требует подключения к роботу** — все данные хранятся локально.

        Args:
            info_type (CoordinateSystemInfoType | CoordinateSystemProperty):
                Тип запрашиваемой информации:
                - `CoordinateSystemInfoType.POSITION_ORIENTATION` | `pose` — вернуть
                  `(X, Y, Z, Rx, Ry, Rz)` в метрах и указанных единицах углов;
                - `CoordinateSystemInfoType.ORIENTATION_UNITS` | `units` — вернуть
                  текущие единицы измерения углов (`'deg'` или `'rad'`).

        Returns:
            PositionOrientation | AngleUnits:

                - Если `info_type == POSITION_ORIENTATION` | `pose`: кортеж из 6 чисел
                  `(X, Y, Z, Rx, Ry, Rz)`, где:
                    * `(X, Y, Z)` — координаты в метрах;
                    * `(Rx, Ry, Rz)` — углы в текущих единицах измерения.
                - Если `info_type == ORIENTATION_UNITS` | `units`: строка `'deg'` или `'rad'`.

        Examples:
            >>> from API.types import (
            ...     CoordinateSystemInfoType,
            ... )
            >>> from API.coords import CoordinateSystem
            >>> cs = CoordinateSystem((0.5, 0.0, 0.3, 0, 0, 90), orientation_units='deg')

            >>> # Получить позицию и ориентацию
            >>> pose = cs.get("pose")
            >>> print(f"Начало СК: {pose}")

            >>> # Получить единицы измерения
            >>> units = cs.get("units")
            >>> print(f"Единицы углов: {units}")  # Вывод: deg

        Notes:
            - Возвращаемая позиция всегда задана **в глобальной системе координат
              основания робота**, даже если сама СК — пользовательская.
            - Этот метод **не взаимодействует с контроллером робота** — он просто
              возвращает локально сохранённые данные.
            - Используйте этот метод, например, для логирования или отладки
              текущей конфигурации СК.
        """
        if isinstance(info_type, CoordinateSystemInfoType):
            if info_type == CoordinateSystemInfoType.POSITION_ORIENTATION:
                return self._position_orientation
            if info_type == CoordinateSystemInfoType.ORIENTATION_UNITS:
                return self._orientation_units
        if info_type == "pose":
            return self._position_orientation
        if info_type == "units":
            return self._orientation_units
        raise KeyError("Wrong type of CoordinateSystem info type")

    def copy(self) -> CoordinateSystem:
        """Получить копию текущей системы координат.

        Метод позволяет полностью скопировать параметры текущего объекта класса
        в новый объект класса.

        Returns:
            CoordinateSystem: новый объект с текущими параметрами.

        Examples:
            >>> from API.coords import CoordinateSystem
            >>> cs = CoordinateSystem((0.5, 0.0, 0.3, 0, 0, 90), orientation_units='deg')
            >>> # Скопировать параметры в новый объект
            >>> new_cs = cs.copy()
        """
        return CoordinateSystem(
            list(self._position_orientation).copy(), self._orientation_units
        )

    def with_units(self, orientation_units: AngleUnits) -> CoordinateSystem:
        """Создаёт новую систему координат с теми же параметрами, но в других единицах измерения углов.

        Метод преобразует углы ориентации из текущих единиц в указанные,
        сохраняя физическое положение и ориентацию системы координат.

        Этот метод **не изменяет исходную систему координат** — он возвращает
        новый объект класса `CoordinateSystem`.

        Args:
            orientation_units (AngleUnits): Целевые единицы измерения углов:
                - `'deg'` — градусы;
                - `'rad'` — радианы.

        Returns:
            CoordinateSystem: Новый объект системы координат с теми же физическими
            параметрами, но с углами, выраженными в указанных единицах.

        Examples:
            >>> from API.coords import CoordinateSystem
            >>>
            >>> # СК в градусах
            >>> cs_deg = CoordinateSystem((0.0, 0.0, 0.0, 0.0, 0.0, 90.0), orientation_units="deg")
            >>>
            >>> # Преобразовать в радианы
            >>> cs_rad = cs_deg.with_units("rad")
            >>> pose_rad = cs_rad.get("pose")
            >>> print(f"Угол Z в радианах: {pose_rad[5]:.3f}")  # ≈ 1.571
            >>>
            >>> # Обратное преобразование
            >>> cs_back = cs_rad.with_units("deg")
            >>> assert cs_back.get("pose") == cs_deg.get("pose")

        Notes:
            - Физическая ориентация СК **не меняется** — только численное представление углов.
            - Исходный объект остаётся неизменным — возвращается **новый экземпляр**.
        """

        Validation.literal("angle", orientation_units)

        if orientation_units == self._orientation_units:
            return self.copy()

        current_angles = self._position_orientation[3:]
        if self._orientation_units == "deg" and orientation_units == "rad":
            new_angles = degrees_to_radians(current_angles)
        elif self._orientation_units == "rad" and orientation_units == "deg":
            new_angles = radians_to_degrees(current_angles)
        else:
            raise ApiValueError(
                f"Unsupported unit conversion: {self._orientation_units} → {orientation_units}"
            )

        new_pose = list(self._position_orientation)[:3] + new_angles
        return CoordinateSystem(new_pose, orientation_units)

    def offset(
        self, dx: float = 0, dy: float = 0, dz: float = 0
    ) -> CoordinateSystem:
        """Создаёт новую систему координат, смещённую относительно текущей.

        Новая система координат имеет то же направление осей, что и текущая,
        но её начало смещено на заданные расстояния вдоль **локальных осей**
        текущей системы координат (X, Y, Z).

        Этот метод **не изменяет исходную систему координат** — он возвращает
        новый объект класса `CoordinateSystem`.

        Args:
            dx (float, optional): Смещение вдоль локальной оси X текущей СК, в метрах.
                По умолчанию 0.0.
            dy (float, optional): Смещение вдоль локальной оси Y текущей СК, в метрах.
                По умолчанию 0.0.
            dz (float, optional): Смещение вдоль локальной оси Z текущей СК, в метрах.
                По умолчанию 0.0.

        Returns:
            CoordinateSystem: Новый объект системы координат, смещённый относительно
            текущего. Единицы измерения углов сохраняются без изменений.

        Examples:
            >>> from API.coords import CoordinateSystem
            >>>
            >>> # Базовая СК в начале координат
            >>> base = CoordinateSystem((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            >>>
            >>> # Сместить на 10 см вперёд (вдоль локальной X)
            >>> part = base.offset(dx=0.1)
            >>> print(part.get("pose"))
            ... # (0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
            >>>
            >>> # Сместить вверх от уже повёрнутой СК
            >>> tilted = CoordinateSystem((0.5, 0.2, 0.1, 0.0, 0.0, 90.0))
            >>> above = tilted.offset(dz=0.2)  # 20 см вверх по локальной Z

        Notes:
            - Смещение всегда применяется в **локальной системе координат текущей СК**.
              Например, если текущая СК повёрнута на 90° вокруг Z, то смещение по X
              будет направлено вдоль мировой оси Y.
            - Исходный объект остаётся неизменным — возвращается **новый экземпляр**.
        """
        new_pose = offset_coordinate_system(
            base_pose=self._position_orientation,
            base_units=self._orientation_units,
            dx=dx,
            dy=dy,
            dz=dz,
        )
        return CoordinateSystem(new_pose, self._orientation_units)

    def rotate(
        self,
        drx: float = 0,
        dry: float = 0,
        drz: float = 0,
        orientation_units: AngleUnits | None = None,
    ) -> CoordinateSystem:
        """Создаёт новую систему координат, повёрнутую относительно текущей.

        Новая система координат имеет то же начало, что и текущая, но её оси
        повёрнуты на заданные углы вокруг **локальных осей** текущей системы
        координат (X, Y, Z).

        Этот метод **не изменяет исходную систему координат** — он возвращает
        новый объект класса `CoordinateSystem`.

        Args:
            drx (float, optional): Угол поворота вокруг локальной оси X, в единицах,
                указанных в `orientation_units`. По умолчанию 0.0.
            dry (float, optional): Угол поворота вокруг локальной оси Y, в единицах,
                указанных в `orientation_units`. По умолчанию 0.0.
            drz (float, optional): Угол поворота вокруг локальной оси Z, в единицах,
                указанных в `orientation_units`. По умолчанию 0.0.
            orientation_units (AngleUnits, optional): Единицы измерения входных углов
                (`drx`, `dry`, `drz`). Если не указано, используются единицы
                текущей системы координат.

        Returns:
            CoordinateSystem: Новый объект системы координат, повёрнутый относительно
            текущего. Единицы измерения углов результата совпадают с единицами
            исходной системы координат.

        Examples:
            >>> import math
            >>> from API.coords import CoordinateSystem
            >>>
            >>> # Базовая СК без поворота
            >>> base = CoordinateSystem(
            >>>     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            >>>     orientation_units="deg"
            >>> )
            >>>
            >>> # Повернуть на 90 градусов вокруг локальной Z
            >>> marker = base.rotate(drz=90)
            >>> print(marker.get("pose"))
            ... # (0.0, 0.0, 0.0, 0.0, 0.0, 90.0)
            >>>
            >>> # То же самое, но через радианы
            >>> marker_rad = base.rotate(drz=math.pi / 2, orientation_units="rad")
            >>> assert marker.get("pose") == marker_rad.get("pose")
            >>>
            >>> # Комбинированный поворот
            >>> final = base.rotate(drz=90).rotate(drx=45)

        Notes:
            - Поворот выполняется в **локальной системе координат текущей СК**,
              то есть последовательно: сначала применяется текущая ориентация,
              затем — приращение.
            - Исходный объект остаётся неизменным — возвращается **новый экземпляр**.
            - Результат всегда использует те же единицы измерения углов,
              что и исходная система координат, независимо от `orientation_units`.
        """
        orientation_units = orientation_units or self._orientation_units
        Validation.literal("angle", orientation_units)
        new_pose = rotate_coordinate_system(
            base_pose=self._position_orientation,
            base_units=self._orientation_units,
            drx=drx,
            dry=dry,
            drz=drz,
            input_units=orientation_units,
        )
        return CoordinateSystem(new_pose, self._orientation_units)

    def is_close(self, other: CoordinateSystem, atol: float = 1e-6) -> bool:
        """Сравнивает две системы координат с учётом допуска на погрешность.

        Метод проверяет, совпадают ли две системы координат с заданной абсолютной
        точностью. Сравнение выполняется по всем шести компонентам позы
        (X, Y, Z, Rx, Ry, Rz). Перед сравнением обе системы координат
        приводятся к единой системе единиц измерения углов (градусам).

        Args:
            other (CoordinateSystem): Другая система координат для сравнения.
            atol (float, optional): Абсолютный допуск для сравнения каждой
                компоненты позы. По умолчанию 1e-6 (микрометр для позиции,
                ~0.00006° для углов в градусах).

        Returns:
            bool: True, если все компоненты двух систем координат отличаются
                не более чем на `atol`; иначе False.

        Examples:
            >>> from API.coords import CoordinateSystem
            >>>
            >>> # Две почти идентичные СК
            >>> cs1 = CoordinateSystem((0.1, 0.2, 0.3, 0, 0, 90.0))
            >>> cs2 = CoordinateSystem((0.1000001, 0.2, 0.3, 0, 0, 90.000001))
            >>>
            >>> # Сравнение с допуском по умолчанию
            >>> print(cs1.is_close(cs2))  # True
            >>>
            >>> # Сравнение с более строгим допуском
            >>> print(cs1.is_close(cs2, atol=1e-8))  # False
            >>>
            >>> # Сравнение СК в разных единицах
            >>> import math
            >>> cs_rad = CoordinateSystem(
            ...     (0.1, 0.2, 0.3, 0, 0, math.radians(90.0)),
            ...     orientation_units="rad"
            ... )
            >>> print(cs1.is_close(cs_rad))  # True

        Notes:
            - Метод корректно обрабатывает системы координат с разными единицами
              измерения углов (`'deg'` и `'rad'`).
            - Для компонент позиции (X, Y, Z) допуск задаётся в метрах.
            - Для компонент ориентации (Rx, Ry, Rz) допуск задаётся в тех же
              единицах, что и углы после приведения к градусам.
        """
        if not isinstance(other, CoordinateSystem):
            return False

        self_deg = self.with_units("deg")
        other_deg = other.with_units("deg")

        for a, b in zip(
            self_deg._position_orientation, other_deg._position_orientation
        ):
            if abs(a - b) > atol:
                return False
        return True

    def distance_to(self, other: CoordinateSystem) -> float:
        """Вычисляет евклидово расстояние между началами двух систем координат.

        Метод возвращает расстояние в метрах между точками начала текущей
        и указанной систем координат. Ориентация систем при расчёте **не учитывается** —
        сравниваются только координаты (X, Y, Z).

        Args:
            other (CoordinateSystem): Другая система координат, до начала которой
                необходимо вычислить расстояние.

        Returns:
            float: Расстояние в метрах между началами двух систем координат.

        Examples:
            >>> from API.coords import CoordinateSystem
            >>>
            >>> # Базовая СК в начале координат
            >>> base = CoordinateSystem((0.0, 0.0, 0.0, 0, 0, 0))
            >>>
            >>> # СК детали на расстоянии 0.5 м по X
            >>> part = CoordinateSystem((0.5, 0.0, 0.0, 0, 0, 0))
            >>>
            >>> # Расстояние от базы до детали
            >>> dist = base.distance_to(part)
            >>> print(f"Расстояние до детали: {dist:.3f} м")  # 0.500 м
            >>>
            >>> # Расстояние в трёхмерном пространстве
            >>> tool = CoordinateSystem((0.3, 0.4, 0.0, 0, 0, 0))
            >>> print(base.distance_to(tool))  # 0.5 м (т.к. √(0.3² + 0.4²) = 0.5)

        Notes:
            - Расстояние вычисляется только по линейным компонентам (X, Y, Z).
            - Угловые компоненты (Rx, Ry, Rz) игнорируются.
            - Результат всегда неотрицателен и выражен в метрах.
        """
        if not isinstance(other, CoordinateSystem):
            return 0
        dx = self._position_orientation[0] - other._position_orientation[0]
        dy = self._position_orientation[1] - other._position_orientation[1]
        dz = self._position_orientation[2] - other._position_orientation[2]
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def align_with_vector(
        self,
        target: tuple[float, float, float],
        up_vector: tuple[float, float, float] = (0.0, 0.0, 1.0),
    ) -> CoordinateSystem:
        """Создаёт новую систему координат, выровненную по направлению к цели.

        Новая система координат имеет то же начало, что и текущая, но её оси
        ориентированы так, чтобы:
        - **ось X** была направлена от начала текущей СК к точке `target`;
        - **ось Z** была как можно ближе к указанному `up_vector` (вектору "вверх").

        Args:
            target (Tuple[float, float, float]): Точка в **глобальных координатах**,
                на которую должна быть направлена ось X новой СК.
            up_vector (Tuple[float, float, float], optional): Желаемое направление
                оси Z ("вверх"). По умолчанию (0, 0, 1).

        Returns:
            CoordinateSystem: Новая система координат с тем же началом, что и текущая,
            но с ориентацией, выровненной по направлению к цели.

        Examples:
            >>> from API.coords import CoordinateSystem
            >>>
            >>> # Базовая СК в начале координат
            >>> base = CoordinateSystem((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            >>>
            >>> # Создать СК, направленную на точку (1, 1, 0)
            >>> target_cs = base.align_with_vector(target=(1.0, 1.0, 0.0))
            >>> pose = target_cs.get("pose")
            >>> print(f"Ориентация: {pose[3:]}")  # Rx=0, Ry=0, Rz≈45°

        Notes:
            - Точка `target` должна отличаться от начала текущей СК, иначе направление
              не определится.
            - Вектор `up_vector` не должен быть коллинеарен направлению к цели,
              иначе система координат вырождается.
            - Метод использует ортогонализацию Грама-Шмидта для построения
              правой тройки векторов.
        """

        new_pose = align_coordinate_system_with_vector(
            base_pose=self._position_orientation,
            base_units=self._orientation_units,
            target=target,
            up_vector=up_vector,
        )
        return CoordinateSystem(new_pose, self._orientation_units)

    @contextmanager
    def in_frame(self) -> Generator[None, None, None]:
        """
        Контекстный менеджер для автоматического преобразования координат.

        Контекстный менеджер позволяет автоматически неявно применять
        пользовательскую систему координат во всех методах, в которых можно
        передать в качестве параметра пользовательскую систему координат для
        проведения расчетов относительно пользовательской СК (обычно параметр
        **coordinate_system**).

        При этом, если в метод, ожидающий пользовательскую систему координат,
        СК будет явно передана, то применится явно переданная система координат.

        Контекстный менеджер потокобезопасен и поддерживает вложенность.
        Однако пока контекст активен, система координат **заблокирована**
        для изменения: попытка вызвать сеттеры (например, `set_pose()`)
        приведёт к исключению `RunTimeError`.

        Yields:
            None

        Examples:
            >>> # Определяем пользовательскую систему координат (например, относительно детали)
            >>> part_cs = CoordinateSystem((0.5, 0.2, 0.1, 0.0, 0.0, 90.0))
            >>>
            >>> # Все позы внутри блока интерпретируются относительно `part_cs`
            >>> with part_cs.in_frame():
            ...     robot.motion.linear.add_new_waypoint([0.1, 0.0, 0.0, 0, 0, 0])
            ...     # Робот переместится на 10 см вперёд от начала детали
            ...
            >>> # Явно переданная СК имеет приоритет над контекстом
            >>> another_cs = CoordinateSystem((1.0, 0.0, 0.0, 0, 0, 0))
            >>> with part_cs.in_frame():
            ...     robot.some_method(
            ...         pose=[0, 0, 0, 0, 0, 0],
            ...         coordinate_system=another_cs  # ← будет использована эта СК
            ...     )

        Notes:
            - Систему координат **нельзя изменять**, пока она активна в контексте.
              Попытка вызвать методы вроде `set_pose()` вызовет `RunTimeError`.
            - Контекст является потоколокальным: каждый поток хранит свою активную СК.
        """
        self._in_context = True
        old = getattr(CoordinateSystem._local, "active_context", None)
        CoordinateSystem._local.active_context = self
        try:
            yield
        finally:
            self._in_context = False
            CoordinateSystem._local.active_context = old

    def _set_coordinate_system(
        self,
        position_orientation: PositionOrientation,
        orientation_units: AngleUnits,
    ) -> None:
        """
        Метод для установки системы координат и единиц измерения с валидацией
        входных данных.

        Args:
            position_orientation: Система координат в формате
                (X, Y, Z, Rx, Ry, Rz).
            orientation_units: Единицы измерения углов.
        """
        self._check_context_lock()
        Validation.literal("angle", orientation_units)
        Validation.length(
            position_orientation,
            POSITION_ORIENTATION_LENGTH,
            "position_orientation",
        )
        self._position_orientation = position_orientation
        self._orientation_units = orientation_units

    def _check_context_lock(self):
        if self._in_context:
            raise ApiRuntimeError(
                "Cannot modify CoordinateSystem while it is active in 'in_frame' context. "
                "Create a new CoordinateSystem instead."
            )

    @staticmethod
    def _normalize_angles(
        pose: PositionOrientation, units: AngleUnits
    ) -> PositionOrientation:
        factor = 180.0 if units == "deg" else math.pi
        position = list(pose[:3])
        orientation = [
            ((a + factor) % (2 * factor)) - factor for a in pose[3:]
        ]
        return tuple(position + orientation)

    def __str__(self) -> str:
        pose = ", ".join(f"{x:.2f}" for x in self._position_orientation)
        return f"CoordinateSystem({pose} (3*m, 3*{self._orientation_units}))"

    def __repr__(self):
        return f"CoordinateSystem(pose={self._position_orientation}, units={self._orientation_units!r})"

    def __eq__(self, other):
        if not isinstance(other, CoordinateSystem):
            return False
        return (
            self._position_orientation == other._position_orientation
            and self._orientation_units == other._orientation_units
        )
