from __future__ import annotations

import math
from collections.abc import Generator
from typing import Any

from api.robot_api_rc5v15.API.source.models.type_aliases import PositionOrientation

from .dataclasses import GCodeState
from .utils import gcode_val_to_meters

# Конфигурация рабочих плоскостей для круговой интерполяции.
# Сопоставляет плоскости (G17/G18/G19) с индексами осей в 6D-позе
# робота и стандартными параметрами смещения центра дуги (I, J, K).
#
# Структура значения:
# - `axes`: `(u, v, ortho)` — индексы осей в кортеже (X, Y, Z, A, B, C).
# - `offsets`: `(param_u, param_v)` — G-кодовые параметры центра дуги
#  относительно стартовой точки.
#
# Examples:
#    G17 G2 X50 Y50 I50 J0   ; XY-плоскость, offsets=("I", "J")
#    G18 G3 X50 Z50 I50 K0   ; XZ-плоскость, offsets=("I", "K")

_PLANE_CONFIG: dict[str, Any] = {
    "XY": {"axes": (0, 1, 2), "offsets": ("I", "J")},
    "XZ": {"axes": (0, 2, 1), "offsets": ("I", "K")},
    "YZ": {"axes": (1, 2, 0), "offsets": ("J", "K")},
}


def interpolate_linear(
    start: PositionOrientation,
    end: PositionOrientation,
    step_m: float,
) -> Generator[PositionOrientation, None, None]:
    """Разбиение линейного перемещения на сегменты заданной длины.

    Вычисляет промежуточные 6D-позы вдоль прямой от `start` до `end`,
    гарантируя, что расстояние между соседними точками не превышает `step_m`.
    Интерполяция применяется линейно ко всем 6 осям (X, Y, Z, A, B, C).

    Args:
        start: Начальная поза (X, Y, Z, A, B, C) в метрах и градусах.
        end: Конечная поза (X, Y, Z, A, B, C) в метрах и градусах.
        step_m: Максимальная длина сегмента в метрах.

    Yields:
        Последовательность промежуточных поз. Всегда включает конечную точку `end`.
        Если евклидово расстояние по XYZ < 1e-6 м, выдаётся только `end`.

    Examples:
        >>> start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        >>> end = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0)  # 100 мм по X
        >>> list(interpolate_linear(start, end, step_m=0.03))
        [(0.025, 0.0, 0.0, 0.0, 0.0, 0.0), (0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
         (0.075, 0.0, 0.0, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.0, 0.0, 0.0)]
    """

    vec = [ev - sv for sv, ev in zip(start, end)]
    dist = math.hypot(*vec[:3])

    if dist < 1e-6:
        yield end
        return

    steps = max(1, math.ceil(dist / step_m))
    for i in range(1, steps + 1):
        t = i / steps
        interp_pose = tuple(sv + v * t for sv, v in zip(start, vec))

        yield interp_pose


def debug_arc(
    start: PositionOrientation,
    end: PositionOrientation,
    params: dict[str, Any],
    is_cw: bool,
    state: GCodeState,
) -> None:
    """Диагностический вывод геометрических параметров дуги.

    Вычисляет и выводит в консоль центр дуги, рассчитанный и фактический
    радиусы, а также погрешность замыкания. Используется для отладки
    постпроцессоров и проверки валидности G-кода.

    Args:
        start: Начальная точка дуги (X, Y, Z, A, B, C) в метрах.
        end: Конечная точка дуги (X, Y, Z, A, B, C) в метрах.
        params: Параметры команды (I, J, K), задающие смещение центра.
        is_cw: Флаг направления обхода (True = по часовой, False = против).
        state: Текущее состояние интерпретатора. Используется для выбора
            плоскости и единиц измерения.

    Examples:
        >>> debug_arc(start, end, {"I": 0.05, "J": 0.05}, False, state)
        ARC XY | I=50.0 J=50.0 mm | Center=(50.00, 50.00) | R_calc=70.711 | R_actual=70.711 | ΔR=0.000 mm
    """
    cfg = _PLANE_CONFIG[state.plane]
    u, v, _ = cfg["axes"]
    i = gcode_val_to_meters(params.get(cfg["offsets"][0], 0.0), state.units_mm)
    j = gcode_val_to_meters(params.get(cfg["offsets"][1], 0.0), state.units_mm)

    center = (start[u] + i, start[v] + j)
    r_calc = math.hypot(i, j)
    r_actual = math.hypot(end[u] - center[0], end[v] - center[1])

    print(
        f"ARC {state.plane} | I={i * 1000:.1f} J={j * 1000:.1f} mm | "
        f"Center=({center[0] * 1000:.2f}, {center[1] * 1000:.2f}) | "
        f"R_calc={r_calc * 1000:.3f} | R_actual={r_actual * 1000:.3f} | "
        f"ΔR={abs(r_calc - r_actual) * 1000:.3f} mm"
    )


def compute_arc_geometry(
    start: PositionOrientation,
    end: PositionOrientation,
    params: dict[str, Any],
    is_cw: bool,
    state: GCodeState,
) -> dict[str, Any]:
    """Вычисляет геометрические параметры дуги (центр, радиус, углы).

    Рассчитывает координаты центра, радиус, начальный и конечный углы,
    а также угловое смещение (delta) с нормализацией для CW/CCW.
    Является единым источником данных о геометрии для интерполяции и
    нативных команд дуг.

    Args:
        start: Начальная поза (X, Y, Z, A, B, C) в метрах.
        end: Конечная поза (X, Y, Z, A, B, C) в метрах.
        params: Параметры смещения центра (I, J, K) в единицах G-code.
        is_cw: Флаг направления обхода (True = по часовой стрелке).
        state: Состояние интерпретатора (используется для выбора плоскости
            и единиц измерения).

    Returns:
        Словарь с вычисленными параметрами:
        - ``c_u``, ``c_v``: Координаты центра в плоскости дуги (м).
        - ``r``: Радиус дуги (м).
        - ``th_s``: Начальный угол (рад).
        - ``delta``: Угловое смещение от старта до конца (рад).
        - ``u_idx``, ``v_idx``, ``ortho_idx``: Индексы осей.
        - ``start``, ``end``: Исходные точки.

    Raises:
        ValueError: Если вычисленный радиус меньше 1e-6 м (вырожденная дуга).

    Examples:
        >>> geo = _compute_arc_geometry(start, end, {"I": 0.05, "J": 0.0}, False, state)
        >>> geo["r"]
        0.05
    """
    cfg = _PLANE_CONFIG[state.plane]
    u_idx, v_idx, ortho_idx = cfg["axes"]
    i_key, j_key = cfg["offsets"]

    i_val = gcode_val_to_meters(params.get(i_key, 0.0), state.units_mm)
    j_val = gcode_val_to_meters(params.get(j_key, 0.0), state.units_mm)

    c_u = start[u_idx] + i_val
    c_v = start[v_idx] + j_val
    radius = math.hypot(i_val, j_val)

    if radius < 1e-6:
        raise ValueError(f"Arc radius too small ({radius * 1000:.2f} mm)")

    th_s = math.atan2(start[v_idx] - c_v, start[u_idx] - c_u)
    th_e = math.atan2(end[v_idx] - c_v, end[u_idx] - c_u)
    delta = th_e - th_s

    if abs(delta) < 1e-6:
        # Полная окружность
        delta = -2 * math.pi if is_cw else 2 * math.pi
    else:
        # Нормализация направления: CCW -> (+), CW -> (-)
        if is_cw and delta > 0:
            delta -= 2 * math.pi
        elif not is_cw and delta < 0:
            delta += 2 * math.pi

    return {
        "c_u": c_u,
        "c_v": c_v,
        "r": radius,
        "th_s": th_s,
        "delta": delta,
        "u_idx": u_idx,
        "v_idx": v_idx,
        "ortho_idx": ortho_idx,
        "start": start,
        "end": end,
    }


def pose_at_angle(geo: dict[str, Any], theta: float) -> PositionOrientation:
    """Вычисляет 6D-позу на дуге по заданному углу."""
    pose = [0.0] * 6
    pose[geo["u_idx"]] = geo["c_u"] + geo["r"] * math.cos(theta)
    pose[geo["v_idx"]] = geo["c_v"] + geo["r"] * math.sin(theta)

    # Линейная интерполяция для ортогональной оси и ориентации
    t = (theta - geo["th_s"]) / geo["delta"] if geo["delta"] != 0 else 0.0
    pose[geo["ortho_idx"]] = (
        geo["start"][geo["ortho_idx"]]
        + (geo["end"][geo["ortho_idx"]] - geo["start"][geo["ortho_idx"]]) * t
    )
    pose[3:6] = tuple(
        s + (e - s) * t for s, e in zip(geo["start"][3:], geo["end"][3:])
    )
    return tuple(pose)


def process_native_arc_points(
    start: PositionOrientation,
    end: PositionOrientation,
    params: dict[str, Any],
    is_cw: bool,
    state: GCodeState,
) -> tuple[PositionOrientation, PositionOrientation]:
    """Вычисляет промежуточную (via) точку для нативной круговой интерполяции.

    Рассчитывает точку на середине дуги, необходимую для API-команд типа
    `MoveC`, требующих задания промежуточной и конечной позиций.
    Ориентация инструмента в via-точке вычисляется как среднее арифметическое
    начальной и конечной ориентаций.

    Args:
        start: Начальная поза (X, Y, Z, A, B, C) в метрах.
        end: Конечная поза (X, Y, Z, A, B, C) в метрах.
        params: Параметры смещения центра (I, J, K) в единицах G-code.
        is_cw: Флаг направления обхода (True = по часовой стрелке).
        state: Состояние интерпретатора (используется плоскость и единицы).

    Returns:
        Кортеж `(via_pose, end_pose)`, где `via_pose` — вычисленная середина
        дуги, а `end_pose` — исходная конечная точка.

    Raises:
        ValueError: Если радиус дуги меньше 1e-6 м (делегировано из `_compute_arc_geometry`).
    """
    g = compute_arc_geometry(start, end, params, is_cw, state)
    th_mid = g["th_s"] + g["delta"] / 2.0

    via = [0.0] * 6
    via[g["u_idx"]] = g["c_u"] + g["r"] * math.cos(th_mid)
    via[g["v_idx"]] = g["c_v"] + g["r"] * math.sin(th_mid)
    via[g["ortho_idx"]] = (
        g["start"][g["ortho_idx"]] + g["end"][g["ortho_idx"]]
    ) / 2.0
    via[3:6] = tuple(
        (s + e) / 2.0 for s, e in zip(g["start"][3:], g["end"][3:])
    )

    return tuple(via), g["end"]


def interpolate_arc(
    start: PositionOrientation,
    end: PositionOrientation,
    params: dict[str, Any],
    is_cw: bool,
    step_m: float,
    state: GCodeState,
) -> Generator[PositionOrientation, None, None]:
    """Генератор промежуточных точек для аппроксимации дуги линейными сегментами.

    Разбивает круговую траекторию на последовательность 6D-поз с длиной шага,
    не превышающей `step_m`. Применяет линейную интерполяцию к пространственным
    координатам и углам ориентации инструмента.

    Args:
        start: Начальная поза (X, Y, Z, A, B, C) в метрах.
        end: Конечная поза (X, Y, Z, A, B, C) в метрах.
        params: Параметры смещения центра дуги (I, J, K) в единицах G-code.
        is_cw: Флаг направления обхода (True = по часовой стрелке).
        step_m: Максимальная длина линейного сегмента в метрах.
        state: Состояние интерпретатора (используется плоскость и единицы).

    Yields:
        Последовательность промежуточных 6D-поз вдоль дуги.
        Всегда включает конечную точку `end`.

    Raises:
        ValueError: Если радиус дуги меньше 1e-6 м (делегировано из `_compute_arc_geometry`).
    """
    g = compute_arc_geometry(start, end, params, is_cw, state)
    arc_len = abs(g["delta"]) * g["r"]
    steps = max(1, math.ceil(arc_len / step_m))

    for i in range(1, steps + 1):
        t = i / steps
        if t >= 1.0:
            yield g["end"]
            continue

        theta = g["th_s"] + g["delta"] * t
        u = g["c_u"] + g["r"] * math.cos(theta)
        v = g["c_v"] + g["r"] * math.sin(theta)
        ortho = (
            g["start"][g["ortho_idx"]]
            + (g["end"][g["ortho_idx"]] - g["start"][g["ortho_idx"]]) * t
        )
        ori = tuple(
            s + (e - s) * t for s, e in zip(g["start"][3:], g["end"][3:])
        )

        pose = [0.0] * 6
        pose[g["u_idx"]] = u
        pose[g["v_idx"]] = v
        pose[g["ortho_idx"]] = ortho
        pose[3:6] = ori
        yield tuple(pose)
