from __future__ import annotations

import math
import time
from typing import Any, cast

from api.robot_api_rc5v15.API.source.models.type_aliases import AnalogIndex, DigitalIndex

from .dataclasses import GCodeCommand, GCodeExecutionContext, GCodeState
from .tools import (
    compute_arc_geometry,
    interpolate_arc,
    interpolate_linear,
    pose_at_angle,
)
from .utils import gcode_val_to_meters

_DEFAULT_OFFSETS = {
    1: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # G54
    2: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # G55
    3: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # G56
    4: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # G57
    5: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # G58
    6: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # G59
}


def handle_g0_g1(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G0/G1: Быстрое (G0) и линейное (G1) перемещение.

    Выполняет перемещение к целевой позиции по прямой. При включённой
    интерполяции разбивает отрезок на мелкие сегменты для плавности траектории.

    Args:
        cmd: Команда G0 или G1 с целевыми координатами и опциональной подачей.
        state: Текущее состояние. Обновляет `state.current_pose`, сбрасывает
            `state.is_machine_mode` и запоминает `state.last_motion_cmd`.
        ctx: Контекст выполнения. Управляет буфером очереди, применяет фрейм
            координат и отправляет точки через API робота.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G0 X100 Y0 Z5       ; Быстрое перемещение в точку (100, 0, 5)
        G1 X50 Y50 Z0 F1000 ; Линейное движение с подачей 1000 мм/мин
    """
    target = state.resolve_target(cmd)
    _, cmd_idx = cmd.parse_command()
    is_rapid = cmd_idx == 0
    with ctx.coordinate_system.in_frame():
        if ctx.use_joint_motion_for_g0 and is_rapid:
            ctx.robot.motion.wait_waypoint_completion(ctx.max_buffer_size - 1)
            ctx.robot.motion.advanced.add_movej_tcp_waypoint(
                tcp_pose=target,
                blend=ctx.active_blend,
            )
        elif not ctx.interpolate_linear:
            ctx.robot.motion.wait_waypoint_completion(ctx.max_buffer_size - 1)
            ctx.robot.motion.advanced.add_movel_waypoint(
                tcp_pose=target,
                translation_speed=state.feed_m_s,
                blend=ctx.active_blend,
            )
        else:
            for point in interpolate_linear(
                start=state.current_pose,
                end=target,
                step_m=ctx.interpolation_step,
            ):
                ctx.robot.motion.wait_waypoint_completion(
                    ctx.max_buffer_size - 1
                )
                ctx.robot.motion.advanced.add_movel_waypoint(
                    tcp_pose=point,
                    translation_speed=state.feed_m_s,
                    blend=ctx.active_blend,
                )
        state.current_pose = target
        state.is_machine_mode = False
        state.last_motion_cmd = cmd.command


def handle_g2_g3(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G2/G3: Круговое движение (по/против часовой стрелки).

    Выполняет перемещение по дуге в заданной плоскости. Использует параметры
    I/J/K для задания центра дуги относительно стартовой точки.

    Args:
        cmd: Команда G2 или G3 с целевыми координатами и смещениями центра (I/J/K).
        state: Текущее состояние. Обновляет `state.current_pose`, сбрасывает
            `state.is_machine_mode` и запоминает `state.last_motion_cmd`.
        ctx: Контекст выполнения. Проверяет наличие I/J/K, рассчитывает промежуточную
            точку или интерполирует дугу, управляет буфером очереди.
        custom_ctx: Пользовательский контекст (передаётся в fallback-обработчик).

    Examples:
        G2 X15 Y190.05 I15.05 J0 ; Дуга по часовой стрелке
        G3 X50 Y50 I50 J0 F800   ; Дуга против часовой стрелки с подачей
    """
    target = state.resolve_target(cmd)
    _, cmd_idx = cmd.parse_command()
    is_cw = cmd_idx == 2

    with ctx.coordinate_system.in_frame():
        if any(k in cmd.params for k in ("I", "J", "K")):
            if not ctx.interpolate_arcs:
                geo = compute_arc_geometry(
                    state.current_pose, target, cmd.params, is_cw, state
                )
                delta = geo["delta"]

                max_native_angle = 0.99 * math.pi

                n_segs = math.ceil(abs(delta) / max_native_angle)

                for i in range(n_segs):
                    theta_mid = geo["th_s"] + delta * (i + 0.5) / n_segs
                    theta_end = geo["th_s"] + delta * (i + 1) / n_segs

                    via_pose = pose_at_angle(geo, theta_mid)
                    end_pose = (
                        target
                        if i == n_segs - 1
                        else pose_at_angle(geo, theta_end)
                    )

                    seg_blend = 0.001 if i < n_segs - 1 else ctx.active_blend

                    ctx.robot.motion.wait_waypoint_completion(
                        ctx.max_buffer_size - 2
                    )

                    ctx.robot.motion.advanced.add_movec_waypoint(
                        tcp_pose_1=via_pose,
                        tcp_pose_2=end_pose,
                        translation_speed=state.feed_m_s,
                        blend=seg_blend,
                    )
            else:
                for point in interpolate_arc(
                    start=state.current_pose,
                    end=target,
                    params=cmd.params,
                    is_cw=is_cw,
                    step_m=ctx.interpolation_step,
                    state=state,
                ):
                    ctx.robot.motion.wait_waypoint_completion(
                        ctx.max_buffer_size - 1
                    )
                    ctx.robot.motion.advanced.add_movel_waypoint(
                        tcp_pose=point,
                        translation_speed=state.feed_m_s,
                        blend=ctx.active_blend,
                    )

        else:
            logger = ctx.logger
            if logger:
                logger.warning(
                    f"Arc command {cmd.command} at line {cmd.line_number} missing I/J/K. Fallback to linear."
                )
            handle_g0_g1(cmd, state, ctx, custom_ctx)

        state.current_pose = target
        state.is_machine_mode = False
        state.last_motion_cmd = cmd.command


def handle_g4(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G4: Выдержка (Dwell).

    Приостанавливает выполнение программы на указанное время. Поддерживает
    параметры P, X или U для задания длительности в секундах.

    Args:
        cmd: Команда с параметром `P<сек>`, `X<сек>` или `U<сек>`.
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G4 P0.5           ; Выдержка 0.5 секунды
        G4 X1.0           ; Выдержка 1.0 секунды (альтернативный синтаксис)
    """
    seconds = cmd.params.get(
        "P", cmd.params.get("X", cmd.params.get("U", 0.0))
    )

    if seconds <= 0:
        return

    time.sleep(seconds)


def handle_g17(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G17: Выбор плоскости XY для интерполяции дуг.

    Все последующие команды дуг (G2/G3) будут рассчитываться в плоскости XY.

    Args:
        cmd: Команда G17 (параметры игнорируются).
        state: Текущее состояние. Устанавливает `state.plane = "XY"`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G17               ; Плоскость XY (режим по умолчанию)
        G17 G3 X50 Y50 I50 J0 ; Дуга в плоскости XY
    """
    state.update_modal(cmd)


def handle_g18(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G18: Выбор плоскости XZ для интерполяции дуг.

    Все последующие команды дуг (G2/G3) будут рассчитываться в плоскости XZ.

    Args:
        cmd: Команда G18 (параметры игнорируются).
        state: Текущее состояние. Устанавливает `state.plane = "XZ"`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G18               ; Плоскость XZ
        G18 G2 X50 Z50 I0 K50 ; Дуга в плоскости XZ
    """
    state.update_modal(cmd)


def handle_g19(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G19: Выбор плоскости YZ для интерполяции дуг.

    Все последующие команды дуг (G2/G3) будут рассчитываться в плоскости YZ.

    Args:
        cmd: Команда G19 (параметры игнорируются).
        state: Текущее состояние. Устанавливает `state.plane = "YZ"`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G19               ; Плоскость YZ
        G19 G3 Y50 Z50 J0 K50 ; Дуга в плоскости YZ
    """
    state.update_modal(cmd)


def handle_g20(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G20: Выбор единиц измерения — дюймы.

    Все последующие линейные координаты (X, Y, Z, I, J, K) интерпретируются
    как дюймы до команды G21.

    Args:
        cmd: Команда G20 (параметры игнорируются).
        state: Текущее состояние. Устанавливает `state.units_mm = False`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G20               ; Единицы: дюймы
        G20 G1 X5.0 Y2.0 F20 ; Движение в дюймах
    """
    state.update_modal(cmd)


def handle_g21(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G21: Выбор единиц измерения — миллиметры.

    Все последующие линейные координаты интерпретируются как миллиметры.

    Args:
        cmd: Команда G21 (параметры игнорируются).
        state: Текущее состояние. Устанавливает `state.units_mm = True`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G21               ; Единицы: миллиметры (режим по умолчанию)
        G21 G1 X100 Y50 F500 ; Движение в миллиметрах
    """
    state.update_modal(cmd)


def handle_g43_1(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G43.1: Прямое задание смещения инструмента (TCP offset).

    Применяет указанные смещения по осям X, Y, Z, A, B, C к текущему
    положению инструмента. Линейные оси автоматически конвертируются в метры.

    Args:
        cmd: Команда с параметрами осей, например `G43.1 Z15.0 A90`.
        state: Текущее состояние (не изменяется напрямую).
        ctx: Контекст выполнения. Применяет смещение через `ctx.robot.tool.set`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G43.1 Z15.0       ; Смещение TCP на 15 мм по оси Z
        G43.1 X5 Y0 Z10   ; Комплексное смещение TCP по XYZ
    """
    offsets = [
        cmd.params.get(axis, 0.0) for axis in ("X", "Y", "Z", "A", "B", "C")
    ]
    for i in range(3):
        offsets[i] = gcode_val_to_meters(offsets[i], state.units_mm)

    ctx.robot.tool.set(offsets, units="deg")


def handle_g49(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G49: Отмена смещения инструмента.

    Сбрасывает активное смещение TCP (G43.1) к нулю по всем осям.

    Args:
        cmd: Команда G49 (параметры игнорируются).
        state: Текущее состояние (не изменяется напрямую).
        ctx: Контекст выполнения. Сбрасывает смещение через `ctx.robot.tool.set`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G49               ; Сбросить смещение TCP
    """
    ctx.robot.tool.set([0.0] * 6)


def handle_g53(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G53: Машинная система координат.

    Переключает интерпретацию координат на абсолютные машинные координаты (относительно
    нуля станка), игнорируя рабочие смещения G54–G59. Действует только на одну команду
    движения в текущей строке.

    Args:
        cmd: Команда G53 (параметры игнорируются).
        state: Текущее состояние. Устанавливает `state.is_machine_mode = True`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G53 G0 Z0         ; Быстрый подъём в машинный ноль по Z
        G53 G1 X100 Y50 F2000 ; Движение в абсолютные машинные координаты
    """
    state.is_machine_mode = True


def handle_g54(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G54: Активация рабочего смещения #1.

    Загружает предустановленное смещение рабочей системы координат. Поддерживает
    параметр `P` для выбора индекса смещения (G54 P1..P48 в расширенных системах).

    Args:
        cmd: Команда с опциональным параметром `P<индекс>`, например `G54 P2`.
        state: Текущее состояние. Обновляет `state.work_offset`, `state.active_work_offset`,
            и сбрасывает `state.is_machine_mode = False`.
        ctx: Контекст выполнения. Используется `ctx.offset_table` для поиска смещений.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G54               ; Активировать смещение #1 по умолчанию
        G54 P3            ; Активировать смещение #3 (например, для второй детали)
    """
    idx = int(cmd.params.get("P", 1))
    state.work_offset = ctx.offset_table.get(
        idx, _DEFAULT_OFFSETS.get(idx, [0.0] * 6)
    )
    state.active_work_offset = idx
    state.is_machine_mode = False


def handle_g61(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G61: Режим точной остановки (Exact Stop).

    Отключает сглаживание траектории: робот полностью останавливается в каждой
    точке перед началом следующего движения. Используется для операций, требующих
    высокой позиционной точности.

    Args:
        cmd: Команда G61 (параметры игнорируются).
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения. Устанавливает `ctx.active_blend = 0.0`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G61 G1 X50 Y0 F500  ; Точное позиционирование в точку (50, 0)
        G61                 ; Переключение в режим точных остановок
    """
    ctx.active_blend = 0.0


def handle_g64(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G64: Режим непрерывного пути (Continuous Path).

    Включает сглаживание траектории: робот плавно проходит через точки без полной
    остановки. Опциональный параметр `P` задаёт допуск сглаживания в мм.

    Args:
        cmd: Команда с опциональным параметром `P<допуск>`, например `G64 P0.5`.
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения. Обновляет `ctx.active_blend` из параметра `P`
            или использует значение по умолчанию `ctx.blend`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G64               ; Включить сглаживание с допуском по умолчанию
        G64 P0.1          ; Включить сглаживание с допуском 0.1 мм
    """
    ctx.active_blend = cmd.params.get("P", ctx.blend)


def handle_g90(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G90: Абсолютный режим позиционирования.

    Все последующие координаты интерпретируются относительно начала рабочей системы
    координат (нуля детали, заданного через G54–G59).

    Args:
        cmd: Команда G90 (параметры игнорируются).
        state: Текущее состояние интерпретатора. Устанавливает `state.absolute_mode = True`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G90 X100 Y50      ; Перемещение в абсолютную точку (100, 50) мм
        G90 G1 Z-10 F500  ; Линейное движение в абсолютную координату Z=-10 мм
    """
    state.update_modal(cmd)


def handle_g91(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """G91: Инкрементальный (относительный) режим позиционирования.

    Все последующие координаты интерпретируются как смещение относительно текущей
    позиции робота.

    Args:
        cmd: Команда G91 (параметры игнорируются).
        state: Текущее состояние интерпретатора. Устанавливает `state.absolute_mode = False`.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        G91 X10 Y-5       ; Смещение на +10 мм по X и -5 мм по Y от текущей позиции
        G91 G1 Z2 F300    ; Подъём на 2 мм относительно текущего Z
    """
    state.update_modal(cmd)


def handle_f(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """F: Установка скорости подачи (feed rate).

    Задаёт скорость перемещения инструмента в мм/мин (при активном G94). Значение
    сохраняется в состоянии и применяется ко всем последующим движениям до изменения.

    Args:
        cmd: Команда с параметром `F<значение>`, например `F1500.0`.
        state: Текущее состояние. Обновляет `state.feed_m_s` (конвертирует мм/мин -> м/с).
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        F1200             ; Установить подачу 1200 мм/мин
        G1 X50 Y0 F500    ; Движение с новой подачей 500 мм/мин
    """
    state.update_modal(cmd)


def handle_m2(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """M2: Конец программы.

    Завершает выполнение программы без сброса модальных параметров.

    Args:
        cmd: Команда M2 (параметры игнорируются).
        state: Текущее состояние интерпретатора (не используется).
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).
    """
    pass


def handle_m30(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """M30: Конец программы и сброс состояния.

    Завершает выполнение программы и сбрасывает все модальные параметры к значениям
    по умолчанию для безопасного запуска следующей программы.

    Args:
        cmd: Команда M30 (параметры игнорируются).
        state: Текущее состояние интерпретатора. Сбрасывает режимы позиционирования,
            единицы измерения, плоскость, подачу, смещения и последнюю команду движения.
        ctx: Контекст выполнения (не используется).
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        M30               ; Завершение программы, возврат к дефолтным настройкам
        %                 ; Конец файла (обычно следует после M30)
    """
    state.absolute_mode = True
    state.units_mm = True
    state.plane = "XY"
    state.feed_m_s = 0.01
    state.is_machine_mode = False
    state.work_offset = [0.0] * 6
    state.last_motion_cmd = "G0"


def handle_m62(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """M62 P<n>: Включить цифровой выход.

    Активирует указанный цифровой выход контроллера (устанавливает высокий уровень).

    Args:
        cmd: Команда с параметром `P<номер>`, например `M62 P3`.
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения. Используется `ctx.robot`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        M62 P0            ; Включить выход #0
        M62 P3            ; Включить выход #3 (например, сигнал "готов к приёму")
    """
    index = int(cmd.params.get("P", 0))
    ctx.robot.io.digital.set_output(cast(DigitalIndex, index), True)


def handle_m63(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """M63 P<n>: Выключить цифровой выход.

    Деактивирует указанный цифровой выход контроллера (устанавливает низкий уровень).

    Args:
        cmd: Команда с параметром `P<номер>`, например `M63 P3`.
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения. Используется `ctx.robot`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        M63 P0            ; Выключить выход #0
        M63 P3            ; Выключить выход #3 (например, сброс сигнала "готов")
    """
    index = int(cmd.params.get("P", 0))
    ctx.robot.io.digital.set_output(cast(DigitalIndex, index), False)


def handle_m67(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """M67 E<n> Q<val>: Установка аналогового выхода.

    Задаёт напряжение/ток на указанном аналоговом канале. Единицы измерения берутся
    из `ctx.analog_outputs_units`.

    Args:
        cmd: Команда с параметрами `E<канал>` и `Q<значение>`, например `M67 E2 Q4.5`.
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения. Используется `ctx.robot`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        M67 E0 Q10.0      ; Установить 10 В на канале 0
        M67 E1 Q4.5       ; Установить 4.5 мА на канале 1
    """
    index = int(cmd.params.get("E", 0))
    value = float(cmd.params.get("Q", 0.0))
    ctx.robot.io.analog.set_output(
        cast(AnalogIndex, index), value, ctx.analog_outputs_units
    )


def handle_m66(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: Any,
):
    """M66 P<n> L<type> Q<timeout>: Ожидание цифрового входа.

    Блокирует выполнение до изменения состояния указанного входа.

    Args:
        cmd: Команда с параметрами `P<порт>`, `L<тип>`, `Q<таймаут>`.
        state: Текущее состояние интерпретатора (не изменяется).
        ctx: Контекст выполнения. Используется `ctx.robot`.
        custom_ctx: Пользовательский контекст (не используется).

    Examples:
        M66 P0 L2 Q5.0    ; Ждать высокий уровень на входе 0, таймаут 5 сек
        M66 P1 L0 Q10.0   ; Ждать фронт на входе 1, таймаут 10 сек
    """
    index = int(cmd.params.get("P", 0))
    wait_type = int(
        cmd.params.get("L", 0)
    )  # 0=rising, 1=falling, 2=high, 3=low

    high = wait_type == 0 or wait_type == 2
    timeout = cmd.params.get("Q", 50.0)

    ctx.robot.io.digital.wait_input(
        cast(DigitalIndex, index), value=high, await_sec=timeout
    )
