"""
Пример создания и регистрации пользовательского обработчика G-кода.

Демонстрирует полный цикл интеграции пользовательской команды (аналог G28) в
интерпретатор `GCodeExecutor`: от определения контекста данных до запуска
программы на роботе.

Ключевые концепции:
    - Наследование сигнатуры обработчика: `(cmd, state, ctx, custom_ctx) -> None`.
    - Использование `GCodeCustomContext` (dataclass) для типизированной передачи
      пользовательских параметров (безопасная поза, скорости) в обработчики.
    - Регистрация команды через `executor.register()` с настройкой синхронизации:
      `wait_waypoint_completion_before`, `pause_before`, `start_motion_after`.
    - Управление движением робота через `RobotApi` внутри обработчика
      (добавление точки `MoveJ` в очередь контроллера).
    - Запуск интерпретатора `executor.run()` с передачей `custom_context`
      и активной системы координат `CoordinateSystem`.
"""

from dataclasses import dataclass

from API import RobotApi
from API.coords import CoordinateSystem
from API.tools import GCodeExecutor
from API.types import (
    GCodeCommand,
    GCodeExecutionContext,
    GCodeState,
    PositionOrientation,
)

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"

# Для удобства G код представлен в виде строки, может читаться из файла
G_CODE = """
; Настройки
G90          ; Абсолютные координаты
G21          ; Единицы измерения: миллиметры
F1000        ; Скорость подачи (мм/мин)

; Подготовка
G28          ; Вернуться в безопасную позицию
G0 X0 Y0     ; Переместиться в ноль
G1 Z0        ; Поднять инструмент
G1 X0 Y50    ; Переместить к начальной точке (верх звезды)

; Рисование контура пятиконечной звезды
G1 Z10        ; Опустить инструмент (начало рисования)
G1 X11.76 Y16.18
G1 X47.55 Y15.45
G1 X19.02 Y-6.18
G1 X29.39 Y-40.45
G1 X0 Y-20
G1 X-29.39 Y-40.45
G1 X-19.02 Y-6.18
G1 X-47.55 Y15.45
G1 X-11.76 Y16.18
G1 X0 Y50    ; Замкнуть контур в начальной точке

; Завершение
G1 Z0        ; Поднять инструмент
G0 X0 Y0     ; Вернуться в ноль
G28          ; Вернуться в безопасную позицию
M30          ; Конец программы
"""


@dataclass
class GCodeCustomContext:
    """
    Класс для хранения параметров пользовательского (кастомного) контекста.
    Может быть любой сущностью, при передаче в качестве параметра в метод
    GCodeExecutor.run будет доступен в качестве 4 аргумента во всех
    обработчиках.
    """

    safe_joint_pose: PositionOrientation = (-180, -120, 120, -90, -90, 0)
    safe_joint_pose_velocity: float = 30  # deg/s


def handle_g28(
    cmd: GCodeCommand,
    state: GCodeState,
    ctx: GCodeExecutionContext,
    custom_ctx: GCodeCustomContext,
):
    """Обработчик команды возврата в безопасную позицию (аналог стандартного G28).

    Выполняет плавный отвод манипулятора робота в предустановленную конфигурацию
    суставов (`safe_joint_pose`), заданную в пользовательском контексте.
    Используется для безопасной остановки между операциями или в аварийных
    сценариях. Движение выполняется в режиме совместного перемещения
    всех осей (MoveJ) с заданной скоростью.

    Аргументы:
        cmd (GCodeCommand): Команда G-кода, содержащая исходную
            строку, идентификатор команды (например, "G28") и параметры.
            В данном обработчике не используется, но требуется для совместимости
            с сигнатурой `GCodeHandlerType`.
        state (GCodeState): Текущее модальное состояние интерпретатора
            (активные плоскости, режимы единиц, последние команды движения).
            В данном обработчике не используется.
        ctx (GCodeExecutionContext): Контекст выполнения, содержащий экземпляр
            `RobotApi` для отправки команд, активную систему координат, параметры
            буферизации и логгер. Используется для доступа к `ctx.robot.motion`
            и `ctx.logger`.
        custom_ctx (GCodeCustomContext): Пользовательский контекст, переданный
            в `GCodeExecutor.run()`. Содержит целевую позу `safe_joint_pose`
            и скорость `safe_joint_pose_velocity` для выполнения отвода.

    Возвращает:
        None. Функция выполняет побочный эффект — добавляет точку в очередь
        движений контроллера робота.

    Examples:
        >>> # Регистрация обработчика
        >>> executor.register("G28", handle_g28, start_motion_after=True)
        ...
        >>> # Подготовка пользовательского контекста
        >>> ctx = GCodeCustomContext()
        >>> ctx.safe_joint_pose = (0, -30, 60, 0, 90, 0)
        >>> ctx.safe_joint_pose_velocity = 50.0
        ...
        >>> # Выполнение программы с кастомным контекстом
        >>> executor.run("program.nc", custom_context=ctx)
        ...
        >>> # В файле program.nc команда:
        >>> # G28 ; Робот плавно перейдёт в позу (0, -30, 60, 0, 90, 0)
        >>> #       со скоростью 50 град/с
    """
    logger = ctx.logger
    if logger:
        logger.info("Moving to safe pose")
    ctx.robot.motion.advanced.add_movej_waypoint(
        custom_ctx.safe_joint_pose,
        joints_speed=custom_ctx.safe_joint_pose_velocity,
    )


def draw_five_pointed_star(robot_ip: str):
    """
    Нарисовать пятиконечную звезду.

    Args:
        robot_ip: IPv4 адрес робота.
    """

    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров инструмента
    robot.tool.set((0, 0, 0.1, 0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Система координат, в которой будет рисоваться звезда
    # Команда 'G0 X0 Y0' при 'use_joint_motion_for_g0=True' переместит робота
    # в положение, соответствующее центру заданной системы координат
    # перемещением по углам сочленений (нелинейно)
    cs = CoordinateSystem((0.5, 0.5, 0.2, -180, 0, 0))

    # Создание экземпляра класса-исполнителя G кода и его настройка
    executor = GCodeExecutor(
        robot=robot,
        use_joint_motion_for_g0=True,  # G0 интерпретируется как movej_tcp
        max_buffer_size=15,
        min_buffer_size=5,
    )

    # Регистрируем обработчик для команды 'G28'. Указываем, что необходимо
    # дождаться выполнения всех точек до обработки команды, подождать 5 секунд
    # после этого до обработки команды, после чего обработать команду и
    # запустить движение после ее обработки (также движение можно запустить
    # и дождаться его завершения явно в обработчике, если это необходимо)
    executor.register(
        "G28",
        handler=handle_g28,
        wait_waypoint_completion_before=True,
        start_motion_after=True,
        pause_before=5,
    )

    # Создание объекта кастомного контекста
    custom_context = GCodeCustomContext()

    # Запуск выполнения G кода с передачей кастомного контекста
    # Предварительного перевода робота в стартовую позицию не делается так как
    # ранее был определен обработчик для команды G28, который это делает, и эта
    # команда вызывается в начале G кода
    executor.run(
        G_CODE.split("\n"),  # Подача команд по строкам
        coordinate_system=cs,
        custom_context=custom_context,
    )


if __name__ == "__main__":
    # Запуск определенной выше функции
    draw_five_pointed_star(ROBOT_IP)
