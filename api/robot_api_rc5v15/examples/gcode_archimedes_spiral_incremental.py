"""
Пример генерации и отрисовки спирали Архимеда с использованием потокового G-кода.

Демонстрирует создание математической траектории через генератор-функцию,
построчно выдающую команды управления, и их выполнение на роботе через
`GCodeExecutor`. Траектория задается в инкрементальном режиме (G91).

Ключевые концепции:
    - Потоковая генерация траектории: функция-генератор `yield` строки G-кода
      по мере необходимости, что позволяет строить траектории произвольной длины
      без загрузки всего файла в память. Идеально для длинных спиралей,
      сложных кривых или передачи по последовательному порту.
    - Инкрементальный режим (G91): все перемещения задаются как относительные
      приращения (dx, dy), а не абсолютные координаты. Упрощает математическую
      генерацию кривых и снижает накопление ошибок округления.
    - Математическое моделирование: спираль Архимеда строится по закону
      `r(θ) = r_start + k·θ`, где `k = (r_max - r_start) / (2π·coils)`.
      Координаты вычисляются через `x = r·cos(θ)`, `y = r·sin(θ)`.
    - Работа с вертикальной плоскостью: система координат `CoordinateSystem`
      с ориентацией `(-90, 0, -45)` разворачивает траекторию в вертикальную
      плоскость, позволяя рисовать на стене или вертикальной поверхности.
    - Выбор решения обратной кинематики: `ik_solution_id=4` фиксирует конкретную
      конфигурацию суставов для воспроизводимости стартовой позы.

Функции:
    generate_archimedes_spiral_incremental(...):
        Генератор, возвращающий строки G-кода для спирали Архимеда.
        Поддерживает настройку начального/конечного радиуса, количества витков,
        скоростей подачи и глубины по оси Z.
    draw_archimedes_spiral(robot_ip: str):
        Основная функция примера: подключается к роботу, настраивает параметры,
        инициализирует систему координат и запускает выполнение траектории.
"""

import math
from collections.abc import Generator

from API import RobotApi
from API.coords import CoordinateSystem
from API.tools import GCodeExecutor

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def generate_archimedes_spiral_incremental(
    radius_start=5.0,
    radius_max=200.0,
    coils=3,
    feedrate=5000,
    feedrate_z=300,
    z_depth=10,
) -> Generator[str, None, None]:
    """
    Генератор G-кода для создания спирали Архимеда в инкрементальном режиме (G91).

    Функция построчно отдаёт команды управления через `yield`, что позволяет
    генерировать траектории произвольной длины без загрузки всего G-кода в
    оперативную память. Идеально подходит для потоковой передачи по Serial/USB.

    Args:
        radius_start (float): Начальный радиус спирали (мм).
            Робот будет позиционирован в точку `(radius_start, 0)` перед началом рисования.
        radius_max (float): Конечный радиус спирали после завершения всех витков (мм).
        coils (int): Количество полных витков. Чем больше витков, тем плотнее спираль.
        feedrate (float): Скорость подачи в плоскости XY (мм/мин).
        feedrate_z (float): Скорость перемещения по оси Z (мм/мин).
        z_depth (float): Координата Z для рабочего хода (мм).

    Yields:
        Generator[str, None, None]: Итератор, возвращающий строки G-кода.
            Каждая строка уже содержит символ перевода строки `\\n`.

    Example:
        >>> # Сохранение в файл
        >>> with open("spiral.gcode", "w", encoding="utf-8") as f:
        ...     for line in generate_archimedes_spiral_incremental(coils=2):
        ...         f.write(line)

        >>> # Потоковая передача (псевдокод)
        >>> for cmd in generate_archimedes_spiral_incremental(radius_max=50):
        ...     serial_port.write(cmd.encode())

        >>> # Пример генерируемого G кода
        ... ; Archimedes Spiral G-Code Generator (Incremental / G91)
        ... G21 ; Millimeters
        ... G90 ; Absolute positioning for setup
        ... G17 ; XY plane
        ... G94 ; Feed rate mm/min
        ... G0 Z5.0 ; Retract Z
        ...
        ... G0 X5.000 Y0.0
        ... G1 Z0.000 F300.000
        ... F1000
        ...
        ... G91 ; Relative positioning (increments)
        ...
        ... G1 X+0.0895 Y+0.0888
        ... G1 X+0.0879 Y+0.0920
        ... G1 X+0.0862 Y+0.0951
        ... G1 X+0.0844 Y+0.0981
        ... G1 X+0.0826 Y+0.1011
        ... G1 X+0.0807 Y+0.1041
        ... G1 X+0.0787 Y+0.1071
        ... # ...

    Notes:
        - Инкрементальный режим: Все движения генерируются в режиме G91
            (относительные смещения).
        - Разрешение траектории: `points_per_rev = 120` -> шаг 3° на виток.
        - Завершение программы: Используется `M30`.
        - Математика: Спираль строится по закону `r(θ) = r_start + (Δr / N) * θ`,
            где `Δr = radius_max - radius_start`, `N = 360 * coils`.
    """
    points_per_rev = 120
    total_points = int(points_per_rev * coils)
    radius_delta = (radius_max - radius_start) / total_points

    yield "; Archimedes Spiral G-Code Generator (Incremental / G91)\n"
    yield "G21 ; Millimeters\n"
    yield "G90 ; Absolute positioning for setup\n"
    yield "G17 ; XY plane\n"
    yield f"G0 Z0 F{feedrate}\n\n"

    # 1. Позиционируемся в начальную точку
    yield f"G0 X{radius_start:.3f} Y0.0\n"
    yield f"G1 Z{z_depth:.3f} F{feedrate_z:.3f}\n"
    yield f"F{feedrate}\n\n"

    # 2. Переключаемся в режим приращений
    yield "G91 ; Relative positioning (increments)\n\n"

    prev_x = radius_start
    prev_y = 0.0

    for i in range(1, total_points + 1):
        theta = (i / points_per_rev) * 2 * math.pi
        current_r = radius_start + (radius_delta * i)
        x = current_r * math.cos(theta)
        y = current_r * math.sin(theta)

        # Вычисляем приращения относительно предыдущей точки
        dx = x - prev_x
        dy = y - prev_y

        yield f"G1 X{dx:+.4f} Y{dy:+.4f}\n"
        prev_x = x
        prev_y = y

    # 3. Возвращаем абсолютный режим
    yield "\nG90 ; Back to absolute\n"
    yield "G0 Z0 ; Retract Z\n"
    yield "G0 X0 Y0 ; Return to WCS origin\n"
    yield "M30 ; End\n"


def draw_archimedes_spiral(robot_ip: str):
    """
    Нарисовать спираль Архимеда в вертикальной плоскости.

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

    # Система координат, в которой будет рисоваться логотип
    # Команда 'G0 X0 Y0' переместит робота в положение, соответствующее
    # центру заданной системы координат
    cs = CoordinateSystem((0.4, 0.4, 0.6, -90, 0, -45))

    # Подготовка робота - перевод в стартовую позицию. Рекомендуется явно
    # переводить робота в стартовую позицию (не через G0/G1 G кода).
    # В данном случае производится перевод робота в начало заданной системы
    # координат в заданной конфигурации (ik_solution_id=1)
    robot.motion.advanced.add_movej_tcp_waypoint(
        tcp_pose=(0, 0, 0, 0, 0, 0),
        ik_solution_id=4,
        orientation_units="deg",
        coordinate_system=cs,
    )
    robot.motion.mode.set("move_adv")
    robot.motion.wait_waypoint_completion()

    # Создание экземпляра класса-исполнителя G кода и его настройка
    executor = GCodeExecutor(
        robot=robot,
        use_joint_motion_for_g0=False,
        max_buffer_size=30,
        min_buffer_size=20,
    )

    # Запуск выполнения G кода
    executor.run(
        generate_archimedes_spiral_incremental(
            radius_start=5,  # mm
            radius_max=200,  # mm
            coils=5,
            feedrate=3000,  # mm/min
            feedrate_z=500,  # mm/min
            z_depth=10,  # mm
        ),
        coordinate_system=cs,
    )


if __name__ == "__main__":
    # Запуск определенной выше функции
    draw_archimedes_spiral(ROBOT_IP)
