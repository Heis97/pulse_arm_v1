"""
Пример отрисовки концентрических окружностей с мониторингом скорости робота.

Демонстрирует выполнение циклической траектории (дуги G2/G3, линейные перемещения)
с одновременным выводом текущей скорости робота в реальном времени через
`PeriodicPublisher`.

Ключевые концепции:
    - Использование `PeriodicPublisher` для периодического опроса состояния робота
      (`get_actual_velocity`) и вызова пользовательского колбэка без блокировки
      основного потока выполнения.
    - Работа с пользовательской системой координат (`CoordinateSystem`): все
      координаты в G-коде интерпретируются относительно заданного фрейма.
"""

import logging
import math

from API import RobotApi
from API.coords import CoordinateSystem
from API.tools import GCodeExecutor, PeriodicPublisher
from API.types import PositionOrientation

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"

# Для удобства G код представлен в виде строки, может читаться из файла
G_CODE = """
; Настройки
G90          ; Абсолютные координаты
G21          ; Миллиметры
G0 X0 Y0 Z0  ; Начальная позиция
F5000        ; Подача для перемещений

; Подготовка
G1 Z0        ; Убедиться, что инструмент в безопасной зоне
G1 X0 Y200   ; Переход к стартовой точке внешнего круга

; Рисование внешней окружности (R=200)
G1 Z10 F3000            ; Опускание на рабочую глубину
G3 X0 Y200 I0 J-200     ; Полный круг против часовой стрелки
G1 Z0 F5000             ; Подъём

; Рисование средней окружности (R=150)
G1 X0 Y150
G1 Z10 F3000
G2 X0 Y150 I0 J-150     ; Полный круг по часовой стрелке
G1 Z0 F5000

; Рисование внутренней окружности (R=100)
G1 X0 Y100
G1 Z10 F3000
G3 X0 Y100 I0 J-100     ; Полный круг против часовой стрелки
G1 Z0 F5000

; Рисование центральной окружности (R=50)
G1 X0 Y50
G1 Z10 F3000
G2 X0 Y50 I0 J-50       ; Полный круг по часовой стрелке
G1 Z0 F5000

; Рисование крестовины
G1 X-200 Y0 Z0
G1 Z10 F3000
G1 X200 Y0 F5000
G1 Z0 F5000
G1 X0 Y-200 Z0
G1 Z10 F3000
G1 X0 Y200 F5000
G1 Z0 F5000

; Завершение
G0 X0 Y0 Z0
M30          ; Конец программы
"""


def print_robot_velocity(velocity: PositionOrientation):
    vx, vy, vz, vrx, vry, vrz = velocity
    print(
        f"Current absolute linear robot velocity is "
        f"{math.sqrt(vx**2 + vy**2 + vz**2):.3f} m/s"
    )


def draw_concentric_circles(robot_ip: str):
    """
    Нарисовать концентрические окружности ("мишень").

    Args:
        robot_ip: IPv4 адрес робота.
    """

    # Подключение к роботу
    robot = RobotApi(
        ip=robot_ip,
        show_std_traceback=True,
        autoconnect=True,
        log_std_level=logging.INFO,
    )
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров инструмента
    robot.tool.set((0, 0, 0.1, 0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Подготовка робота - перевод в стартовую позицию. Рекомендуется явно
    # переводить робота в стартовую позицию (не через G0/G1 G кода)
    robot.motion.advanced.add_movej_waypoint((-145, -120, 120, -90, -90, 0))
    robot.motion.mode.set("move_adv")
    robot.motion.wait_waypoint_completion()

    # Система координат, в которой будет рисоваться мишень
    # Команда 'G0 X0 Y0' переместит робота в положение, соответствующее
    # центру заданной системы координат
    cs = CoordinateSystem((0.4, 0.4, 0.2, -180, 0, 0))

    # Создание объекта для вызова метода robot.motion.get_actual_velocity
    # и передачи его результата в метод print_robot_velocity каждые 0.1 секунды
    # для вывода текущей скорости робота в выбранной системе координат
    publisher = PeriodicPublisher(
        func=robot.motion.get_actual_velocity,
        callback=print_robot_velocity,
        interval=0.1,
        orientation_units="deg",  # параметр функции get_actual_velocity
        position_format="tcp",  # параметр функции get_actual_velocity
        coordinate_system=cs,  # параметр функции get_actual_velocity
    )
    # Запуск процесса вывода скорости робота
    publisher.start()

    # Создание экземпляра класса-исполнителя G кода и его настройка
    executor = GCodeExecutor(
        robot=robot,
        max_buffer_size=5,
        min_buffer_size=2,
        interpolate_linear=False,
        interpolate_arcs=False,
        log_std_level=logging.INFO,
    )
    # Запуск выполнения G кода
    executor.run(
        G_CODE.split("\n"),  # Подача команд по строкам
        coordinate_system=cs,
    )


if __name__ == "__main__":
    # Запуск определенной выше функции
    draw_concentric_circles(ROBOT_IP)
