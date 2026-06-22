"""
Скрипт для калибровки TCP (Tool Center Point) робота методом 4 точек.

Данный скрипт использует режим "свободный привод" (Free Drive) для удобства
позиционирования робота оператором. Однако, для высокоточной калибровки в
реальном применении этот подход НЕ РЕКОМЕНДУЕТСЯ ввиду большой вероятности
неточной установки робота в позицию.

Для промышленного применения рекомендуется подводить робота к точкам, например,
с помощью режимов 'jog' или графического окна 'Simple joystick'.
"""

import logging
import threading
import time

from API import RobotApi
from API.tools import calibrate_tcp
from API.types import PositionOrientation

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"
# Количество калибровочных точек (должно быть не меньше 4)
POINTS_NUMBER: int = 4


def free_drive(robot: RobotApi):
    """
    Фоновый поток для поддержания режима свободного привода (Free Drive).
    """
    while robot.is_connected():
        robot.motion.free_drive()
        time.sleep(0.01)


def calibrate_robot_tcp_params(robot_ip: str):
    """
    Откалибровать параметры ЦТИ инструмента посредством сбора точек в режиме
    'Свободный привод'.

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

    # Сбрасываем параметры инструмента к нулевым.
    # Это критически важно: координаты, которые мы
    # будем считывать, должны описывать положение
    # ЦЕНТРА ФЛАНЦА, а не ЦТИ старого инструмента.
    robot.tool.set((0, 0, 0, 0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Запуск фонового потока для Free Drive.
    # Используем threading.Event, чтобы иметь возможность
    # остановить поток в будущем, если потребуется
    # (в данном примере не используется, ).
    free_drive_thread = threading.Thread(
        target=free_drive, args=(robot,), daemon=True
    )
    free_drive_thread.start()

    # Небольшая пауза для того, чтобы вывод в терминале
    # не перемешался с логами робота
    time.sleep(0.2)

    # Выводим инструкцию для оператора
    print("\n" + "-" * 65)
    print("Please move the robot tool tip to the calibration reference point.")
    print(
        "Ensure the tool orientation is significantly different for each pose."
    )
    print("  - Pose 1: Standard vertical approach")
    print("  - Pose 2: Tilted ~45 degrees along X-axis")
    print("  - Pose 3: Tilted ~45 degrees along Y-axis")
    print("  - Pose 4: Combined tilt (avoid singularities)")
    print("-" * 65)

    collected_poses: list[PositionOrientation] = []
    for i in range(POINTS_NUMBER):
        print()
        input(f">> Press [ENTER] to record Pose {i + 1}/{POINTS_NUMBER}...")

        # Считываем текущее положение фланца в базовой системе координат
        pose = robot.motion.linear.get_actual_position()
        collected_poses.append(pose)

        # Форматируем вывод для удобочитаемости (позиция | ориентация)
        pos_str = f"[{pose[0]:6.3f}, {pose[1]:6.3f}, {pose[2]:6.3f}]"
        ori_str = f"[{pose[3]:7.2f}, {pose[4]:6.2f}, {pose[5]:7.2f}]"
        print(f"  -> Pose {i + 1} recorded: {pos_str} | {ori_str}")

    print("\nAll required poses collected. Starting optimization...")

    # Запуск алгоритма калибровки.
    # Мы передаем default_tcp_orientation=(0, 0, 0), так как метод 4 точек
    # физически не может рассчитать ориентацию инструмента (Rx, Ry, Rz).
    result = calibrate_tcp(collected_poses, default_tcp_orientation=(0, 0, 0))

    # Подробный вывод результатов
    tcp_pos_mm = [x * 1000 for x in result.tcp[:3]]
    tcp_ori_deg = result.tcp[3:]
    target_mm = [x * 1000 for x in result.target_point[:3]]
    errors_mm = [x * 1000 for x in result.errors]
    rmse_mm = result.rmse * 1000
    max_err_mm = result.max_error * 1000

    print("-" * 65)
    print("  Calculated TCP (Flange -> Tool):")
    print(
        f"    Position (X,Y,Z) : [{tcp_pos_mm[0]:7.2f}, "
        f"{tcp_pos_mm[1]:7.2f}, {tcp_pos_mm[2]:7.2f}] mm"
    )
    print(
        f"    Orient (Rx,Ry,Rz): [{tcp_ori_deg[0]:7.2f}, "
        f"{tcp_ori_deg[1]:7.2f}, {tcp_ori_deg[2]:7.2f}] deg"
    )
    print("-" * 65)
    print("  Target Point (in specified CS):")
    print(
        f"    Position (X,Y,Z) : [{target_mm[0]:7.2f}, "
        f"{target_mm[1]:7.2f}, {target_mm[2]:7.2f}] mm"
    )
    print("-" * 65)
    print("  Accuracy Metrics:")
    print(f"    RMSE             : {rmse_mm:7.2f} mm")
    print(f"    Max Error        : {max_err_mm:7.2f} mm")
    print(
        f"    Errors per point : [{', '.join([f'{e:5.2f}' for e in errors_mm])}] mm"
    )

    response = (
        input("\n>> Apply calibrated TCP parameters to the robot? [y/N]: ")
        .strip()
        .lower()
    )
    if response not in ("y", "yes"):
        print("  -> Calibration parameters were NOT applied.")
        print(f"  -> You can apply them manually: {result.tcp}")
        return

    robot.tool.set(result.tcp, units=result.orientation_units)
    print("  -> TCP parameters applied successfully!")


if __name__ == "__main__":
    # Запуск определенной выше функции
    calibrate_robot_tcp_params(ROBOT_IP)
