"""
Выполнение цикла с возможностью начать его сначала при нажатии кнопки на запястье.
"""

import time

from API import RobotApi
from API.types import DigitalWristIndex

# IPv4 адрес целевого робота
ROBOT_IP: str = "192.168.0.163"


def wait_cycle_completion(
    robot: RobotApi, input_index: DigitalWristIndex
) -> bool:
    """
    Ожидание завершения движения с проверкой нажатия указанной кнопки запястья.
    """

    while not robot.motion.check_waypoint_completion():
        # Если кнопка нажата
        if robot.wrist.digital.get_input(index=input_index):
            # Останавливаем движение
            robot.motion.mode.set("hold")
            break
        time.sleep(0.001)
    # Ждем отпускание кнопки
    while robot.wrist.digital.get_input(index=input_index):
        time.sleep(0.001)
    return True


def reset_cycle_by_input(robot_ip: str):
    """
    Переместить робота в положение 'семерка'.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))
    robot.tool.set(tool_end_point=(0, 0, 0, 0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=0.2, acceleration=0.2)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Определение одного цикла движения
    while robot.is_connected():
        robot.motion.joint.add_new_waypoint(
            angle_pose=(0, -115, -80, -100, -90, 0),
            speed=70,
            accel=70,
            units="deg",
        )
        robot.motion.joint.add_new_waypoint(
            angle_pose=(-180, -100, -50, -90, 90, -80),
            speed=90,
            accel=90,
        )
        robot.motion.joint.add_new_waypoint(
            angle_pose=((-100, -140, -40, -60, 90, 0)), speed=90, accel=90
        )

        # Запуск движения и ожидание его завершения или нажатия кнопки с
        # указанным индексом на плате запястья
        robot.motion.mode.set("move")
        if wait_cycle_completion(robot, input_index=1):
            continue


if __name__ == "__main__":
    # Запуск определенной выше функции
    reset_cycle_by_input(ROBOT_IP)
