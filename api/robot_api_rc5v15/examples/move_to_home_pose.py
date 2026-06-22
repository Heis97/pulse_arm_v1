"""
Перевод робота в домашнее положение.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def move_to_home_pose(robot_ip: str):
    """
    Переместить робота в домашнее положение.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Вывод параметров домашней позиции
    print("Домашняя позиция робота: ", robot.motion.get_home_pose(units="deg"))

    # Запуск перемещения в домашнее положение
    robot.motion.move_to_home_pose()

    # Ожидание завершения движения
    robot.motion.wait_waypoint_completion(0)


if __name__ == "__main__":
    # Запуск определенной выше функции
    move_to_home_pose(ROBOT_IP)
