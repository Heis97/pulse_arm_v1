"""
Запуск графического интерфейса для управления роботом с пользовательской
системой координат.
"""

from API import RobotApi
from API.coords import CoordinateSystem

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def start_simple_joystick(robot_ip: str):
    """
    Запустить графический интерфейс 'Simple Joystick' для управления роботом.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Создание пользовательской системы координат
    user_coordinate_system = CoordinateSystem(
        (
            -0.6268585854335833,
            -0.481927454358517,
            0.00739,
            179.9295,
            -0.1177,
            44.1798,
        )
    )

    # Запуск графического интерфейса относительно пользовательской системы координат
    robot.motion.simple_joystick(coordinate_system=user_coordinate_system)


if __name__ == "__main__":
    # Запуск определенной выше функции
    start_simple_joystick(ROBOT_IP)
