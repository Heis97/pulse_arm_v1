"""
Постановка робота на сервоудержание и на штатные тормоза.
При перемещении в точку робот автоматически активирует сервоудержание, для
активации штатных тормозов необходимо изменить состояние контроллера на 'on'.
"""

import time

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def brakes_activation(robot_ip: str):
    """
    Переключить способ удержания робота.

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

    # Запуск робота без деактивации тормозов
    robot.controller.state.set("on", await_sec=120)
    print("Робот удерживается с помощью штатных тормозов")
    time.sleep(5)

    # Деактивация тормозов
    robot.controller.state.set("run", await_sec=120)
    print("Робот удерживается с помощью сервоудержания")
    time.sleep(5)

    # Активация тормозов
    robot.controller.state.set("on", await_sec=120)
    print("Робот удерживается с помощью штатных тормозов")
    time.sleep(5)


if __name__ == "__main__":
    # Запуск определенной выше функции
    brakes_activation(ROBOT_IP)
