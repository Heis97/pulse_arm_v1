"""
Запуск графического интерфейса для управления роботом и цикла перемещения
в отдельном потоке.

Можно запустить в отдельном потоке и графический интерфейс, но этого делать не
рекомендуется, так как запуск графического интерфейса не в главном потоке
программы может привести к возникновению непредвиденных ошибок.
"""

import threading

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def in_thread_movement_logic(robot: RobotApi):
    """
    Логика перемещения робота. Используется для запуска в отдельном потоке.
    """

    while robot.is_connected():
        robot.motion.joint.add_new_waypoint(
            angle_pose=(0, -115, 120, -100, -90, 0),
            speed=70,
            accel=70,
            blend=0,
            units="deg",
        )
        robot.motion.linear.add_new_waypoint(
            tcp_pose=(-0.44, -0.16, 0.337, -175, 0, 90),
            speed=0.5,
            accel=0.5,
            orientation_units="deg",
        )
        robot.motion.mode.set("move")
        robot.motion.wait_waypoint_completion(0)


def start_simple_joystick_with_thread(robot_ip: str):
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
    robot.motion.scale_setup.set(velocity=0.3, acceleration=0.3)

    # Запуск перемещения робота в отдельном потоке
    movement_thread = threading.Thread(
        target=in_thread_movement_logic,
        args=(robot,),
        daemon=True,
        name="RobotMovement-Thread",
    )
    movement_thread.start()

    # Запуск графического интерфейса
    robot.motion.simple_joystick()


if __name__ == "__main__":
    # Запуск определенной выше функции
    start_simple_joystick_with_thread(ROBOT_IP)
