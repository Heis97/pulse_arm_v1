"""
Перемещение робота с использованием типа движения 'advanced' по кругу.
Перемещение по точкам в бесконечном цикле.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def move_advanced_circular(robot_ip: str):
    """
    Нарисовать круг с помощью типа движения 'advanced'.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=0.5, acceleration=0.5)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Перемещение в рабочее положение
    robot.motion.joint.add_new_waypoint((0, -115, 120, -100, -90, 0))
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)

    # Добавление первой точки дуги
    robot.motion.advanced.add_movel_waypoint((-0.2, -0.3, 0.3, -175, 0, 90))

    while robot.is_connected():
        # Добавление остальных точек окружности
        robot.motion.advanced.add_movec_waypoint(
            (-0.3, -0.4, 0.3, -175, 0, 90),
            (-0.4, -0.3, 0.3, -175, 0, 90),
            translation_speed=0.2,
            translation_accel=5,
            blend=0.01,
        )
        robot.motion.advanced.add_movec_waypoint(
            (-0.3, -0.2, 0.3, -175, 0, 90),
            (-0.2, -0.3, 0.3, -175, 0, 90),
            translation_speed=0.2,
            translation_accel=5,
            blend=0.01,
        )

        # Запуск движения и ожидание выполнения всех точек, кроме последней
        robot.motion.mode.set("move_adv")
        robot.motion.wait_waypoint_completion(1)


if __name__ == "__main__":
    # Запуск определенной выше функции
    move_advanced_circular(ROBOT_IP)
