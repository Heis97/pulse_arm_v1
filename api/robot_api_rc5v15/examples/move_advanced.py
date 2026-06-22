"""
Перемещение робота с использованием типа движения 'advanced'.
Перемещение по точкам в бесконечном цикле.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def move_advanced(robot_ip: str):
    """
    Переместить робота с использованием типа движения 'advanced'.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=0.6, acceleration=0.6)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Перемещение в рабочее положение
    robot.motion.advanced.add_movej_waypoint((0, -115, 120, -100, -90, 0))
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)

    while robot.is_connected():
        # Добавление точки типа 'linear'
        robot.motion.advanced.add_movel_waypoint(
            (-0.20, -0.35, 0.35, -3.05, 0, 1.57),
            blend=0.05,
            orientation_units="rad",
        )
        # Добавление точки типа 'process'
        robot.motion.advanced.add_movep_waypoint(
            (-0.45, -0.1, 0.2, -135, 15, 45),
            translation_speed=0.1,
            rotation_speed=30,
        )
        # Добавление точки типа 'circular'
        robot.motion.advanced.add_movec_waypoint(
            (-0.35, -0.20, 0.4, -160, 0, 80),
            (-0.25, -0.30, 0.4, -150, 0, 60),
            translation_speed=0.5,
            translation_accel=5,
            blend=0.05,
        )

        # Запуск движения и ожидание его завершения
        robot.motion.mode.set("move_adv")
        robot.motion.wait_waypoint_completion(0)


if __name__ == "__main__":
    # Запуск определенной выше функции
    move_advanced(ROBOT_IP)
