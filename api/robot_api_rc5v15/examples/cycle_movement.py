"""
Бесконечное выполнение цикла перемещения между несколькими точками.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def cycle_movement(robot_ip: str):
    """
    Запустить бесконечный цикл перемещения робота между целевыми точками.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=0.2, acceleration=0.2)

    # Настройка параметров инструмента
    robot.tool.set(tool_end_point=(0, 0, 0, 0, 0, 0))

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    while robot.is_connected():
        # Добавление целевых точек
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

        # Запуск движения и ожидание его завершения
        robot.motion.mode.set("move")
        robot.motion.wait_waypoint_completion(0)


if __name__ == "__main__":
    # Запуск определенной выше функции
    cycle_movement(ROBOT_IP)
