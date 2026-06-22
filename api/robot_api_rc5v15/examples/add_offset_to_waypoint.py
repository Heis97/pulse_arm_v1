"""
Добавление смещения к целевой точке.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def add_offset_to_point(robot_ip: str):
    """
    Переместить робота в целевую точку и точку, смещенную относительно целевой.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=0.3, acceleration=0.3)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Перемещение робота в рабочее положение
    robot.motion.joint.add_new_waypoint(
        angle_pose=(0, -115, 120, -100, -90, 0),
        speed=70,
        accel=70,
        blend=0,
        units="deg",
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)

    target_waypoint = (-0.44, -0.16, 0.337, -175, 0, 90)
    target_offset = (0.1, 0.5, 0, 0, 0, 0)

    # Перемещение к целевой точке
    robot.motion.linear.add_new_waypoint(
        tcp_pose=target_waypoint,
        speed=0.5,
        accel=0.5,
        orientation_units="deg",
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)
    print(
        "Положение в целевой точке: ",
        robot.motion.get_actual_position(
            orientation_units="deg", position_format="tcp"
        ),
    )

    # Перемещение к точке, смещенной относительно целевой
    robot.motion.linear.add_new_offset(
        waypoint=target_waypoint,
        offset=target_offset,
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)
    print(
        "Положение при смещении относительно целевой точки: ",
        robot.motion.get_actual_position(
            orientation_units="deg", position_format="tcp"
        ),
    )


if __name__ == "__main__":
    # Запуск определенной выше функции
    add_offset_to_point(ROBOT_IP)
