"""
Перевод робота в положение 'семерка' (положение, в котором робот напоминает
цифру 7).
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def move_to_7_pose(robot_ip: str):
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

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Добавление положения 'семерка'
    robot.motion.joint.add_new_waypoint(
        angle_pose=(0, -120, 120, -90, -90, 0),
        speed=100,
        accel=10,
        blend=0,
        units="deg",
    )

    # Запуск движения и ожидание его завершения
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)


if __name__ == "__main__":
    # Запуск определенной выше функции
    move_to_7_pose(ROBOT_IP)
