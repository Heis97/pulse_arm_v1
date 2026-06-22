"""
Перемещение робота с помощью режима jogging - цикличное изменение выбранной
координаты.
Длительность задержки между командами влияет на пройденный роботом путь.

! Одновременное изменение более одной координаты не поддерживается.
"""

import time

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def robot_jogging(robot_ip: str):
    """
    Изменение положения робота с помощью режима движения jogging.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения, эти настройки влияют на скорость jogging-га.
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Перемещение в положение 'семерка'
    print("Перемещение в положение 7")
    robot.motion.joint.add_new_waypoint(
        angle_pose=(0, -120, 120, -90, -90, 0),
        speed=100,
        accel=10,
        blend=0,
        units="deg",
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)

    print("Вращение 0 звена")
    for _ in range(200):
        robot.motion.joint.jog_once(0, "+")
        time.sleep(0.01)

    print("Вращение 4 звена")
    for _ in range(200):
        robot.motion.joint.jog_once(4, "-")
        time.sleep(0.01)

    print("Опускание ЦТИ по оси Z")
    for _ in range(200):
        robot.motion.linear.jog_once("Z", "-")
        time.sleep(0.01)

    print("Вращение ЦТИ вокруг оси X")
    for _ in range(200):
        robot.motion.linear.jog_once("Rx", "+")
        time.sleep(0.01)


if __name__ == "__main__":
    # Запуск определенной выше функции
    robot_jogging(ROBOT_IP)
