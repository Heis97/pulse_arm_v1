"""
Работа с системами координат, их использование для задания целевых точек
движения робота.
"""

from API import RobotApi
from API.coords import CoordinateSystem

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def creation_user_coordinate_system():
    """
    Работа с системами координат.
    """

    # Создание системы координат
    base = CoordinateSystem(
        (0, 0, 0, 0, 0, 0),
        orientation_units="deg",
    )  # СК основания робота

    # Создание новой системы с помощью смещения и поворота исходной СК
    cs1 = base.offset(dy=0.2).rotate(drz=90)
    print("cs1:", cs1)
    # CoordinateSystem(0.00, 0.20, 0.00, 0.00, 0.00, 90.00 (3*m, 3*deg))

    # Преобразование единиц измерения существующей системы
    cs2 = cs1.with_units("rad")
    print("cs2:", cs2)
    # CoordinateSystem(0.00, 0.20, 0.00, 0.00, 0.00, 1.57 (3*m, 3*rad))

    # Сравнение систем координат
    close = cs1.is_close(cs2, atol=0.001)
    print(f"Координатные системы 'cs1' и 'cs2' равны: {close}")

    # Смещение системы координат
    cs3 = cs1.offset(dx=0.3, dz=0.4)
    print("cs3:", cs3)
    # CoordinateSystem(0.00, 0.50, 0.40, 0.00, 0.00, 90.00 (3*m, 3*deg))

    # Расчет расстояния между центрами систем координат
    distance = cs3.distance_to(cs1)
    print("Расстояние между 'cs1' и 'cs3':", distance, "метров")

    # Создание системы координат с указанным направлением осей
    x_target = (1, 1, 0)  # Точка, лежащая на оси X
    cs4 = base.align_with_vector(target=x_target)
    print("cs4:", cs4)
    # CoordinateSystem(0.00, 0.00, 0.00, 0.00, 0.00, 45.00 (3*m, 3*deg))


def using_user_coordinate_system(robot_ip: str):
    """
    Перемещение робота в глобальной и локальной системах координат.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=0.4, acceleration=0.4)

    # Настройка параметров инструмента
    robot.tool.set(tool_end_point=(0, 0, 0.14329, 0, 0, 0))

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Определение локальной системы координат
    local_coord_system = CoordinateSystem(
        position_orientation=((-0.3332, -0.1838, -0.0198, 3.138, 0, 0.8195)),
        orientation_units="deg",
    )
    target_point = (-0.45, -0.1, 0.2, -135, 15, 45)

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

    # Перемещение в целевую точку, заданную в глобальной системе координат
    robot.motion.joint.add_new_waypoint(
        tcp_pose=target_point,
        speed=30,
        accel=30,
        units="deg",
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)

    global_pose = robot.motion.get_actual_position(
        orientation_units="deg", position_format="tcp"
    )
    local_pose = robot.motion.get_actual_position(
        orientation_units="deg",
        position_format="tcp",
        coordinate_system=local_coord_system,
    )
    print(
        "Положение робота: ",
        f"\tв глобальной системе координат: {global_pose}",
        f"\tв локальной системе координат: {local_pose}",
        sep="\n",
    )

    # Перемещение в целевую точку, заданную в локальной системе координат
    robot.motion.joint.add_new_waypoint(
        tcp_pose=target_point,
        speed=30,
        accel=30,
        units="deg",
        coordinate_system=local_coord_system,
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)

    global_pose = robot.motion.get_actual_position(
        orientation_units="deg", position_format="tcp"
    )
    local_pose = robot.motion.get_actual_position(
        orientation_units="deg",
        position_format="tcp",
        coordinate_system=local_coord_system,
    )
    print(
        "Положение робота: ",
        f"\tв глобальной системе координат: {global_pose}",
        f"\tв локальной системе координат: {local_pose}",
        sep="\n",
    )

    # Использование контекстного менеджера для указания системы координат
    with local_coord_system.in_frame():
        # Явно не указываем систему координат - берется система из контекстного
        # менеджера. Удобно использовать при вызове большого количества команд,
        # использующих системы координат
        local_pose = robot.motion.get_actual_position(
            orientation_units="deg",
            position_format="tcp",
        )
        global_coord_system = local_coord_system.copy().set((0, 0, 0, 0, 0, 0))
        # Применится явно указанная система, а не из контекстного менеджера
        global_pose = robot.motion.get_actual_position(
            orientation_units="deg",
            position_format="tcp",
            coordinate_system=global_coord_system,  # Вот тут явно указали
        )

        # Результат будет идентичен предыдущему
        print(
            "Положение робота: ",
            f"\tв глобальной системе координат: {global_pose}",
            f"\tв локальной системе координат: {local_pose}",
            sep="\n",
        )


if __name__ == "__main__":
    # Запуск определенных выше функций
    creation_user_coordinate_system()
    using_user_coordinate_system(ROBOT_IP)
