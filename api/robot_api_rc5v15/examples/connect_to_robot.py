"""
Подключение к роботу.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def autoconnect_to_robot(robot_ip: str):
    """
    Подключение к роботу при создании экземпляра класса API.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу при создании экземпляра класса
    robot = RobotApi(ip=robot_ip, show_std_traceback=True)
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено True

    # Отключение от робота
    robot.disconnect()
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено False

    # Повторное подключение без создания нового экземпляра класса
    robot.connect()
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено True

    # Отключение от робота
    robot.disconnect()


def advanced_connect_to_robot(robot_ip: str):
    """
    Управление подключением к роботу.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Создание экземпляра класса
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=False)
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено False

    # Подключение к роботу
    robot.connect()
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено True

    # Отключение от робота
    robot.disconnect()
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено False

    # Повторное подключение к роботу в режиме "read only"
    robot.connect(read_only=True)
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено True

    print(
        "Текущее положение робота: ",
        robot.motion.get_actual_position(
            orientation_units="deg", position_format="joints"
        ),
    )
    # Отключение от робота
    robot.disconnect()


def connect_to_robot_using_with(robot_ip: str):
    """
    Использование контекстного менеджера with для подключения к роботу.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Создание экземпляра класса
    with RobotApi(ip=robot_ip, show_std_traceback=True) as robot:
        print(
            "Подключение с роботом установлено: ", robot.is_connected()
        )  # Будет выведено True
    # В этом месте подключение будет автоматически разорвано

    # Более удобный вариант использования
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=False)
    # Подключение к роботу в режиме "read only"
    with robot.connected(read_only=True):
        print(
            "Подключение с роботом установлено: ", robot.is_connected()
        )  # Будет выведено True

        print(
            "Текущее положение робота: ",
            robot.motion.get_actual_position(
                orientation_units="deg", position_format="joints"
            ),
        )
    # В этом месте подключение будет автоматически возвращено в состояние
    # до блока with (в данном случае разорвано)

    # Подключаемся к роботу в режиме 'read only'
    robot.connect(read_only=True)
    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено True

    # Вызываем блок кода с подключением в полноценном режиме
    with robot.connected(read_only=False) as r:
        print(
            "Подключение с роботом установлено: ", r.is_connected()
        )  # Будет выведено True

        print("Включение робота...")
        r.controller.state.set("run")
    # В этом месте подключение будет автоматически возвращено в состояние
    # до блока with (в данном случае в режим 'read only')

    print(
        "Подключение с роботом установлено: ", robot.is_connected()
    )  # Будет выведено True

    robot.disconnect()


if __name__ == "__main__":
    # Запуск определенных выше функции
    autoconnect_to_robot(ROBOT_IP)
    advanced_connect_to_robot(ROBOT_IP)
    connect_to_robot_using_with(ROBOT_IP)
