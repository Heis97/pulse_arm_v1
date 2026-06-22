"""
Выбор действия в зависимости от условий. Данный пример показывает один из
способов организации управляющей программы, в которой происходит выбор
определенного действия в зависимости от некоторого условия.

В данном случае в качестве условий будут выступать номера активных цифровых
входов, так как такой пример можно повторить без дополнительного оборудования.
В реальном же применении информация о требуемом выборе может поступать по
любому другому каналу: Modbus соединение, PLC контроллер, TCP/gRPC сервер и т.д.
Рекомендуется обратить внимание способ организации программы, а не на способ
получения информации о требуемом выборе.
"""

import time
from typing import List

from API import RobotApi
from API.types import DigitalIndex

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"
# Индексы цифровых входов, которые используются для выбора действий
CONTROL_INDEXES: List[DigitalIndex] = [0, 1, 2, 3, 4, 5]
# Время работы управляющего цикла
CYCLE_TIME: float = 0.1  # sec

# Для организации программы рекомендуется написать обработчик (функцию,
# описывающую порядок действий) для каждого случая. Такой подход упростит
# понимание программы и позволит просто и быстро ее расширять.
# Рекомендуется давать обработчикам названия, отражающие их суть.

# При написании обработчиков договоримся, что каждый обработчик должен быть
# блокирующим, то есть не должен завершаться до полного выполнения действия.
# Определим несколько таких обработчиков.


def handle_move_to_7_pose(robot: RobotApi):
    """
    Обработчик перемещения робота в положение 'семерка'.

    Args:
        robot: готовый к работе экземпляр класса RobotApi.
    """
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


def handle_move_to_home_pose(robot: RobotApi):
    """
    Обработчик перемещения робота в домашнее положение.

    Args:
        robot: готовый к работе экземпляр класса RobotApi.
    """
    robot.motion.move_to_home_pose()
    robot.motion.wait_waypoint_completion(0)


def handle_activate_conveyor(robot: RobotApi):
    """
    Обработчик запуска конвейерной ленты (предположим, что она запускается
    при логической 1 на 0 цифровом выходе контроллера).

    Args:
        robot: готовый к работе экземпляр класса RobotApi.
    """
    robot.io.digital.set_output(index=0, value=True)


def handle_deactivate_conveyor(robot: RobotApi):
    """
    Обработчик запуска конвейерной ленты (предположим, что она останавливается
    при логическом 0 на 0 цифровом выходе контроллера).

    Args:
        robot: готовый к работе экземпляр класса RobotApi.
    """
    robot.io.digital.set_output(index=0, value=False)


def handle_wait_product(robot: RobotApi):
    """
    Обработчик ожидания доставки продукта по конвейерной ленте (предположим,
    что датчик уведомляет нас о доставке путем выставления логической 1 на 20
    цифровом входе контроллера).

    Args:
        robot: готовый к работе экземпляр класса RobotApi.
    """
    robot.io.digital.wait_input(index=20, value=True)


# Далее определим вспомогательную функцию для получения управляющего сигнала с
# цифрового входа


def get_active_control_input_index(robot: RobotApi) -> DigitalIndex:
    """
    Метод получения индекса управляющего цифрового входа. Примем, что для
    указания требуемого действия из 6 (5 определенных выше +1 служебный без
    обработчика) должны использоваться цифровые входы контроллера 0-5.
    Остальные входы могут использоваться под другие задачи. При этом выбор
    действия происходи установкой выбранного входа в логическую 1.

    Args:
        robot: готовый к работе экземпляр класса RobotApi.
    """

    # Бесконечно ждем логическую 1 на любом из интересующих нас цифровом входе
    while True:
        for index in CONTROL_INDEXES:
            if robot.io.digital.get_input(index=index):
                # На входе логическая 1
                return index
        # Ожидание перед следующей проверкой
        time.sleep(CYCLE_TIME)


def control_loop(robot_ip: str):
    """
    Основная логика управляющего цикла. Организует подключение, выбор действия
    и отключение.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=True)
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота
    robot.controller.state.set("run", await_sec=120)

    # Управляющий цикл
    while True:
        # Получаем индекс активного управляющего входа
        index = get_active_control_input_index(robot)

        # Определяем обработчики
        if index == 0:
            # Обработчик 0 индекса
            handle_move_to_7_pose(robot)

        # Обработчик 1 индекса
        elif index == 1:
            handle_move_to_home_pose(robot)

        # Обработчик 2 индекса
        elif index == 2:
            handle_activate_conveyor(robot)

        # Обработчик 3 индекса
        elif index == 3:
            handle_deactivate_conveyor(robot)

        # Обработчик 4 индекса
        elif index == 4:
            handle_wait_product(robot)

        # Обработчик 5 индекса (служебный)
        elif index == 5:
            # Останавливаем управляющий цикл
            break

        # Обработчик остальных индексов (в данном случае бесполезен)
        else:
            print(f"Действие с индексом {index} не поддерживается")

    # При штатном выходе из цикла выключаем робота
    if robot.is_connected():
        robot.controller.state.set("off")
    robot.disconnect()


if __name__ == "__main__":
    # Запуск определенного выше управляющего цикла
    control_loop(ROBOT_IP)

    # Стоит обратить внимание, что для организации отказоустойчивой системы
    # вызов функции control_loop(ROBOT_IP) стоит поместить в бесконечный цикл
    # и обернуть конструкцией try - except.
