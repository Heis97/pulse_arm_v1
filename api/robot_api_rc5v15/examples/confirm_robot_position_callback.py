"""
Задание обработчика для подтверждения позиции робота при рассогласовании
реального положения с последним сохраненным в контроллере.

Позволяет переопределить стандартное подтверждение позиции (через пульт
оператора) на другое. Рекомендуется использовать только в случае крайней
необходимости, либо, с полным пониманием того, к чему может привести
необдуманное подтверждение позиции.

В API реализован стандартный терминальный обработчик, который запрашивает
подтверждение через терминал, из которого была запущена программа. В ситуациях,
когда программа начинает работу через автозапуск, терминальный обработчик не
позволит подтвердить позицию из-за недоступности терминала.

В таком случае необходимо реализовать подтверждение другим способом, например,
можно расширить границы допустимого рассогласования углов. Вариант исполнения
такого обработчика представлен ниже.
"""

from typing import Tuple

from API import RobotApi
from API.types import JointAngleDiscrepancy

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def handle_robot_joint_angle_discrepancy(
    data: Tuple[JointAngleDiscrepancy, ...],
) -> bool:
    """
    Обработчик подтверждения положения, расширяющий границу
    допустимого расхождения углов.
    """
    # Задаем максимально допустимое рассогласование углов
    ANGLE_DISCREPANCY_THRESHOLD = 10  # градусов

    # По умолчанию считаем, что позиция подтверждена
    is_confirmed = True

    # Проверяем данные каждого полученного звена
    for joint in data:
        # Если модуль рассогласования углов превышает пороговое значение,
        # то считаем, что позиция не подтверждена
        if (
            abs(joint.actual_position - joint.saved_position)
            > ANGLE_DISCREPANCY_THRESHOLD
        ):
            is_confirmed = False
    return is_confirmed


def test_confirm_position_callback(robot_ip: str):
    """
    Проверить работу переопределенной функции подтверждения позиции.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Создание объекта API для управления роботом
    robot = RobotApi(ip=robot_ip, show_std_traceback=True, autoconnect=False)

    # Задание обработчика для подтверждения позиции
    robot.controller.state.set_confirm_position_callback(
        handle_robot_joint_angle_discrepancy
    )

    # Подключение к роботу
    robot.connect()
    robot.controller.state.set("off")

    # Настройка параметров нагрузки
    robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))

    # Настройка параметров движения
    robot.motion.scale_setup.set(velocity=1, acceleration=1)

    # Запуск робота, в этот момент будет вызвана функция подтверждения позиции,
    # если имеет место рассогласование углов.
    robot.controller.state.set("run", await_sec=120)

    # Переход в положение 'семерка'
    robot.motion.joint.add_new_waypoint(
        angle_pose=(0, -120, 120, -90, -90, 0),
        speed=100,
        accel=10,
        blend=0,
        units="deg",
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)


if __name__ == "__main__":
    # Запуск определенной выше функции
    test_confirm_position_callback(ROBOT_IP)
