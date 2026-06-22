"""
Управление захватом DH Robotics PGC-140-50 (возможно, будет работать и с другими
захватами серии PGC) через RS-485 интерфейс запястья робота.

Данный пример демонстрирует:
- настройку Modbus-клиента для работы с устройством на шине RS-485 (создание
    'драйвера' устройства),
- запись параметров (позиция, усилие, скорость),
- чтение текущего состояния захвата.

Захват должен быть подключён к RS-485 порту платы запястья робота.
"""

import time

from API import RobotApi
from API.io import WristModbusRS485Client

# IPv4 адрес целевого робота
ROBOT_IP: str = "192.168.0.153"


class DHRoboticsPGCGripper:
    """
    Класс-драйвер для управления захватом DH Robotics PGC-140-50.

    Реализует базовые операции через Modbus RTU-регистры, как описано в
    документации производителя. Ограничения позиции (500 - 1000) обусловлено
    особенностью конструкции (при подаче меньшего значения губки захвата
    сталкиваются).
    """

    def __init__(self, modbus_client: WristModbusRS485Client):
        self._modbus_client = modbus_client

    def connect(self):
        """
        Установить соединение с захватом.
        """
        self._modbus_client.connect()

    def disconnect(self):
        """
        Разорвать соединение с захватом.
        """
        self._modbus_client.close()

    def set_gripper_pos(self, value: int = 600) -> None:
        """
        Устанавливает целевую позицию захвата.

        Args:
            value: Значение позиции (500–1000).
        """
        self._modbus_client.write_register(
            address=0x103, value=value, no_response_expected=True
        )

    def get_gripper_pos(self) -> int:
        """
        Возвращает фактическую позицию захвата.

        Returns:
            Текущая позиция захвата.
        """
        resp = self._modbus_client.read_holding_registers(address=0x202)
        return resp.registers[0]

    def set_gripper_force(self, value: int = 600) -> None:
        """
        Устанавливает усилие сжатия захвата.

        Args:
            value: Усилие (0–1000, где 1000 — максимальное).
        """
        self._modbus_client.write_register(
            address=0x101, value=value, no_response_expected=True
        )

    def get_gripper_force(self) -> int:
        """
        Возвращает заданное усилие захвата.

        Returns:
            Заданное усилие захвата.
        """
        resp = self._modbus_client.read_holding_registers(address=0x101)
        return resp.registers[0]

    def set_gripper_speed(self, value: int = 600) -> None:
        """
        Устанавливает скорость движения захвата.

        Args:
            value: Скорость захвата (0–1000, где 1000 — максимальная).
        """
        self._modbus_client.write_register(
            address=0x104, value=value, no_response_expected=True
        )

    def get_gripper_speed(self) -> int:
        """
        Возвращает заданную скорость захвата.

        Returns:
            Заданная скорость захвата.
        """
        resp = self._modbus_client.read_holding_registers(address=0x104)
        return resp.registers[0]


def control_gripper(robot_ip: str) -> None:
    """
    Управляет захватом DH Robotics через RS-485 порт запястья.

    Последовательность действий:
    1. Подключение к роботу.
    2. Перевод платы запястья в режим RS-485.
    3. Создание Modbus-клиента для захвата.
    4. Настройка параметров и выполнение операций.

    Важно отметить, что для использования RS-485 порта подключение с роботом
    (через RobotApi) устанавливать не обязательно, но НЕОБХОДИМО, чтобы плата
    запястья робота находилась в режиме 'rs485'. В данном примере подключение к
    с роботом устанавливается исключительно для проверки режима работы платы
    запястья.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    robot = RobotApi(
        ip=robot_ip, show_std_traceback=True, autoconnect=True, read_only=True
    )
    # Убедимся, что плата запястья в нужном режиме
    if robot.wrist.get_mode() != "rs485":
        with robot.connected(read_only=False):
            robot.wrist.set_mode("rs485", await_sec=30)

    # Создание Modbus-клиента для захвата
    modbus_client = WristModbusRS485Client(host=robot_ip)

    # Помимо представленного выше способа можно создать Modbus-клиента, передав
    # уже созданный WristRS485 объект класса:
    #
    # from API.io import WristRS485
    # rs485 = WristRS485(robot_ip)
    # modbus_client = WristModbusRS485Client(wrist_rs_485=rs485)

    # Либо, если создан и используется экземпляр класса RobotApi, то можно
    # сделать так:
    #
    # modbus_client = WristModbusRS485Client(wrist_rs_485=robot.wrist.rs_485)

    # Инициализация драйвера захвата
    gripper = DHRoboticsPGCGripper(modbus_client)

    # Подключение к захвату
    gripper.connect()

    pos = gripper.get_gripper_pos()
    print("Начальная позиция захвата:", pos)

    # Полностью открываем захват
    gripper.set_gripper_speed(80)
    gripper.set_gripper_pos(1000)
    time.sleep(1)

    open_pos: int = 1000
    close_pos: int = 500

    # Изменяем положение захвата с разной скоростью
    for i in range(1, 6):
        velocity = 10 * i
        moving_time = abs(open_pos - close_pos) / (velocity * 10) + 0.2
        print("Время перемещения захвата:", moving_time, "с")
        gripper.set_gripper_speed(velocity)
        gripper.set_gripper_pos(close_pos)
        time.sleep(moving_time)
        gripper.set_gripper_pos(open_pos)
        time.sleep(moving_time)

    # Полностью открываем захват
    gripper.set_gripper_speed(80)
    gripper.set_gripper_pos(1000)
    time.sleep(1)

    # Отключение от захвата
    gripper.disconnect()


if __name__ == "__main__":
    # Запуск управления захватом
    control_gripper(ROBOT_IP)
