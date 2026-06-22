"""
Примеры работы с диагностической информацией робота.
"""

from API import RobotApi

# IPv4 адрес целевого робота
ROBOT_IP: str = "127.0.0.1"


def read_basic_diagnostics(robot_ip: str):
    """
    Получение базовой диагностической информации о роботе.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    # Подключение к роботу в режиме 'read only'
    with RobotApi(ip=robot_ip, read_only=True) as robot:
        # Температура контроллера
        temp = robot.diagnostics.get_controller_temperature()
        print(f"Температура контроллера: {temp:.1f} °C")

        # Напряжение и ток питания робота
        voltage = robot.diagnostics.get_robot_voltage()
        current = robot.diagnostics.get_robot_current()
        print(f"Напряжение питания: {voltage:.2f} В")
        print(f"Ток потребления: {current:.2f} А")

        # Ток модулей ввода-вывода и инструмента
        io_current = robot.diagnostics.get_io_current()
        tool_current = robot.diagnostics.get_tool_current()
        print(f"Ток I/O: {io_current:.2f} А")
        print(f"Ток инструмента: {tool_current:.2f} А")


def read_joints_diagnostics(robot_ip: str):
    """
    Получение диагностической информации по сочленениям робота.

    Args:
        robot_ip: IPv4 адрес робота.
    """
    with RobotApi(ip=robot_ip, read_only=True) as robot:
        # Температуры двигателей и контроллеров сочленений
        motor_temps = robot.diagnostics.get_joints_motor_temperatures()
        board_temps = robot.diagnostics.get_joints_controller_temperatures()
        print(
            "Температуры двигателей сочленений (°C):",
            [f"{t:.1f}" for t in motor_temps],
        )
        print(
            "Температуры контроллеров сочленений (°C):",
            [f"{t:.1f}" for t in board_temps],
        )

        # Электрические параметры двигателей
        currents = robot.diagnostics.get_joints_currents()
        voltages = robot.diagnostics.get_joints_voltages()
        torques = robot.diagnostics.get_joints_torques()
        print("Токи двигателей (А):", [f"{i:.2f}" for i in currents])
        print("Напряжения двигателей (В):", [f"{v:.2f}" for v in voltages])
        print("Моменты двигателей (Н·м):", [f"{t:.2f}" for t in torques])


def monitor_diagnostics_continuously(
    robot_ip: str, duration_sec: float = 2.0, interval_sec: float = 0.1
):
    """
    Пример циклического опроса диагностических данных.

    Args:
        robot_ip: IPv4 адрес робота.
        duration_sec: Общая длительность мониторинга в секундах.
        interval_sec: Интервал между запросами (не чаще 0.01 с согласно документации).
    """
    import time

    with RobotApi(ip=robot_ip, read_only=True) as robot:
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            try:
                temp = robot.diagnostics.get_controller_temperature()
                voltage = robot.diagnostics.get_robot_voltage()
                current = robot.diagnostics.get_robot_current()
                print(
                    f"[{time.time():.3f}] "
                    f"Контроллер: {temp:.1f} °C | "
                    f"Питание: {voltage:.2f} В / {current:.2f} А"
                )
                time.sleep(interval_sec)
            except KeyboardInterrupt:
                print("\nМониторинг прерван пользователем.")
                break


if __name__ == "__main__":
    # Запуск примеров
    read_basic_diagnostics(ROBOT_IP)
    print("-" * 80)  # Визуальный разделитель примеров
    read_joints_diagnostics(ROBOT_IP)
    print("-" * 80)  # Визуальный разделитель примеров
    monitor_diagnostics_continuously(
        ROBOT_IP, duration_sec=5.0, interval_sec=0.1
    )
