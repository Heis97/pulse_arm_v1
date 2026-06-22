"""
Минимально рабочий пример управления движением робота через скрипт в режиме
совместимости с ПО 'Пульс'.

Демонстрирует базовый сценарий формирования очереди точек (угловые и
линейные перемещения) и их выполнения.
"""

from API import RobotApi

# Количество циклов может задаваться через переменную ПО 'Пульс', в настоящем
# примере этого не делается для возможности запуска скрипта без создания
# каких-либо переменных в ПО 'Пульс'
ITERATIONS: int = 5


# Подключение к роботу. Указывать IP адрес необязательно. Подключиться можно
# только к роботу, на котором запускается скрипт.
robot = RobotApi()

# Настройка параметров движения
robot.motion.scale_setup.set(velocity=0.2, acceleration=0.2)

# Этап запуска робота: robot.controller.state.set("run", await_sec=120)
# пропускается, так как робот уже запущен (метод robot.controller.state.set
# недоступен в режиме совместимости с ПО 'Пульс')

for i in range(ITERATIONS):
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

# Не обязательно, но рекомендуется явно управлять отключением
robot.disconnect()
