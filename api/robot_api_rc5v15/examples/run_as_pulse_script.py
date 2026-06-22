"""
Пример работы со скриптом в режиме совместимости с ПО 'Пульс'.

Содержит как базовое взаимодействие с переменными, так и более приближенные к
реальности примеры использования предоставляемого функционала.
"""

from API import RobotApi
from API.tools import (
    impulse_vars,
    load_impulse_vars,
    save_impulse_vars,
    send_error_to_impulse,
)


def adjust_robot_parameters():
    """
    Менее абстрактный пример, имитирующий взаимодействие с реальным роботом.
    Корректирует параметры движения робота, полученные из ПО 'Пульс', и
    сохраняет изменения обратно.
    """
    # Работа с конкретными переменными 'Пульса' через контекстный менеджер
    with impulse_vars("speed", "tool_id", "gripper_force") as vars:
        # Чтение существующих переменных (должны быть определены в 'Пульсе')
        try:
            # Зададим аннотацию типов через определение новых переменных
            current_speed: float = vars.speed  # например, speed = 50
            tool_id: int = vars.tool_id  # например, tool_id = 1
        except AttributeError as e:
            send_error_to_impulse(f"Требуемая переменная не найдена: {e}")
            return

        # Валидация входных данных
        if not (10 <= current_speed <= 100):
            send_error_to_impulse("Недопустимое значение скорости")
            return

        # Обновление существующей переменной
        vars.speed = min(current_speed + 10, 100)

        # Дополнительная логика работы с захватом
        if not vars.has_var("gripper_force"):
            send_error_to_impulse(
                "Переменная 'gripper_force' не была загружена"
            )
            return
        if tool_id == 1:
            vars.gripper_force = 80
        else:
            vars.gripper_force = 50

        # Все изменения (в данном случае speed, gripper_force)
        # будут автоматически сохранены в 'Пульс' при выходе из блока


def tracking_robot_position():
    """
    Отслеживание положения ЦТИ робота и изменение значения переменной при заходе
    в запретную зону.
    """
    # Определяем границы запретной зоны (в метрах)
    FORBIDDEN_X_MIN = 0.200
    FORBIDDEN_X_MAX = 0.300
    FORBIDDEN_Y_MIN = -0.100
    FORBIDDEN_Y_MAX = 0.100
    FORBIDDEN_Z_MIN = 0
    FORBIDDEN_Z_MAX = 0.150

    loaded_vars = {}
    # Загружаем переменную, отвечающую за остановку программы
    load_impulse_vars("need_stop", target=loaded_vars)

    if loaded_vars["need_stop"]:
        return

    with RobotApi("127.0.0.1", read_only=True) as robot:
        # IP адрес может быть любым (например, адрес реального робота, на
        # котором тестировалась логика скрипта без изменения переменных
        # "Пульса"), он автоматически будет заменен на необходимый при запуске
        # на "Пульсе".

        # Флаг read_only обязателен, управление роботом в режиме совместимости
        # с "Пульсом" пока недоступно.
        while True:
            position = robot.motion.get_actual_position(
                position_format="tcp", orientation_units="deg"
            )

            x, y, z = position[:3]  # берём только координаты

            # Проверяем попадание в запретную зону
            in_forbidden = (
                FORBIDDEN_X_MIN <= x <= FORBIDDEN_X_MAX
                and FORBIDDEN_Y_MIN <= y <= FORBIDDEN_Y_MAX
                and FORBIDDEN_Z_MIN <= z <= FORBIDDEN_Z_MAX
            )
            if in_forbidden:
                loaded_vars["need_stop"] = False
                save_impulse_vars(target=loaded_vars)
                # Можно не указывать имя переменной, так
                # как была загружена только одна переменная
                return


if __name__ == "__main__":
    # Запуск определённых выше функций
    # Запуск этих функций не на "Пульсе" приведет к ошибке. Рекомендуется
    # вынести каждый пример в отдельный файл перед запуском на "Пульсе"
    adjust_robot_parameters()
    tracking_robot_position()
