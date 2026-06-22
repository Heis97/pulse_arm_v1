"""
Простой пример стриминга ServoJ по фиксированному таймеру с пакетной отправкой.
Такой подход не рекомендуется использовать при запуске программы не на
контроллере ввиду отсутствия обратной связи по количеству точек в буфере.
"""

import math
import time
from typing import Tuple

from API import RobotApi

# Конфигурация
ROBOT_IP: str = "127.0.0.1"
STREAM_DURATION_S: float = 50.0
CYCLE_TIME_S: float = 0.002  # 2 мс (500 Гц)
PREFILL_TARGET: int = 16  # Сколько точек отправить до старта цикла
SAFETY_MARGIN: int = 2  # Резерв слотов для избежания переполнения
START_POSE_DEG: Tuple[float, ...] = (0.0, -120.0, 120.0, -90.0, -90.0, 0.0)


# Вспомогательные функции
def sleep_until(deadline_s: float, spin_window_s: float = 0.0002) -> None:
    """
    Ждёт до абсолютного дедлайна. Финальное спин-окно с time.sleep(0)
    отдаёт GIL фоновым потокам, не блокируя CPU.
    """
    remaining = deadline_s - time.monotonic()
    if remaining > spin_window_s:
        time.sleep(remaining - spin_window_s)
    while time.monotonic() < deadline_s:
        time.sleep(0)


def target_pose_at(
    t_s: float,
    base_pose: Tuple[float, ...],
    amplitude: float,
    frequency_hz: float,
) -> Tuple[float, ...]:
    """Генерирует целевую позу на момент времени t_s."""
    p = list(base_pose)
    p[0] += 4.0 * amplitude * math.sin(2 * math.pi * frequency_hz / 10 * t_s)
    p[4] += 3.0 * amplitude * math.sin(2 * math.pi * frequency_hz * t_s)
    p[5] += 4.0 * amplitude * math.sin(2 * math.pi * frequency_hz * t_s)
    return tuple(p)


# Основная программа
def run_simple_servoj(robot_ip: str) -> None:
    with RobotApi(ip=robot_ip, autoconnect=True) as robot:
        # 1. Подготовка и переход в стартовую позу
        robot.controller.state.set("off")
        robot.payload.set(mass=0.0, tcp_mass_center=(0.0, 0.0, 0.0))
        robot.motion.scale_setup.set(velocity=1.0, acceleration=1.0)
        robot.controller.state.set("run", await_sec=30)

        robot.motion.joint.add_new_waypoint(
            angle_pose=START_POSE_DEG,
            speed=30.0,
            accel=60.0,
            blend=0.0,
            units="deg",
        )
        robot.motion.mode.set("move")
        robot.motion.wait_waypoint_completion(0, await_sec=30)

        # 2. Включение режима реального времени
        robot.motion.realtime.set_servoj_params(
            gain=800.0, lookahead_time=0.005
        )
        robot.motion.mode.set("realtime")

        base_pose = tuple(robot.motion.joint.get_actual_position(units="rad"))
        amplitude = math.radians(25.0)
        frequency_hz = 0.5
        start_time = time.monotonic()
        total_samples = int(STREAM_DURATION_S / CYCLE_TIME_S)

        # 3. Предзаполнение буфера
        status = robot.motion.realtime.get_motion_queue_status()
        if status is None:
            raise RuntimeError(
                "RTD-статус недоступен. Предзаполнение невозможно."
            )

        # Сколько точек безопасно отправить прямо сейчас
        available = max(status.available_slots - SAFETY_MARGIN, 0)
        prefill_count = min(PREFILL_TARGET, available, total_samples)

        print(f"Предзаполнение буфера: отправляем {prefill_count} точек...")
        for i in range(prefill_count):
            t_s = i * CYCLE_TIME_S
            robot.motion.realtime.servoj(
                target_pose_at(t_s, base_pose, amplitude, frequency_hz),
                time=CYCLE_TIME_S,
                units="rad",
            )

        samples_sent = prefill_count
        # Корректируем старт цикла: контроллер уже исполняет отправленные точки.
        # Сдвигаем временную базу назад на время, которое они займут.
        start_time = time.monotonic() - (samples_sent * CYCLE_TIME_S)

        # 4. Основной цикл
        print(f"Запуск таймерного стриминга: всего {total_samples} шагов")

        try:
            while samples_sent < total_samples:
                # Размер пачки для текущего цикла (обычно 1, но можно увеличить)
                batch = min(1, total_samples - samples_sent)

                for _ in range(batch):
                    t_s = samples_sent * CYCLE_TIME_S
                    robot.motion.realtime.servoj(
                        target_pose_at(
                            t_s, base_pose, amplitude, frequency_hz
                        ),
                        time=CYCLE_TIME_S,
                        units="rad",
                    )
                    samples_sent += 1

                # Синхронизация с исполнением контроллером
                sleep_until(start_time + samples_sent * CYCLE_TIME_S)

                # Лёгкий вывод прогресса
                if samples_sent % 500 == 0:
                    elapsed = time.monotonic() - start_time
                    print(
                        f"  Отправлено {samples_sent}/{total_samples} | t={elapsed:.2f}s"
                    )
                    time.sleep(0)
        finally:
            robot.motion.realtime.stopj()
            print("Движение завершено")

        # Явный переход в hold перед добавлением новых точек
        # (фактически ожидание завершения realtime движений)
        robot.motion.mode.set("hold")

        robot.motion.joint.add_new_waypoint(
            angle_pose=START_POSE_DEG,
            speed=30,
            accel=60,
            blend=0.01,
            units="deg",
        )
        robot.motion.mode.set("move")
        robot.motion.wait_waypoint_completion()


if __name__ == "__main__":
    run_simple_servoj(ROBOT_IP)
