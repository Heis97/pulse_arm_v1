"""
Пример ServoJ с адаптивным управлением потоком (Queue-Driven Flow Control).
Отправка пачками динамически подстраивается под доступные слоты RTD-очереди
и подтверждение контроллера (ACK). Такой подход является предпочтительным для
использования, так как имеет обратную связь по количеству точек в буфере.
"""

import math
import time
from typing import Tuple

from  api.robot_api_rc5v15.API.rc_api import *

# Конфигурация
ROBOT_IP: str = "10.10.10.3"
STREAM_DURATION_S: float = 50.0
CYCLE_TIME_S: float = 0.002  # 2 мс (500 Гц)
START_POSE_DEG: Tuple[float, ...] = (0.0, -120.0, 120.0, -90.0, -90.0, 0.0)

# Параметры flow-control
TARGET_BURST: int = 16  # Максимальный размер пачки за один цикл
SAFETY_MARGIN: int = 1  # Резерв слотов для избежания переполнения
ACK_TIMEOUT_S: float = 1.0  # Таймаут ожидания подтверждения от контроллера
POLL_INTERVAL_S: float = 0.002  # Базовый интервал опроса RTD-статуса
PROGRESS_LOG_S: float = 1.0  # Интервал вывода прогресса в консоль


# Вспомогательные функции
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
def adaptive_flow_control_servoj(robot_ip: str) -> None:
    with RobotApi15(
        ip=robot_ip,
        show_std_traceback=True,
        autoconnect=True,
        log_realtime=False,  # Можно установить True для подробного вывода лога
    ) as robot:
        # 1. Подготовка и переход в стартовую позу
        robot.controller.state.set("off")
        robot.payload.set(mass=0.0, tcp_mass_center=(0.0, 0.0, 0.0))
        robot.motion.scale_setup.set(velocity=1.0, acceleration=1.0)
        robot.controller.state.set("run", await_sec=120)

        robot.motion.joint.add_new_waypoint(
            angle_pose=START_POSE_DEG,
            speed=30.0,
            accel=60.0,
            blend=0.0,
            units="deg",
        )
        robot.motion.mode.set("move")
        robot.motion.wait_waypoint_completion(0, await_sec=60)

        # 2. Переход в режим реального времени
        robot.motion.realtime.set_servoj_params(
            gain=800.0, lookahead_time=0.005
        )

        base_pose = tuple(robot.motion.joint.get_actual_position(units="rad"))
        amplitude = math.radians(25.0)
        frequency_hz = 0.5
        total_samples = max(
            int(math.ceil(STREAM_DURATION_S / CYCLE_TIME_S)), 1
        )
        samples_sent = 0
        last_rtd_ts = None
        last_log_ts = time.monotonic()

        robot.motion.mode.set("realtime")

        print(f"Запуск стриминга: RTD Flow-Control | {total_samples} шагов")

        # 3. Основной цикл с постоянной проверкой очереди
        try:
            while samples_sent < total_samples:
                # Опрос статуса очереди
                status = robot.motion.realtime.get_motion_queue_status()
                if status is None:
                    continue

                # Пропускаем повторный опрос, если данные не обновились
                if status.timestamp == last_rtd_ts:
                    time.sleep(POLL_INTERVAL_S)
                    continue
                last_rtd_ts = status.timestamp

                # Рассчитываем безопасное количество точек для отправки
                usable = max(status.available_slots - SAFETY_MARGIN, 0)
                burst = min(usable, TARGET_BURST, total_samples - samples_sent)
                if burst <= 0:
                    time.sleep(POLL_INTERVAL_S)
                    continue

                # Отправляем пачку
                base_cnt = status.received_count
                for _ in range(burst):
                    t_s = samples_sent * CYCLE_TIME_S
                    robot.motion.realtime.servoj(
                        target_pose_at(
                            t_s, base_pose, amplitude, frequency_hz
                        ),
                        time=CYCLE_TIME_S,
                        units="rad",
                    )
                    samples_sent += 1

                # Блокирующее ожидание подтверждения (flow-control)
                robot.motion.realtime.wait_waypoints_received(
                    base_received_count=base_cnt,
                    expected_delta=burst,
                    timeout=ACK_TIMEOUT_S,
                )
                time.sleep(0)  # Yield GIL после ACK

                # Периодический вывод прогресса
                now = time.monotonic()
                if now - last_log_ts >= PROGRESS_LOG_S:
                    pct = (samples_sent / total_samples) * 100
                    print(
                        f"  Прогресс: {samples_sent}/{total_samples} ({pct:.1f}%)"
                    )
                    last_log_ts = now

        finally:
            robot.motion.realtime.stopj()
            print("Движение завершено. Робот остановлен.")

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
    adaptive_flow_control_servoj(ROBOT_IP)
