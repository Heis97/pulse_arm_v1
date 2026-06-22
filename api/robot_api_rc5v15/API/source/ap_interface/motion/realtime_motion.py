from __future__ import annotations

import logging
import math
import time

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.argument_error import ArgValueError
from api.robot_api_rc5v15.API.source.core.exceptions.base_api_error import ApiError
from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import (
    RuntimeMotionError,
)
from api.robot_api_rc5v15.API.source.core.network import RealtimeController, RTDReceiver
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    degrees_to_radians,
)
from api.robot_api_rc5v15.API.source.features.tools import (
    normalize_joint_angles,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.realtime_structures import (
    MotionQueueStatus,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    RealtimeCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    POSITION_ORIENTATION_LENGTH,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import AngleUnits, PositionOrientation


class Realtime(LoggerMixin):
    """
    Класс для работы с управлением в режиме реального времени.
    """

    _MAX_WAYPOINTS_COUNT: int = 256
    _MAX_TIME: float = 0.02  # sec
    _RECOMMENDED_TIME: float = 0.002  # sec

    def __init__(
        self,
        realtime_controller: RealtimeController,
        connection_state: ConnectionState,
        rtd_receiver: RTDReceiver,
        log_realtime: bool = True,
        logger: logging.Logger | None = None,
    ) -> None:
        self._realtime_controller = realtime_controller
        self._rtd_receiver = rtd_receiver
        self._connection_state = connection_state
        self._log_realtime = log_realtime
        self._set_logger(logger)

    @handle_connection(available_in_read_only=False)
    def stopj(self) -> bool:
        """Немедленно останавливает движение всех сочленений робота в режиме реального времени.

        Команда прерывает выполнение текущей траектории, сбрасывает угловые скорости
        сочленений до нуля и очищает активную очередь команд движения. Используется
        для безопасного завершения или экстренной остановки во время работы
        в потоковых режимах `servoj` или `speedj`.

        Метод недоступен в режиме «read only» (требует активной сессии управления).

        Returns:
            bool: `True`, если команда остановки успешно отправлена контроллеру;
                `False` в случае ошибки соединения или внутренней ошибки отправки.

        Examples:
            >>> # Остановить движение после выполнения серии команд servoj
            >>> success = robot.motion.realtime.stopj()
            >>> if success:
            ...     print("Команда остановки успешно отправлена.")

            >>> # Безопасное завершение цикла управления
            >>> try:
            ...     while running:
            ...         robot.motion.realtime.servoj(target_angles, dt)
            ... finally:
            ...     robot.motion.realtime.stopj()

        Notes:
            - Контроллер обрабатывает команду только в режиме движения 'realtime'.
            - Остановка применяется ко всем сочленениям одновременно.
            - После вызова метода робот переходит в режим 'hold'.
            - Команда не сбрасывает системные ошибки или флаги безопасности
              контроллера, а только прерывает кинематическое движение.
        """
        self._ensure_socket_connected()
        return self._realtime_controller.send_command(
            RealtimeCommands.REALTIME_STOPJ
        )

    @handle_connection(available_in_read_only=False)
    def servoj(
        self,
        angle_pose: PositionOrientation,
        time: float = _RECOMMENDED_TIME,
        units: AngleUnits | None = None,
    ) -> bool:
        """Отправляет команду сервоуправления в пространстве сочленений (joint-space).

        Метод задаёт целевые углы для всех звеньев робота на заданный временной шаг,
        позволяя формировать высокочастотную траекторию в режиме реального времени.
        Команда отправляется асинхронно и обрабатывается внутренним контроллером
        на частоте цикла управления (~500 Гц).

        Метод недоступен в режиме «read only» (требует активной сессии управления).

        Args:
            angle_pose (PositionOrientation): Целевые углы сочленений.
                Ожидается структура или последовательность из 6 значений,
                соответствующих углам звеньев (J1–J6).
            time (float): Время шага дискретизации (в секундах).
                Определяет интервал, с которым контроллер должен интерполировать
                траекторию к следующей точке. По умолчанию: 0.002 c. Рекомендуется
                использовать значение 0.002 с, использование больших значение
                может привести к рывкам. Максимально допустимое значение - 0.02 с.
            units (AngleUnits, optional): Единицы измерения углов в `angle_pose`.
                Допустимые значения:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
                Если `None`, используются системные единицы по умолчанию.

        Returns:
            bool: `True`, если команда успешно отправлена.

        Examples:
            >>> import time
            >>>
            >>> # 1. Подготовка траектории: 16 целевых положений сочленений (J1–J6)
            >>> trajectory = [
            ...     ( 0.00, -1.57,  1.57,  0.00,  0.00,  0.00),  # Точка 1
            ...     ( 0.12, -1.50,  1.65, -0.05,  0.05,  0.10),  # Точка 2
            ...     # ... (точки 3–14) ...
            ...     (-0.18, -1.62,  1.42,  0.20, -0.08, -0.15)   # Точка 16
            ... ]
            >>>
            >>> # 2. Цикл отправки с адаптивными параметрами и контролем буфера
            >>> dt_base = 0.002  # базовый шаг дискретизации (2 мс / 500 Гц)
            >>> try:
            ...     for idx, joints in enumerate(trajectory, start=1):
            ...         # Контроль очереди: ждём освобождения, если буфер почти полон
            ...         queue = robot.motion.realtime.get_motion_queue_status()
            ...         if queue and queue.available_slots < 3:
            ...             time.sleep(0.002)
            ...
            ...         cycle_start = time.perf_counter()
            ...         success = robot.motion.realtime.servoj(joints, time=dt_base, units="rad")
            ...         if not success:
            ...             raise RuntimeError(f"Отказ отправки точки {idx}")
            ...
            ...         # Выравнивание цикла реального времени
            ...         elapsed = time.perf_counter() - cycle_start
            ...         time.sleep(max(0, dt_base - elapsed))
            ... finally:
            ...     robot.motion.realtime.stopj()

        Notes:
            - Контроллер обрабатывает команду только в режиме движения 'realtime'.
            - Команда не ожидает физического достижения роботом целевой точки;
              она лишь обновляет уставку (setpoint) на следующем цикле контроллера.
            - Для стабильного движения необходимо вызывать метод с постоянной
              частотой. Пропуски кадров или неравномерные интервалы могут вызвать
              рывки или активацию защитных алгоритмов контроллера.
            - Параметр `time` не является таймаутом выполнения; это параметр
              траекторного планирования, влияющий на плавность и ускорение.
            - При потере соединения или переполнении буфера новые команды будут
              проигнорированы.
        """
        return self._send_joints_command(
            RealtimeCommands.REALTIME_SERVOJ, angle_pose, time, units
        )

    @handle_connection(available_in_read_only=False)
    def speedj(
        self,
        target_velocities: PositionOrientation,
        time: float = _RECOMMENDED_TIME,
        units: AngleUnits | None = None,
    ) -> bool:
        """Отправляет команду управления угловыми скоростями сочленений
        (joint-space velocity control).

        Метод задаёт целевые скорости для каждого звена робота на заданный временной шаг,
        позволяя управлять движением без предварительного планирования траектории.
        В отличие от `servoj`, команда не стремится к конкретной позиции, а поддерживает
        заданные скорости до получения новой команды или явной остановки.
        Идеально подходит для плавного jogging или генерации
        траекторий «на лету» в режиме реального времени.

        Метод недоступен в режиме «read only» (требует активной сессии управления).

        Args:
            target_velocities (PositionOrientation): Целевые угловые скорости сочленений (J1–J6).
                Ожидается последовательность или массив из 6 числовых значений.
            time (float, optional): Время шага дискретизации (в секундах).
                Определяет интервал, в течение которого контроллер будет поддерживать
                заданные скорости перед ожиданием следующей команды.
                По умолчанию: 0.002 c (рекомендуется 0.002–0.008 с).
            units (AngleUnits, optional): Единицы измерения скоростей.
                Допустимые значения:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
                Если `None`, используются системные единицы по умолчанию.

        Returns:
            bool: `True`, если команда успешно отправлена.

        Examples:
            >>> import time
            >>>
            >>> # 1. Профиль скоростей для 16 шагов
            ... # (разгон -> удержание -> торможение -> стоп)
            >>> velocity_profile = [
            ...     (0.00, 0.10, 0.00, 0.00, 0.00, 0.00),  # 1
            ...     (0.00, 0.25, 0.00, 0.00, 0.00, 0.00),  # 2
            ...     (0.00, 0.40, 0.00, 0.00, 0.00, 0.00),  # 3
            ...     (0.00, 0.50, 0.05, 0.00, 0.00, 0.00),  # 4
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 5
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 6
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 7
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 8
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 9
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 10
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 11
            ...     (0.00, 0.50, 0.10, 0.00, 0.00, 0.00),  # 12
            ...     (0.00, 0.40, 0.08, 0.00, 0.00, 0.00),  # 13
            ...     (0.00, 0.25, 0.05, 0.00, 0.00, 0.00),  # 14
            ...     (0.00, 0.10, 0.00, 0.00, 0.00, 0.00),  # 15
            ...     (0.00, 0.00, 0.00, 0.00, 0.00, 0.00)   # 16 (нулевая скорость)
            ... ]
            >>>
            >>> # 2. Цикл отправки с динамическими параметрами и контролем буфера
            >>> dt_base = 0.004
            >>> try:
            ...     for idx, vel in enumerate(velocity_profile, 1):
            ...         # Контроль заполненности RT-очереди
            ...         q = robot.motion.realtime.get_motion_queue_status()
            ...         if q and q.available_slots < 3:
            ...             time.sleep(0.005)
            ...
            ...         t0 = time.perf_counter()
            ...         ok = robot.motion.realtime.speedj(vel, time=dt_base, units="rad")
            ...         if not ok:
            ...             raise RuntimeError(f"Отказ отправки скорости на шаге {idx}")
            ...
            ...         # Выравнивание цикла реального времени
            ...         time.sleep(max(0, cur_dt - (time.perf_counter() - t0)))
            ... finally:
            ...     robot.motion.realtime.stopj()

        Notes:
            - Скорости **не накапливаются** и не интегрируются в позицию автоматически.
              Каждая команда переопределяет текущую уставку на следующем цикле контроллера.
            - Параметр `time` задаёт длительность выполнения команды.
            - В отличие от `servoj`, `speedj` не гарантирует достижение конкретной точки.
              Для точного позиционирования используйте `servoj`.
        """
        return self._send_joints_command(
            RealtimeCommands.REALTIME_SPEEDJ, target_velocities, time, units
        )

    @handle_connection(available_in_read_only=True)
    def get_motion_queue_status(self) -> MotionQueueStatus | None:
        """Возвращает текущее состояние буфера команд движения из RTD-потока.

        Метод предоставляет метаданные очереди реального времени, позволяющие
        контролировать заполненность буфера контроллера и отслеживать прогресс
        обработки траектории. Используется для динамической подстройки частоты
        отправки команд, предотвращения переполнения очереди и синхронизации
        внешнего планировщика с внутренним циклом робота.

        Метод доступен в режиме «read only».

        Returns:
            Optional[MotionQueueStatus]: Экземпляр структуры статуса или `None`,
            если не удалось получить данные. Поля возвращаемого объекта:
                - `available_slots` — количество свободных ячеек в очереди (0–16);
                - `received_count` — циклический счётчик успешно принятых точек
                  (по модулю 256);
                - `timestamp` — время получения данных `(сек, нсек)`.

        Examples:
            >>> import time
            >>>
            >>> # 1. Базовое чтение статуса перед отправкой пакета команд
            >>> status = robot.motion.realtime.get_motion_queue_status()
            >>> if status is None:
            ...     print("RTD-поток недоступен. Отправка приостановлена.")
            ... else:
            ...     print(f"Свободно слотов: {status.available_slots}")
            ...     print(f"Принято точек (цикл): {status.received_count}")
            ...
            >>> # 2. Адаптивный контроль частоты отправки
            >>> while is_running:
            ...     state = robot.motion.realtime.get_motion_queue_status()
            ...     if state is None or state.available_slots == 0:
            ...         # Буфер полон или поток потерян — ждём освобождения
            ...         time.sleep(0.002)
            ...         continue
            ...
            ...     # Отправляем следующую точку, если есть место
            ...     next_pose = trajectory_generator.next()
            ...     robot.motion.realtime.servoj(next_pose, time=0.004)
            ...
            >>> # 3. Детектирование потери пакетов через циклический счётчик
            >>> last_count = 0
            >>> for _ in range(100):
            ...     s = robot.motion.realtime.get_motion_queue_status()
            ...     if s:
            ...         # Проверяем разрыв (с учётом перехода через 255 -> 0)
            ...         delta = (s.received_count - last_count) % 256
            ...         if delta > 1 and delta < 250:
            ...             print(f"Предполагаемая потеря пакетов: пропущено {delta - 1}")
            ...         last_count = s.received_count
            ...     time.sleep(0.004)

        Notes:
            - Счётчик `received_count` является беззнаковым и циклическим (0–255).
              При переходе через 255 он сбрасывается в 0. Для вычисления разницы
              между замерами используйте формулу: `(new - old) % 256`.
            - Поле `timestamp` отражает момент формирования пакета на стороне
              контроллера. Используйте его для оценки задержек сети и
              синхронизации с внешними сенсорами.
            - Опрос статуса не блокирует основной поток и не генерирует трафик
              в направлении робота (только чтение из входящего RTD-потока).
              Рекомендуемая частота опроса: не реже 250 Гц для стабильного
              контроля в real-time сценариях.
        """
        if self._rtd_receiver is None or not self._rtd_receiver.is_connected():
            return None

        rtd_data = self._rtd_receiver.get_data()

        try:
            queue_avail = rtd_data.realtime_queue_avail
            waypoints_received = rtd_data.realtime_waypoints_received
            timestamp = (int(rtd_data.sec), int(rtd_data.nsec))
        except Exception as e:
            self._write_log(
                "warning", f"Failed to get realtime queue status, error: {e}"
            )
            return None

        return MotionQueueStatus(
            available_slots=max(int(queue_avail), 0),
            received_count=int(waypoints_received) % self._MAX_WAYPOINTS_COUNT,
            timestamp=timestamp,
        )

    @handle_connection(available_in_read_only=True)
    def wait_waypoints_received(
        self,
        base_received_count: int,
        expected_delta: int,
        timeout: float = 1.0,
    ) -> MotionQueueStatus | None:
        """Ожидает подтверждения контроллером о приёме заданного количества
        точек траектории.

        Метод блокирует выполнение до тех пор, пока циклический счётчик принятых
        точек в RTD-потоке не увеличится на `expected_delta` относительно
        `base_received_count`. Используется как механизм flow-control при
        пакетной отправке команд `servoj` или `speedj`: гарантирует, что
        контроллер физически обработал предыдущие уставки и освободил слоты
        в очереди перед отправкой новых данных.

        Метод доступен в режиме «read only».

        Args:
            base_received_count (int): Начальное значение счётчика принятых точек
                (обычно получается через `get_motion_queue_status().received_count`
                перед началом серии отправки).
            expected_delta (int): Ожидаемое количество подтверждённых точек.
                Должно быть `> 0`. Если указано `<= 0`, метод немедленно
                возвращает текущий статус без ожидания.
            timeout (float, optional): Максимальное время ожидания в секундах.
                Если подтверждение не получено в указанный промежуток,
                генерируется `ApiError`. По умолчанию: `1.0`.

        Returns:
            Optional[MotionQueueStatus]: Статус очереди в момент достижения
            ожидаемого значения счётчика. Возвращает `None`, если RTD-поток
            недоступен на момент вызова (до истечения таймаута).

        Examples:
            >>> # 1. Базовое использование: отправка 10 точек с подтверждением
            >>> status = robot.motion.realtime.get_motion_queue_status()
            >>> base = status.received_count
            >>> for i in range(10):
            ...     robot.motion.realtime.servoj(trajectory[i], time=0.004)
            ...
            >>> # Ждём, пока контроллер примет все 10 точек
            >>> final_status = robot.motion.realtime.wait_waypoints_received(
            ...     base_received_count=base,
            ...     expected_delta=10,
            ...     timeout=0.5
            ... )
            >>> if final_status:
            ...     print(f"Очередь подтверждена. Свободно слотов: {final_status.available_slots}")

            >>> # 2. Продвинутый паттерн: конвейерная отправка с back-pressure
            >>> BATCH_SIZE = 8
            >>> status = robot.motion.realtime.get_motion_queue_status()
            >>> base = status.received_count
            >>> for i in range(0, len(trajectory), BATCH_SIZE):
            ...     batch = trajectory[i : i + BATCH_SIZE]
            ...     for pose in batch:
            ...         robot.motion.realtime.servoj(pose, time=0.002)
            ...
            ...     # Ждём подтверждения и обновляем базовый счётчик
            ...     confirmed_status = robot.motion.realtime.wait_waypoints_received(base, BATCH_SIZE, timeout=1.0)
            ...     if confirmed_status is None:
            ...         raise RuntimeError("RTD-поток потерян во время ожидания подтверждения")
            ...     base = confirmed_status.received_count

        Notes:
            - Метод учитывает циклическую природу счётчика (модуль 256).
              Внутренняя логика автоматически обрабатывает переход через
              `255 -> 0` при расчёте дельты.
            - Блокировка реализована через опрос статуса с интервалом
              `0.002`. Метод не занимает CPU в активном цикле.
            - Возврат `None` возможен только если RTD-соединение прервалось
              *до* достижения дельты. В таком случае рекомендуется проверить
              состояние соединения и повторить запрос.
            - Таймаут следует выбирать с запасом: при потере пакетов или
              задержках в сети контроллер может обрабатывать очередь медленнее,
              чем планировщик отправляет команды.
        """
        if expected_delta <= 0:
            return self.get_motion_queue_status()

        if not math.isfinite(timeout):
            raise ArgValueError(str(timeout))
        Validation.value(timeout, (0, None), "timeout")

        deadline = time.monotonic() + timeout
        last_status: MotionQueueStatus | None = None
        poll_interval = self._RECOMMENDED_TIME

        while time.monotonic() < deadline:
            status = self.get_motion_queue_status()
            if status is not None:
                last_status = status
                delta = self._realtime_received_delta(
                    base_received_count, status.received_count
                )
                if delta >= expected_delta:
                    if self._log_realtime:
                        self._write_log(
                            "debug",
                            f"Acknowledged {expected_delta} waypoints: "
                            f"base={base_received_count}, current={status.received_count}, delta={delta}",
                        )
                    return status

            time.sleep(poll_interval)

        # Timeout reached
        last_count = last_status.received_count if last_status else None
        raise ApiError(
            f"Timed out after {timeout:.2f}s waiting for {expected_delta} waypoint acknowledgements. "
            f"Expected from {base_received_count}, last received counter: {last_count}"
        )

    @handle_connection(available_in_read_only=False)
    def set_servoj_params(self, gain: float, lookahead_time: float) -> bool:
        """Настраивает параметры сервоуправления для последующих вызовов `servoj`.

        Метод задаёт коэффициент усиления и время предсказания, которые определяют,
        как внутренний контроллер будет интерполировать и отслеживать целевые
        позиции сочленений в режиме реального времени. Изменения применяются
        глобально и действуют до следующего вызова данного метода или перезагрузки
        контроллера.

        Метод недоступен в режиме «read only» (требует активной сессии управления).

        Args:
            gain (float): Коэффициент усиления позиционного контура.
                Управляет жёсткостью слежения за целевой траекторией.
                Рекомендуемый диапазон: `300.0 – 2000.0`.
                - Высокие значения повышают точность и скорость отклика,
                  но могут вызвать высокочастотные колебания или перегрузку
                  приводов при превышении допустимых пределов.
                - Низкие значения обеспечивают плавность, но увеличивают
                  ошибку позиционирования при быстрых движениях.
            lookahead_time (float): Время предсказания (сглаживания) траектории
                в секундах. Определяет горизонт интерполяции, на который
                контроллер "заглядывает вперёд" для фильтрации шумов и
                сглаживания рывков.
                Рекомендуемый диапазон: `0.005 – 1.0` с.
                - Увеличение значения делает движение мягче, но добавляет
                  фазовый сдвиг (задержку отклика).
                - Уменьшение повышает отзывчивость, но может привести к
                  вибрациям на резких изменениях уставки.

        Returns:
            bool: `True`, если команда настройки успешно отправлена контроллеру;
                `False` в случае ошибки соединения или внутренней ошибки отправки.

        Examples:
            >>> # 1. Точное позиционирование (жёсткий контур, минимальное сглаживание)
            >>> robot.motion.realtime.set_servoj_params(gain=2000.0, lookahead_time=0.005)
            >>> for pose in precise_trajectory:
            ...     robot.motion.realtime.servoj(pose, time=0.002)

            >>> # 2. Плавное движение (мягкий контур)
            >>> robot.motion.realtime.set_servoj_params(gain=300.0, lookahead_time=0.05)
            >>> for pose in smooth_trajectory:
            ...     robot.motion.realtime.servoj(pose, time=0.004)

            >>> # 3. Динамическая подстройка во время работы
            >>> robot.motion.realtime.set_servoj_params(gain=1500.0, lookahead_time=0.01)
            >>> # ... выполнение основной фазы ...
            >>> # Переключение на мягкий режим перед взаимодействием
            >>> robot.motion.realtime.set_servoj_params(gain=400.0, lookahead_time=0.08)

        Notes:
            - Параметры действуют **глобально** для всех последующих вызовов `servoj`.
              Их изменение во время движения может вызвать мгновенный скачок
              ускорения, если новые значения резко отличаются от предыдущих.
            - Команда не сбрасывает текущую траекторию и не влияет на уже
              находящиеся в буфере точки. Изменения вступают в силу только
              для новых команд `servoj`, отправленных после вызова этого метода.
        """
        self._ensure_socket_connected()
        Validation.value(gain, (300, 2000), "gain")
        Validation.value(lookahead_time, (0.005, 1.0), "lookahead_time")

        return self._realtime_controller.send_command(
            RealtimeCommands.REALTIME_SET_SERVOJ_PARAMS,
            gain,
            lookahead_time,
        )

    def _send_joints_command(
        self,
        command: RealtimeCommands,
        values: PositionOrientation,
        time: float,
        units: AngleUnits | None = None,
    ) -> bool:
        """
        Отправить команду управления.
        """
        units = units or MOTION_SETUP.units
        Validation.length(values, POSITION_ORIENTATION_LENGTH)
        Validation.value(
            time, (self._RECOMMENDED_TIME, self._MAX_TIME), "time"
        )
        Validation.literal("angle", units)

        if units == "deg":
            values = degrees_to_radians(values)
        values = normalize_joint_angles(values, "rad")

        return self._realtime_controller.send_command(
            command,
            *values,
            time,
        )

    def _ensure_socket_connected(self) -> None:
        """
        Проверить подключение контроллера.
        """
        if not self._realtime_controller.is_connected():
            raise RuntimeMotionError("Realtime socket is not connected")

    @classmethod
    def _realtime_received_delta(cls, previous: int, current: int) -> int:
        return (current - previous) % cls._MAX_WAYPOINTS_COUNT
