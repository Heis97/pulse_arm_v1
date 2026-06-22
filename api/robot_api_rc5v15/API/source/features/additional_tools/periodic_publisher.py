from __future__ import annotations

import concurrent.futures
import logging
import threading
import time
from collections.abc import Callable
from typing import Any

from ..logger import LoggerMixin


class PeriodicPublisher(LoggerMixin):
    """Периодический издатель результатов вычислений.

    Запускает указанную функцию `func` в фоновом потоке с заданным
    интервалом и передаёт возвращаемое значение в `callback`.

    Особенности:
    - Работает в отдельном `daemon`-потоке, не блокируя основной поток приложения.
    - Автоматически управляет жизненным циклом (`start`/`stop`) и корректно
      завершает работу, дожидаясь окончания текущего тика.
    - Поддерживает синхронный и неблокирующий режим вызова callback.
    - Безопасно перехватывает и логирует исключения из `func` и `callback`,
      предотвращая аварийное завершение фонового потока.

    Example:
        >>> def read_sensor(channel: int) -> float:
        ...     return time.time() + channel
        ...
        >>> publisher = PeriodicPublisher(
        ...     func=read_sensor,
        ...     callback=print,
        ...     interval=1.0,
        ...     channel=5
        ... )
        >>> publisher.start()
        >>> # ... через некоторое время ...
        >>> publisher.stop()
    """

    def __init__(
        self,
        func: Callable[[], Any],
        callback: Callable[[Any], None] | None,
        interval: float = 1.0,
        logger: logging.Logger | None = None,
        non_blocking: bool = False,
        max_callback_workers: int = 1,
        *func_args,
        **func_kwargs,
    ):
        """Инициализирует периодический издатель.

        Args:
            func: Функция-источник данных. Будет вызываться в фоновом потоке
                с интервалом `interval`. Должна возвращать значение любого типа.
                Аргументы фиксируются при создании экземпляра и передаются при
                каждом вызове через `*func_args` и `**func_kwargs`.
            callback: Функция-обработчик результата. Принимает ровно один
                позиционный аргумент — результат выполнения `func`.
            interval: Интервал между запусками `func` в секундах.
                Фактический период = `interval` + время выполнения `func`.
                Если `func` выполняется дольше `interval`, следующий тик
                начнётся сразу после завершения (отставание не накапливается).
            logger: Экземпляр `logging.Logger` для внутреннего логирования.
                Если `None`, используется логгер по умолчанию из `LoggerMixin`.
            non_blocking: Режим выполнения `callback`.
                - `False` (по умолчанию): `callback` выполняется синхронно
                  в потоке издателя. Блокирует следующий тик до завершения обработки.
                - `True`: `callback` отправляется в `ThreadPoolExecutor`.
                  Цикл опроса не ждёт завершения обработки результата.
            max_callback_workers: Максимальное количество потоков в пуле
                для `callback`. Учитывается только при `non_blocking=True`.
                Значение `1` гарантирует порядок вызовов (FIFO). Увеличение
                до `>1` позволяет обрабатывать результаты параллельно.
            *func_args: Позиционные аргументы, передаваемые в `func`.
            **func_kwargs: Именованные аргументы, передаваемые в `func`.

        Notes:
            - Аргументы `func` фиксируются на момент создания экземпляра.
              Для динамического изменения параметров используйте метод-сеттер
              или перезапускайте издатель с новыми аргументами.
            - При `non_blocking=True` очередь задач не ограничена. Если
              `callback` выполняется медленнее, чем генерируются результаты,
              возможна растущая нагрузка на память (backpressure не реализован).
            - Поток издателя помечен как `daemon=True` и не блокирует
              завершение Python-процесса.
        """

        self._func = func
        self._callback = callback
        self._interval = interval
        self._set_logger(logger)
        self._func_args = func_args
        self._func_kwargs = func_kwargs

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False

        # Настройка неблокирующего режима
        self._non_blocking = non_blocking
        if self._non_blocking:
            self._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=max_callback_workers,
                thread_name_prefix="PeriodicPublisher-CB",
            )
        else:
            self._executor = None

    def start(self) -> None:
        """Запускает периодический цикл вызова `func` в фоновом потоке.

        Метод идемпотентен: если издатель уже запущен, повторный вызов
        игнорируется. Создаёт и стартует daemon-поток, который будет
        вызывать `func` с заданным интервалом и передавать результат
        в `callback`.

        Notes:
            - Поток помечен как `daemon=True` и не блокирует завершение
              процесса Python.
            - Метод не блокирует вызывающий поток и возвращается сразу
              после постановки задачи в scheduler ОС.
            - Аргументы `func` фиксируются при создании экземпляра.
              Для изменения параметров создайте новый экземпляр.
        """
        if self._running:
            return
        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="PeriodicPublisher"
        )
        self._thread.start()
        self._write_log(
            "debug",
            f"PeriodicPublisher started (interval={self._interval}s, non_blocking={self._non_blocking})",
        )

    def stop(self) -> None:
        """Останавливает периодический цикл и освобождает ресурсы.

        Метод идемпотентен: если издатель не запущен, вызов игнорируется.

        Notes:
            - Ожидание завершения фонового потока ограничено таймаутом
              `interval + 1.0` секунд. Если поток не завершается за это время
              (например, завис в тяжёлом `callback`), `join()` отработает
              по таймауту, и поток продолжит работу как daemon-поток до
              завершения процесса Python.
            - При `non_blocking=True` ожидающие в очереди задачи колбеков
              будут отменены, а уже выполняющиеся в момент вызова `stop()`
              будут доведены до конца.
            - После успешного вызова `stop()` экземпляр полностью готов
              к повторному запуску через `start()` (ресурсы переинициализируются).
        """
        if not self._running:
            return
        self._stop_event.set()
        self._running = False
        if self._thread:
            self._thread.join(timeout=self._interval + 1.0)

        # Graceful shutdown
        if self._executor:
            self._executor.shutdown(wait=True, cancel_futures=True)
            self._executor = None
        self._write_log("debug", "PeriodicPublisher was stopped")

    @property
    def is_running(self) -> bool:
        """Возвращает текущий статус работы издателя.

        Returns:
            bool: `True`, если фоновый цикл запущен и выполняет periodic calls;
                `False`, если издатель остановлен или ещё не был запущен.

        Note:
            - Свойство безопасно для чтения из любого потока.
        """
        return self._running

    def _safe_callback(self, result: Any) -> None:
        """Обёртка для перехвата исключений из колбека (фоновые потоки глотают ошибки)."""
        try:
            if self._callback:
                self._callback(result)
        except Exception as exc:
            self._write_log("error", f"Error in callback: {exc}")

    def _run(self) -> None:
        while not self._stop_event.is_set():
            start_time = time.monotonic()
            try:
                result = self._func(*self._func_args, **self._func_kwargs)
                if self._non_blocking and self._executor:
                    self._executor.submit(self._safe_callback, result)
                else:
                    self._safe_callback(result)
            except Exception as exc:
                self._write_log("error", f"Error in periodic function: {exc}")

            elapsed = time.monotonic() - start_time
            wait_time = max(0.0, self._interval - elapsed)
            self._stop_event.wait(wait_time)
