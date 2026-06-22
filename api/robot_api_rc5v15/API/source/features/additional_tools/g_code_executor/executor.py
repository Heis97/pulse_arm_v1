from __future__ import annotations

import functools
import threading
import time
from collections.abc import Iterable
from pathlib import Path
from typing import Any, Callable, TextIO, overload

from API.rc_api import RobotApi
from api.robot_api_rc5v15.API.source.ap_interface.motion import CoordinateSystem
from api.robot_api_rc5v15.API.source.core.exceptions.base_api_error import ApiError
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin, set_logger
from api.robot_api_rc5v15.API.source.models.type_aliases import PowerUnits

from .dataclasses import GCodeCommand, GCodeExecutionContext, GCodeState
from .default_handlers import (
    handle_f,
    handle_g0_g1,
    handle_g2_g3,
    handle_g4,
    handle_g17,
    handle_g18,
    handle_g19,
    handle_g20,
    handle_g21,
    handle_g43_1,
    handle_g49,
    handle_g53,
    handle_g54,
    handle_g61,
    handle_g64,
    handle_g90,
    handle_g91,
    handle_m2,
    handle_m30,
    handle_m62,
    handle_m63,
    handle_m66,
    handle_m67,
)

GCodeHandlerType = Callable[
    [GCodeCommand, GCodeState, GCodeExecutionContext, Any], None
]


class GCodeExecutor(LoggerMixin):
    def __init__(
        self,
        robot: RobotApi,
        blend: float = 0.001,
        analog_outputs_units: PowerUnits = "V",
        use_joint_motion_for_g0: bool = False,
        min_buffer_size: int = 10,
        max_buffer_size: int = 50,
        interpolate_arcs: bool = False,
        interpolate_linear: bool = False,
        interpolation_step: float = 0.005,
        use_default_handlers: bool = True,
        **kwargs,
    ):
        """Инициализирует исполнитель G-кода.

        Настраивает параметры движения, буферизацию, интерполяцию и подключает
        стандартные обработчики команд (G0/G1/G2/G3, модальные коды, I/O).

        Args:
            robot: Экземпляр RobotApi целевого робота для отправки команд и чтения состояния.
            blend: Базовый допуск сглаживания траектории в метрах.
            analog_outputs_units: Единицы измерения для аналоговых выходов ("V" или "mA").
            use_joint_motion_for_g0: Использовать перемещение по углам сочленений для команды G0.
            min_buffer_size: Минимальное количество точек в очереди для активации backpressure.
            max_buffer_size: Максимальный размер очереди контроллера. При приближении к лимиту
                исполнитель приостанавливает генерацию новых точек.
            interpolate_arcs: Если `True`, дуги (G2/G3) аппроксимируются линейными сегментами
                вместо нативного `MoveC`.
            interpolate_linear: Если `True`, линейные перемещения разбиваются на мелкие отрезки.
            interpolation_step: Максимальная длина сегмента (в метрах) при включённой интерполяции.
            use_default_handlers: Если `True`, автоматически регистрирует встроенные обработчики
                (G0-G3, G17-G19, G20-G21, G90-G91, F, M-коды и т.д.).
            **kwargs: Дополнительные именные параметры:
                - enable_logger (bool, optional): Включить/выключить логирование
                    (в состоянии False последующие аргументы игнорируются).
                - enable_logfile (bool, optional): Включить/выключить
                    логирование в файл.
                - logger (logging.Logger, optional): Пользовательский логгер
                    (переопределяет предустановленные настройки логгера).
                - logfile_path (pathlib.Path, optional): Путь для размещения
                    файлов логирования (по умолчанию создает папку рядом с
                    исполняемым файлом).
                - logfile_name (pathlib.Path, optional): Имя лог-файла (по
                    умолчанию имя в формате '__ДД.ММ.ГГГГ__ЧЧ.00__.log').
                - logfile_level (int, optional): Уровень логирования в файл.
                - log_std_level (int, optional): Уровень консольного логирования.
                - show_std_traceback (bool, optional): Включить/выключить полное
                    отображение ошибок в консоли.
        Examples:
            >>> executor = GCodeExecutor(robot=api)
            >>> executor = GCodeExecutor(
            ...     robot=api,
            ...     blend=0.005,
            ...     interpolate_arcs=True,
            ...     interpolation_step=0.002,
            ...     use_default_handlers=True,
            ... )
        """
        self._set_logger(set_logger(**kwargs))

        self._ctx = GCodeExecutionContext(
            robot=robot,
            blend=blend,
            active_blend=blend,
            use_joint_motion_for_g0=use_joint_motion_for_g0,
            min_buffer_size=min_buffer_size,
            max_buffer_size=max_buffer_size,
            interpolate_arcs=interpolate_arcs,
            interpolate_linear=interpolate_linear,
            interpolation_step=interpolation_step,
            analog_outputs_units=analog_outputs_units,
            logger=self._get_logger(),
        )
        self._handlers: dict[str, GCodeHandlerType] = {}
        self._resume_signal: threading.Event = threading.Event()
        self._stop_signal: threading.Event = threading.Event()
        self._stop_signal.set()
        self._lock: threading.Lock = threading.Lock()

        if use_default_handlers:
            self._register_default_handlers()

    @property
    def current_line_number(self) -> int:
        """Текущий номер обрабатываемой строки.

        Возвращает порядковый номер строки в исходном файле или потоке.
        Обновляется интерпретатором на каждой итерации. Используется для
        логирования, отображения прогресса и отладки.

        Returns:
            Целое число >= 1. Равно `0` до начала выполнения.

        Examples:
            >>> print(executor.current_line_number)  # 42
        """
        with self._lock:
            return self._ctx.current_line_number

    @property
    def current_raw_line(self) -> str:
        """Исходный текст текущей строки.

        Возвращает необработанную строку G-кода, которая в данный момент
        обрабатывается или выполняется. Включает N-коды, комментарии и пробелы.

        Returns:
            Строка с сырым содержимым. Пустая строка до начала выполнения.

        Examples:
            >>> print(executor.current_raw_line)
            >>> # "N95 G2 X15. Y190.05 I15.05 J0."
        """
        with self._lock:
            return self._ctx.current_raw_line

    @overload
    def register(
        self,
        command: str | Iterable[str],
        *,
        handler: None = ...,
        wait_waypoint_completion_before: bool = ...,
        waiting_timeout: float = ...,
        start_motion_after: bool = ...,
        pause_before: float = ...,
        pause_after: float = ...,
    ) -> Callable[[GCodeHandlerType], GCodeHandlerType]: ...

    @overload
    def register(
        self,
        command: str | Iterable[str],
        handler: GCodeHandlerType,
        *,
        wait_waypoint_completion_before: bool = ...,
        waiting_timeout: float = ...,
        start_motion_after: bool = ...,
        pause_before: float = ...,
        pause_after: float = ...,
    ) -> GCodeHandlerType: ...

    def register(  # noqa: C901
        self,
        command: str | Iterable[str],
        handler: GCodeHandlerType | None = None,
        *,
        wait_waypoint_completion_before: bool = False,
        waiting_timeout: float = 200.0,
        start_motion_after: bool = False,
        pause_before: float = 0.0,
        pause_after: float = 0.0,
    ) -> Callable[[GCodeHandlerType], GCodeHandlerType] | GCodeHandlerType:
        """Регистрирует обработчик для одной или нескольких команд G-кода.

        Позволяет привязать пользовательскую функцию к командам (G, M, T, F и т.д.).
        Поддерживает использование как декоратор или как прямой метод. Обеспечивает
        тонкую настройку синхронизации с очередью движений контроллера. Регистр
        идентификаторов игнорируется.

        Args:
            command: Идентификатор команды или итерируемый объект с идентификаторами
                (например, `"G1"`, `["M62", "M63"]`).
            handler: Функция-обработчик с сигнатурой `(cmd, state, ctx, custom_ctx) -> None`.
                Если `None`, метод возвращает декоратор для использования с `@`.
            wait_waypoint_completion_before: Если `True`, блокирует выполнение до полной
                отработки всех точек в очереди контроллера перед вызовом обработчика.
            waiting_timeout: Максимальное время ожидания (в секундах) для операций
                синхронизации с очередью.
            start_motion_after: Если `True`, автоматически включает режим движения
                контроллера после выполнения обработчика.
            pause_before: Время паузы (в секундах) перед вызовом обработчика.
            pause_after: Время паузы (в секундах) после вызова обработчика.

        Returns:
            Декоратор для регистрации обработчика или зарегистрированную функцию, если `handler` передан явно.

        Examples:
            >>> @executor.register("G100")
            >>> def custom_move(cmd, state, ctx, custom_ctx):
            >>>     ...
            ...
            >>> executor.register(
            >>>     ["M62", "M63"],
            >>>     handler=my_io_handler,
            >>>     pause_before=0.5
            >>> )
        """

        if isinstance(command, str):
            cmds = [command.strip().upper()]
        else:
            cmds = [c.strip().upper() for c in command]

        def decorator(func: GCodeHandlerType) -> GCodeHandlerType:
            @functools.wraps(func)
            def wrapper(
                cmd: GCodeCommand,
                state: GCodeState,
                ctx: GCodeExecutionContext,
                custom_ctx: Any,
            ) -> None:
                self._ctx.robot.motion.wait_waypoint_completion(
                    waypoint_count=self._ctx.min_buffer_size,
                    await_sec=waiting_timeout,
                )

                cmd_upper = cmd.command.upper()

                if wait_waypoint_completion_before:
                    self._write_log(
                        "debug",
                        f"Waiting for full motion completion "
                        f"{waiting_timeout:.2f} sec for '{cmd_upper}'...",
                    )
                    if not self._ctx.robot.motion.wait_waypoint_completion(
                        await_sec=waiting_timeout
                    ):
                        self._write_log(
                            "error",
                            f"Waypoints was not executed in specified time "
                            f"({waiting_timeout:.2f} sec)",
                        )

                if pause_before:
                    self._write_log(
                        "debug",
                        f"Waiting for {pause_before:.1f} sec before '{cmd_upper}'...",
                    )
                    time.sleep(pause_before)

                result = func(cmd, state, ctx, custom_ctx)

                if pause_after:
                    self._write_log(
                        "debug",
                        f"Waiting for {pause_after:.1f} sec after '{cmd_upper}'...",
                    )
                    time.sleep(pause_after)

                if start_motion_after:
                    self._ctx.robot.motion.mode.set("move_adv")

                return result

            for cmd_upper in cmds:
                self._handlers[cmd_upper] = wrapper

            self._write_log(
                "debug",
                f"Registered G code handler: {', '.join(cmds)} -> {func.__name__}",
            )

            return wrapper

        if handler is not None:
            return decorator(handler)
        return decorator

    def unregister(self, command: str | Iterable[str]) -> int:
        """Отменяет регистрацию обработчиков для указанных команд.

        Удаляет привязку обработчиков из внутреннего реестра. Поддерживает
        удаление одной команды или группы команд за один вызов. Регистр
        идентификаторов игнорируется.

        Args:
            command: Идентификатор команды или итерируемый объект с идентификаторами
                (например, `"G07"`, `["M62", "M63"]`).

        Returns:
            Количество успешно удалённых обработчиков.

        Examples:
            >>> executor.unregister("G07")
            ... # Удаляет один обработчик
            >>> executor.unregister(["M0", "M1"])
            ... # Удаляет несколько обработчиков
        """
        cmds = (
            [command.strip().upper()]
            if isinstance(command, str)
            else [c.strip().upper() for c in command]
        )
        removed_count = 0

        for cmd in cmds:
            if cmd in self._handlers:
                del self._handlers[cmd]
                self._write_log("debug", f"Unregistered G code handler: {cmd}")
                removed_count += 1
            else:
                self._write_log(
                    "warning",
                    f"Handler for G code command '{cmd}' was not registered.",
                )

        return removed_count

    def get_registered_commands(
        self, prefix: str | None = None
    ) -> tuple[str, ...]:
        """Возвращает отсортированный список зарегистрированных команд.

        Позволяет получить перечень всех команд, для которых привязаны
        обработчики. Опционально поддерживает фильтрацию по буквенному префиксу.

        Args:
            prefix: Строка-фильтр. Если указана, возвращаются только команды,
                начинающиеся с этого префикса (регистр игнорируется).

        Returns:
            Отсортированный список строк с идентификаторами команд.

        Examples:
            >>> executor.get_registered_commands()
            ... # ('%', 'F', 'G0', 'G1', 'M3', ...)
            >>> executor.get_registered_commands("G")
            ... # ('G0', 'G1', 'G2', ...)
            >>> executor.get_registered_commands("M6")
            ... # ('M6', 'M62', 'M63', ...)
        """
        cmds = sorted(self._handlers.keys())
        if prefix:
            p = prefix.strip().upper()
            cmds = [c for c in cmds if c.startswith(p)]
        return tuple(cmds)

    def get_handler(self, command: str) -> tuple[str, GCodeHandlerType] | None:
        """Возвращает имя и ссылку на зарегистрированный обработчик.

        Позволяет проверить наличие привязки или получить метаданные функции
        для отладки, логирования или динамической замены.

        Args:
            command: Идентификатор команды (например, `"G1"`, `"M30"`).

        Returns:
            Кортеж `(имя_функции, обработчик)` или `None`, если команда
            не зарегистрирована.

        Examples:
            >>> executor.get_handler("G1")
            ... # ('handle_g0_g1', <function handle_g0_g1 at 0x...>)
            >>> executor.get_handler("G999")
            ... # None
        """
        handler = self._handlers.get(command.strip().upper())
        if not handler:
            return None
        return handler.__name__, handler

    def run(  # noqa: C901
        self,
        source: Path | str | Iterable[str],
        encoding: str = "utf-8",
        coordinate_system: CoordinateSystem | None = None,
        dry_run: bool = False,
        step_mode: bool = False,
        start_line: int = 1,
        max_lines: int | None = None,
        raise_on_handler_exception: bool = True,
        raise_on_unknown_command: bool = False,
        custom_context: Any | None = None,
    ) -> None:
        """Запускает выполнение G-кода из файла или итерируемого источника.

        Последовательно разбирает строки, обновляет модальное состояние,
        вызывает зарегистрированные обработчики и отправляет команды роботу.
        Поддерживает режимы отладки, ограничения по объёму и пользовательский контекст.

        Args:
            source: Путь к файлу, строка или итерируемый объект со строками G-кода.
            encoding: Кодировка исходного файла. Игнорируется для итерируемых источников.
            coordinate_system: Активная система координат (фрейм) для трансформации точек.
            dry_run: Если `True`, имитирует выполнение без вызовов API робота.
            step_mode: Если `True`, приостанавливает выполнение после каждой команды
                для пошаговой отладки.
            start_line: Номер строки, с которой начинается выполнение.
            max_lines: Максимальное количество обрабатываемых команд. `None` = без лимита.
            raise_on_handler_exception: Если `True`, исключения в обработчиках прерывают выполнение.
            raise_on_unknown_command: Если `True`, отсутствие обработчика вызывает ошибку.
                По умолчанию `False` (логируется предупреждение).
            custom_context: Произвольный объект, передаваемый во все обработчики как `custom_ctx`.

        Returns:
            `None`. Выполнение завершается при окончании источника, ошибке или сигнале остановки.

        Examples:
            >>> # Полное выполнение файла
            >>> executor.run("program.nc")
            ...
            >>> # Валидация файла без движения
            >>> executor.run("program.nc", dry_run=True)
            ...
            >>> # Частичный запуск
            >>> executor.run(lines, start_line=50, max_lines=200)
        """
        if step_mode:
            self._write_log("warning", "Step mode G code execution is active")

        self._stop_signal.clear()
        self._resume_signal.set()

        executed = 0
        state = GCodeState()

        self._ctx.coordinate_system = coordinate_system or CoordinateSystem(
            (0,) * 6
        )
        self._ctx.current_line_number = 0

        is_file = isinstance(source, (str, Path))
        source_iter = (
            open(source, encoding=encoding) if is_file else iter(source)
        )

        try:
            for line_num, raw_line in enumerate(source_iter, 1):
                raw_line = raw_line.strip()
                if self._stop_signal.is_set():
                    break
                if not self._resume_signal.is_set():
                    self._resume_signal.wait()

                self._ctx.current_line_number = line_num
                self._ctx.current_raw_line = raw_line

                cmd = GCodeCommand.from_raw(raw_line, line_number=line_num)
                if cmd is None:
                    continue

                if (
                    cmd.command.startswith(("T", "S", "F"))
                    and len(cmd.command) > 1
                ):
                    try:
                        val = float(cmd.command[1:])
                        cmd = GCodeCommand(
                            command=cmd.command[0],
                            params={cmd.command[0]: val, **cmd.params},
                            line_number=cmd.line_number,
                            raw=cmd.raw,
                        )
                    except ValueError:
                        pass

                state.update_modal(cmd)
                if line_num < start_line:
                    continue

                if max_lines and executed >= max_lines:
                    break

                if not cmd.command.startswith(("G", "M", "T", "S", "F")):
                    cmd = GCodeCommand(
                        command=state.last_motion_cmd,
                        params=cmd.params,
                        line_number=cmd.line_number,
                        raw=cmd.raw,
                    )

                handler = self._handlers.get(
                    cmd.command, self._handlers.get(cmd.command[0], None)
                )

                if handler:
                    try:
                        if dry_run:
                            executed += 1
                            continue
                        if step_mode:
                            self._write_log(
                                "info",
                                f"Paused at line {line_num}: {cmd.raw.strip()}",
                            )
                            input("Press Enter to continue...\n")
                        handler(cmd, state, self._ctx, custom_context)
                        executed += 1
                    except Exception as e:
                        self._write_log(
                            "error", f"Handler failed at line {line_num}: {e}"
                        )
                        if raise_on_handler_exception:
                            raise
                else:
                    self._write_log(
                        "warning",
                        f"Command {cmd.command} is unknown "
                        f"for {self.__class__.__name__}, line number: "
                        f"{line_num} full command: {raw_line}",
                    )
                    if raise_on_unknown_command:
                        raise ApiError(
                            f"Command {cmd.command} is unknown "
                            f"for {self.__class__.__name__}, line number: "
                            f"{line_num} full command: {raw_line}"
                        )
        finally:
            if is_file and isinstance(source_iter, TextIO):
                source_iter.close()
            self._stop_signal.set()
            self._resume_signal.clear()

    def pause(self) -> bool:
        """Приостанавливает обработку G-кода.

        Блокирует основной цикл обработки до вызова метода `resume()`.
        Безопасно вызывать из любого потока или обработчика интерфейса.
        Не останавливает робота.

        Returns:
            `True` при успешной отправке сигнала паузы.

        Examples:
            >>> executor.pause()
            ... # Остановить выполнение в текущей точке
        """
        self._resume_signal.clear()
        return True

    def resume(self) -> bool:
        """Возобновляет выполнение G-кода после паузы.

        Снимает блокировку основного цикла, позволяя интерпретатору
        продолжить разбор следующих команд.

        Returns:
            `True` при успешной отправке сигнала продолжения.

        Examples:
            >>> executor.resume()
            ... # Продолжить выполнение
        """
        self._resume_signal.set()
        return True

    def stop(self) -> bool:
        """Экстренно останавливает выполнение программы.

        Устанавливает флаг остановки, который прерывает цикл обработки
        на следующей проверке. Не очищает очередь контроллера автоматически.
        Не останавливает робота.

        Returns:
            `True` при успешной отправке сигнала остановки.

        Examples:
            >>> executor.stop()
            ... # Прервать выполнение и выйти из метода run()
        """
        self._stop_signal.set()
        return True

    def is_running(self) -> bool:
        """Проверяет, активен ли исполнитель G-кода.

        Возвращает `True`, если не установлен сигнал экстренной остановки.
        Состояние `True` означает, что цикл обработки выполняется.

        Returns:
            `True`, если цикл обработки команд запущен.

        Examples:
            >>> if executor.is_running():
            ...     print("Цикл обработки запущен")
        """
        return not self._stop_signal.is_set()

    def is_paused(self) -> bool:
        """Проверяет, находится ли исполнитель в состоянии паузы.

        Возвращает `True`, если выполнение временно приостановлено
        (вызван `pause()`), но не завершено окончательно (`stop()` не вызван).

        Returns:
            `True`, если активна пауза.

        Examples:
            >>> if executor.is_paused():
            ...     executor.resume()  # Возобновить выполнение
        """
        return (
            not self._stop_signal.is_set() and not self._resume_signal.is_set()
        )

    def _register_default_handlers(self):
        # Motion
        self.register(
            command=["G0", "G00", "G1", "G01"],
            handler=handle_g0_g1,
            start_motion_after=True,
        )
        self.register(
            command=["G2", "G02", "G3", "G03"],
            handler=handle_g2_g3,
            start_motion_after=True,
        )
        self.register(
            command=["G4", "G04"],
            handler=handle_g4,
            wait_waypoint_completion_before=True,
        )

        # Path control
        self.register("G61", handler=handle_g61)
        self.register("G64", handler=handle_g64)

        # Position and TCP
        self.register("G43.1", handler=handle_g43_1)
        self.register("G49", handler=handle_g49)

        # IO
        self.register(
            "M62", handler=handle_m62, wait_waypoint_completion_before=True
        )
        self.register(
            "M63", handler=handle_m63, wait_waypoint_completion_before=True
        )
        self.register(
            "M67", handler=handle_m67, wait_waypoint_completion_before=True
        )
        self.register(
            "M66",
            handler=handle_m66,
            wait_waypoint_completion_before=True,
            waiting_timeout=180.0,
        )

        # Config
        self.register(command="G17", handler=handle_g17)
        self.register(command="G18", handler=handle_g18)
        self.register(command="G19", handler=handle_g19)
        self.register(command="G20", handler=handle_g20)
        self.register(command="G21", handler=handle_g21)
        self.register(command="G90", handler=handle_g90)
        self.register(command="G91", handler=handle_g91)
        self.register(command="F", handler=handle_f)

        self.register("G53", handler=handle_g53)
        self.register(
            ["G54", "G55", "G56", "G57", "G58", "G59"], handler=handle_g54
        )

        self.register(
            ["M2", "M02"],
            handler=handle_m2,
            wait_waypoint_completion_before=True,
            waiting_timeout=500,
        )
        self.register(
            "M30",
            handler=handle_m30,
            wait_waypoint_completion_before=True,
            waiting_timeout=500,
        )
        self.register(command="%", handler=lambda *args: None)
