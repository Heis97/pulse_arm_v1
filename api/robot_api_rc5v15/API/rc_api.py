from __future__ import annotations

import atexit
import copy
import platform
import sys
import threading
import time
import traceback
from collections.abc import Callable, Generator
from contextlib import contextmanager

from api.robot_api_rc5v15.API.source.ap_interface.configuration import Configuration
from api.robot_api_rc5v15.API.source.ap_interface.controller import (
    ControllerGravity,
    ControllerState,
    RobotController,
)
from api.robot_api_rc5v15.API.source.ap_interface.diagnostics import (
    Diagnostics,
)
from api.robot_api_rc5v15.API.source.ap_interface.io import IO
from api.robot_api_rc5v15.API.source.ap_interface.motion import Motion
from api.robot_api_rc5v15.API.source.ap_interface.payload import PayLoad
from api.robot_api_rc5v15.API.source.ap_interface.safety import Safety, SafetyStatus
from api.robot_api_rc5v15.API.source.ap_interface.tool import Tool
from api.robot_api_rc5v15.API.source.ap_interface.wrist.wrist_host import Wrist
from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
    is_in_impulse_compat_mode,
)
from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import (
    ControllerUnlockError,
    ServerPingError,
)
from api.robot_api_rc5v15.API.source.core.network import (
    Controller,
    RealtimeController,
    RTDReceiver,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin, set_logger
from api.robot_api_rc5v15.API.source.features.state_handler import StateHandler
from api.robot_api_rc5v15.API.source.features.warnings import deprecated
from api.robot_api_rc5v15.API.source.models.classes.data_classes.api_version import (
    RobotInfo,
    Version,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.dh_models import DhParamsManager
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import IMPULSE_IP
from api.robot_api_rc5v15.API.source.models.type_aliases import ExcInfoType


class RobotApi15(LoggerMixin):
    _controller: Controller
    _rtd_receiver: RTDReceiver
    _state_handler: StateHandler
    _connection_state: ConnectionState

    tool: Tool
    """Подкласс для работы с инструментом."""
    payload: PayLoad
    """Подкласс для работы с полезной нагрузкой."""
    controller: RobotController
    """Подкласс для работы с контроллером робота (состояние, гравитация и т.д.)."""
    motion: Motion
    """Подкласс для управления движением."""
    io: IO
    """Подкласс для работы с входами/выходами контроллера."""
    wrist: Wrist
    """Подкласс для работы с платой запястья."""
    safety: Safety
    """Подкласс для работы с параметрами безопасности."""
    diagnostics: Diagnostics
    """Подкласс для работы с диагностической информацией."""
    configuration: Configuration
    """Подкласс для работы с конфигурацией робота."""

    @property
    @deprecated(
        "`RobotApi.controller_state` is deprecated.\n"
        "Use `robot.controller.state` instead.",
    )
    def controller_state(self) -> ControllerState:
        """Для обратной совместимости"""
        return self.controller.state

    @property
    @deprecated(
        "`RobotApi.controller_gravity` is deprecated.\n"
        "Use `robot.controller.gravity` instead.",
    )
    def controller_gravity(self) -> ControllerGravity:
        """Для обратной совместимости"""
        return self.controller.gravity

    @property
    @deprecated(
        "`RobotApi.safety_status` is deprecated.\n"
        "Use `robot.safety.status` instead.",
    )
    def safety_status(self) -> SafetyStatus:
        """Для обратной совместимости"""
        return self.safety.status

    def __init__(
        self,
        ip: str = "127.0.0.1",
        ignore_controller_exceptions: bool = False,
        read_only: bool = False,
        autoconnect: bool = True,
        allow_rtd_reconnection: bool = True,
        disable_realtime: bool = False,
        log_realtime: bool = True,
        timeout: int = 5,
        **kwargs,
    ) -> None:
        """Инициализирует клиент для управления роботом.

        Класс `RobotApi` — основная точка входа для взаимодействия с роботом.
        При создании экземпляра задаются параметры подключения, режима работы
        и логирования. По умолчанию выполняется автоматическое подключение
        к контроллеру с проверкой совместимости версий API и ядра управления.
        Поддерживает использование с контекстным менеджером `with`.

        В случае несоответствия версий подключение прерывается.
        Для решения проблемы обратитесь к официальному дистрибьютору продукции.

        Args:
            ip (str): IPv4-адрес робота без указания порта
                (например, "192.168.10.10").
            ignore_controller_exceptions (bool, optional): Флаг, позволяющий
                игнорировать ошибки обработчика состояний контроллера. При
                активации флага пользователям необходимо самостоятельно
                отслеживать состояние безопасности.
            read_only (bool, optional): Флаг работы API в режиме read_only,
                подключение при этом происходит только к порту RTD.
            autoconnect (bool, optional): Флаг, указывающий необходимость
                подключения к роботу по время создания экземпляра класса.
            allow_rtd_reconnection (bool, optional): Флаг, разрешающий
                автоматическое переподключение RTD контроллера к роботу.
                Рекомендуется разрешать переподключение в большинстве случаев,
                не рекомендуется при управлении в реальном времени и ряде других
                специфичных случаев.
            disable_realtime (bool, optional): Флаг, указывающий необходимость
                не подключать к роботу контроллер управления в реальном времени.
            log_realtime (bool, optional): Флаг, указывающий необходимость
                выводить отладочные сообщения при управлении в реальном времени.
            timeout (int, optional): Таймаут на подключение к роботу (сек).
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
            >>> # Стандартное подключение с логированием
            >>> import logging
            >>> robot = RobotApi(
            ...     '192.168.10.10',
            ...     enable_logger=True,
            ...     log_std_level=logging.DEBUG,
            ...     enable_logfile=True,
            ...     logfile_level=logging.INFO
            ... )

            >>> # Создание без автоматического подключения
            >>> robot = RobotApi('192.168.10.1', autoconnect=False)

            >>> # Использование с контекстным менеджером with
            >>> with RobotApi('192.168.10.1') as robot:
            ...     print("Робот подключен:", robot.is_connected())
            ...     # Какая-то логика
            ... # Подключение автоматически закроется

        Notes:
            - При `autoconnect=True` (по умолчанию) метод может выбросить
              исключение в случае ошибки подключения или несовместимости версий.
            - Если передан параметр `logger`, все остальные настройки логирования
              (`logfile_path`, `log_std_level` и т.д.) игнорируются.
            - В режиме `read_only=True` методы, изменяющие состояние робота,
              вызовут ошибку при попытке использования.
        """

        if is_in_impulse_compat_mode():
            self._ip: str = IMPULSE_IP
            self._write_log(
                "info",
                f"API running in Impulse compatible mode, IP was changed from {ip} to {self._ip}",
            )
        else:
            self._ip: str = ip
        self._ignore_controller_exceptions: bool = ignore_controller_exceptions
        self._read_only: bool = read_only
        self._autoconnect: bool = autoconnect
        self._allow_rtd_reconnection: bool = allow_rtd_reconnection
        self._disable_realtime: bool = disable_realtime
        self._log_realtime: bool = log_realtime
        self._timeout: int = timeout

        self._set_logger(set_logger(**kwargs))
        atexit.register(self._exit_program)

        self._write_log(
            "debug",
            f"Python: {platform.python_version()}, "
            f"platform: {platform.system()} {platform.release()}",
        )

        self._controller: Controller = Controller(
            ip=self._ip, timeout=self._timeout, logger=self._get_logger()
        )
        self._realtime_controller: RealtimeController = RealtimeController(
            ip=self._ip, timeout=self._timeout, logger=self._get_logger()
        )
        self._rtd_receiver: RTDReceiver = RTDReceiver(
            ip=self._ip,
            timeout=self._timeout,
            allow_reconnection=self._allow_rtd_reconnection,
            logger=self._get_logger(),
        )
        self._state_handler: StateHandler = StateHandler(
            rtd_receiver=self._rtd_receiver,
            logger=self._get_logger(),
            ignore_errors=self._ignore_controller_exceptions,
        )

        self._lock: threading.Lock = threading.Lock()

        self._disconnection_callback_was_called: bool = False
        self._disconnection_callback: Callable | None = None
        self._connection_state = ConnectionState(
            is_connected=False, is_read_only=True
        )

        self._init_subclasses()

        self._write_log(
            "info", f"Client version: [{RobotInfo.client_version}]"
        )
        if self._autoconnect:
            self.connect(read_only=read_only)

    @handle_connection(available_in_disconnected_state=True)
    def connect(
        self,
        read_only: bool = False,
    ) -> bool:
        """Выполняет подключение к контроллеру робота.

        Метод позволяет установить соединение с роботом в выбранном режиме и
        изменить режим подключения к роботу. Поддерживается как повторное
        подключение как после штатного отключения (через disconnect()), так и
        после потери связи из-за сетевых или аппаратных сбоев.

        При подключении выполняется проверка совместимости версий API и ядра
        управления робота. В случае несоответствия подключение отклоняется.

        Args:
            read_only (bool, optional): Если True, устанавливается подключение
                только к RTD-порту (режим "read only"). В этом режиме доступны
                методы получения состояния, но недоступны управляющие команды. По
                умолчанию False.

        Returns:
            bool: True — если подключение успешно установлено и версии совместимы;
                  False — если подключение не удалось (таймаут, недоступность,
                  несовместимость версий или попытка повторного подключения в
                  том-же режиме)

        Examples:
            >>> robot = RobotApi('192.168.10.1', autoconnect=False)
            >>> if robot.connect():
            ...     print("Подключено успешно")
            ...
            >>> # Подключение в режиме только для чтения
            >>> robot.connect(read_only=True)
            ...
            >>> # Смена режимов подключения
            >>> robot = RobotApi('192.168.10.1', autoconnect=False)
            >>> robot.connect(read_only=True)
            >>> robot.connect(read_only=False)
            >>> robot.connect(read_only=True)

        Notes:
            - Метод идемпотентен: повторный вызов при активном соединении
              возвращает True без побочных эффектов.
            - Если экземпляр был создан с read_only=True в конструкторе,
              значение read_only в этом методе является приоритетным при
              подключении.
            - При несовместимости версий рекомендуется обратиться
              к официальному дистрибьютору продукции.
        """
        if (
            self._connection_state.is_connected()
            and read_only == self._connection_state.is_read_only()
        ):
            self._write_log(
                "warning",
                f"Can't connect to Robot at [{self._ip}] - connection already establish in specified mode",
            )
            return False
        if self._connection_state.is_connected():
            if read_only:
                self._write_log(
                    "info",
                    f"Reconnecting to Robot at [{self._ip}] in read only mode",
                )
                return self._disconnect_control_socket()
            else:
                self._write_log(
                    "info",
                    f"Reconnecting to Robot at [{self._ip}] in full mode",
                )
                return self._connect_control_socket()

        if read_only:
            self._write_log(
                "info",
                f"Connecting to Robot at [{self._ip}] in read only mode",
            )
            if not self._connect_in_read_only_mode():
                self.disconnect()
                return False
            self._disconnection_callback_was_called = False
            self._connection_state.connect()
            self._connection_state.read_only()
            return True

        self._write_log("info", f"Connecting to Robot at [{self._ip}]")
        if not self._connect_in_full_mode(enable_state_handler=True):
            self.disconnect()
            return False
        self._disconnection_callback_was_called = False
        self._connection_state.connect()
        self._connection_state.full()
        return True

    @contextmanager
    @handle_connection(available_in_disconnected_state=True)
    def connected(
        self, read_only: bool = False
    ) -> Generator[RobotApi, None, None]:
        """Позволяет подключаться к роботу с помощью конструкции with.

        Метод позволяет установить соединение с роботом в выбранном режиме с
        помощью контекстного менеджера with. После использования возвращает
        подключение в то состояние, в котором оно было до применения метода
        (отключено, 'read only' или 'full').

        При подключении выполняется проверка совместимости версий API и ядра
        управления робота. В случае несоответствия подключение отклоняется.

        Args:
            read_only (bool, optional): Если True, устанавливается подключение
                только к RTD-порту (режим "read only"). В этом режиме доступны
                методы получения состояния, но недоступны управляющие команды. По
                умолчанию False.

        Yields:
            RobotApi

        Examples:
            >>> # Сначала необходимо создать экземпляр класса RobotApi
            >>> robot = RobotApi('192.168.10.1', autoconnect=False)
            ...
            >>> # Подключение в режиме только для чтения
            >>> with robot.connected(read_only=True):
            ...     print("Робот подключен:", robot.is_connected())
            ...
            >>> # Подключение в полноценном режиме
            >>> with robot.connected() as r:
            ...     print("Робот подключен:", r.is_connected())

        Notes:
            - Метод идемпотентен: повторный вызов при активном соединении
              возвращает True без побочных эффектов.
            - Если экземпляр был создан с read_only=True в конструкторе,
              значение read_only в этом методе является приоритетным при
              подключении.
            - При несовместимости версий рекомендуется обратиться
              к официальному дистрибьютору продукции.
        """

        initial_connection_state = copy.copy(self._connection_state)

        # Connection
        if not initial_connection_state.is_connected():
            self.connect(read_only=read_only)
        else:
            if initial_connection_state.is_read_only() != read_only:
                self.connect(read_only=read_only)

        try:
            yield self
        finally:
            # Restoring initial state
            if not initial_connection_state.is_connected():
                self.disconnect()
            else:
                if initial_connection_state.is_read_only() != read_only:
                    self.connect(
                        read_only=initial_connection_state.is_read_only()
                    )

    @handle_connection(available_in_disconnected_state=True)
    def is_connected(self) -> bool:
        """Получает актуальное состояние подключения к роботу.

        Метод проверяет, активно ли соединение с контроллером на момент вызова.
        Возвращает актуальный статус, даже если соединение было потеряно
        асинхронно (например, из-за сетевого сбоя или перезагрузки робота).

        Returns:
            bool: True — если подключение активно;
                  False — если соединение отсутствует, разорвано или не было
                  установлено.

         Examples:
            >>> robot = RobotApi('192.168.10.1', autoconnect=False)
            >>> print(robot.is_connected())  # False
            >>> robot.connect()
            >>> print(robot.is_connected())  # True

        Notes:
            - Метод не пытается восстановить соединение — он только проверяет
              текущее состояние.
        """
        with self._lock:
            return self._connection_state.is_connected()

    @handle_connection(available_in_disconnected_state=True)
    def disconnect(self) -> bool:
        """Выполняет штатное отключение от контроллера робота.

        Метод корректно завершает сетевое соединение с роботом,
        освобождает занятые ресурсы и переводит объект в состояние «отключено».
        Безопасен для повторного вызова: если соединение уже разорвано,
        метод не вызывает ошибку.

        Returns:
            bool: True — если отключение выполнено успешно или соединение
                  уже было разорвано.

        Examples:
            >>> robot = RobotApi('192.168.10.10')
            >>> robot.disconnect()

        Notes:
            - Метод является идемпотентным: его можно вызывать многократно
              без побочных эффектов.
            - Даже при возврате False объект считается отключённым — повторный
              вызов всё равно вернёт True.
            - Рекомендуется вызывать явно перед завершением работы программы
              или при переходе в неактивное состояние.
        """
        with self._lock:
            if self._connection_state.is_connected():
                self._shutdown()
                self._connection_state.disconnect()
        return True

    @handle_connection(available_in_read_only=False)
    def get_robot_info(self) -> RobotInfo:
        """Получает техническую информацию о модели робота и контроллере.

        Метод возвращает данные о версии аппаратного и программного обеспечения,
        включая модель робота, версию прошивки контроллера, версию клиентского API
        и параметры кинематической модели (DH-параметры).

        Метод недоступен в режиме read-only.

        Returns:
            RobotInfo: Объект dataclass, содержащий следующие поля:
                - robot_model (str): модель робота, например "rc10";
                - client_version (str): версия клиентского API и прошивки,
                  например "31.22.11.33555456/3";
                - dh_model (DhModelParams): параметры Денавита–Хартенберга
                  для текущей модели робота.
        Examples:
            >>> info = robot.get_robot_info()
            >>> print(f"Модель: {info.robot_model}")
            >>> print(f"Версия: {info.client_version}")
            ... # Вывод:
            ... # Модель: rc10b2
            ... # Версия: 31.22.11.33555456/3
        """
        if not self._controller.is_connected():
            self._write_log(
                "error", "Can't get robot info - controller is not connected"
            )
            return RobotInfo()

        robot_model = self._get_robot_model()
        if robot_model is None:
            return RobotInfo()

        return RobotInfo(
            robot_model=robot_model, dh_model=DhParamsManager.get(robot_model)
        )

    @handle_connection(available_in_read_only=False)
    def save(self) -> bool:
        """
        Сохраняет пользовательские настройки для работы робота после его
        перезапуска.

        Метод недоступен в режиме read-only.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> robot = RobotApi('192.168.10.10')
            >>> robot.save()

        Notes:
            - Успешный возврат `True` означает только отправку команды,
              а не подтверждение её выполнения контроллером.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error",
                "Can't save robot settings - controller is not connected",
            )
            return False
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_STORE_SETTINGS
        )

    def set_disconnection_callback(self, callback: Callable):
        """Устанавливает обработчик для события незапланированного обрыва соединения с роботом.

        Указанный обработчик будет вызван **только при нештатном разрыве связи**
        (например, сетевой сбой, отключение питания робота). Он **не вызывается**
        при штатном отключении через метод `disconnect()`.

        Перед вызовом обработчика API автоматически закрывает все соединения
        с контроллером и переводит объект в состояние «отключено».

        Args:
            callback (Callable[[], None]): Функция без параметров и без возвращаемого
                значения, которая будет выполнена при потере соединения.
                Должна завершаться за разумное время — API не накладывает таймаут,
                поэтому зависание в обработчике приведёт к полной остановке программы.

        Examples:
            >>> def handle_connection_failure() -> None:
            ...     print("Соединение с роботом потеряно!")
            ...     # Можно записать в лог, уведомить оператора и т.д.
            ...
            >>> robot.set_disconnection_callback(handle_connection_failure)

        Notes:
            - Обработчик вызывается после освобождения системных ресурсов.
            - Обработчик вызывается **синхронно** в том же потоке, где был
                обнаружен разрыв соединения (обычно — фоновый поток приема RDT).
            - Избегайте в обработчике:
                * блокирующих операций (input(), time.sleep() без ограничения),
                * долгих вычислений.
            - Если обработчик не задан, событие игнорируется.
        """
        self._write_log("info", "Robot disconnection callback was set")
        self._disconnection_callback = callback

    def _connect_control_socket(self) -> bool:
        """
        Подключиться по управляющему каналу к роботу, применять только после
        подключения в read_only моде или отключения управляющего сокета.

        Returns:
            bool: True если подключение выполнено успешно.
        """
        self._write_log(
            "debug", f"Connecting control socket for Robot at [{self._ip}]"
        )
        if self._connect_in_full_mode():
            self._connection_state.full()
            return True
        return False

    def _disconnect_control_socket(self) -> bool:
        """
        Отключиться от управляющего сокета робота - перейти в режим read_only.

        Returns:
            bool: True если отключение выполнено успешно.
        """
        self._write_log(
            "debug", f"Disconnecting control socket for Robot at [{self._ip}]"
        )
        self._connection_state.read_only()
        self._stop_state_handler()
        self._stop_realtime_controller()
        self._stop_controller()
        return True

    def _init_subclasses(self):
        """
        Инициализировать все подклассы API.
        """
        self.safety = Safety(
            rtd_receiver=self._rtd_receiver,
            controller=self._controller,
            connection_state=self._connection_state,
            logger=self._get_logger(),
        )
        self.motion = Motion(
            controller=self._controller,
            realtime_controller=self._realtime_controller,
            rtd_receiver=self._rtd_receiver,
            connection_state=self._connection_state,
            logger=self._get_logger(),
            log_realtime=self._log_realtime,
        )
        self.tool = Tool(
            controller=self._controller,
            connection_state=self._connection_state,
        )
        self.payload = PayLoad(
            controller=self._controller,
            connection_state=self._connection_state,
        )
        self.controller = RobotController(
            controller=self._controller,
            rtd_receiver=self._rtd_receiver,
            motion=self.motion,
            connection_state=self._connection_state,
            logger=self._get_logger(),
        )
        self.io = IO(
            controller=self._controller,
            rtd_receiver=self._rtd_receiver,
            connection_state=self._connection_state,
            logger=self._get_logger(),
        )
        self.wrist = Wrist(
            host=self._ip,
            controller=self._controller,
            rtd_receiver=self._rtd_receiver,
            connection_state=self._connection_state,
            logger=self._get_logger(),
        )
        self.diagnostics = Diagnostics(
            rtd_receiver=self._rtd_receiver,
            connection_state=self._connection_state,
        )
        self.configuration = Configuration(
            controller=self._controller,
            connection_state=self._connection_state,
        )

    def _connect_in_read_only_mode(self) -> bool:
        """
        Подключиться к роботу в режиме 'read only'.

        Returns:
            bool: True если подключение прошло успешно.
        """
        if self._rtd_receiver.is_connected():
            return True
        if not self._start_rtd_receiver():
            self._write_log("error", "Failed to connect RTD receiver")
            self._stop_rtd_receiver()
            return False
        self._stop_controller()
        self._stop_realtime_controller()

        return True

    def _connect_in_full_mode(self, enable_state_handler: bool = True) -> bool:
        """
        Подключиться к роботу в полноценном режиме.

        Returns:
            bool: True если подключение прошло успешно.
        """
        if not self._connect_in_read_only_mode():
            return False

        if not self._start_controller():
            self._write_log("error", "Failed to connect controller")
            self._stop_controller()
            return False

        if not self._start_realtime_controller():
            self._write_log("warning", "Failed to connect realtime controller")
            self._stop_realtime_controller()

        if not self._ping_loop(3):
            self._write_log("error", "Failed to connect to Robot")
            self._shutdown_api_sockets()
            return False

        self._validate_api_version()
        if enable_state_handler:
            self._start_state_handler()

        return True

    def _start_rtd_receiver(self) -> bool:
        """
        Запуск фонового потока получения метрик робота 'rt_data'.

        Returns:
            bool: True если приемник метрик робота инициализирован успешно.
        """
        if not self._rtd_receiver.connect():
            self._write_log(
                "error",
                "Can't initialize RTD receiver - initialization failed",
            )
            return False
        self._rtd_receiver.start_loop(
            error_callback=self._shutdown_from_thread
        )
        return True

    def _stop_rtd_receiver(self):
        """
        Остановить прием метрик робота.
        """
        if (
            not self._rtd_receiver.is_connected()
            and not self._rtd_receiver.is_running()
        ):
            return
        self._rtd_receiver.stop_loop()
        self._rtd_receiver.disconnect()

    def _start_controller(self) -> bool:
        """
        Запустить контроллер робота (установить соединение с контролирующем сокетом).

        Returns:
            bool: True если инициализация проведена успешно.
        """
        if (
            not self._controller.is_connected()
            and not self._controller.connect()
        ):
            return False
        return True

    def _stop_controller(self, graceful: bool = True):
        """
        Разорвать контролирующее соединение с роботом.
        """
        if self._controller.is_connected():
            if graceful:
                try:
                    self._controller.send_command(
                        ControllerCommands.CTRLR_COMS_CTRL_DISCONNECT
                    )
                except Exception:
                    pass
            self._controller.disconnect()

    def _start_realtime_controller(self) -> bool:
        """
        Запустить realtime контроллер робота (установить соединение с контролирующем сокетом).

        Returns:
            bool: True если инициализация проведена успешно.
        """
        if self._disable_realtime:
            return True
        if (
            not self._realtime_controller.is_connected()
            and not self._realtime_controller.connect()
        ):
            return False
        return True

    def _stop_realtime_controller(self):
        """
        Разорвать realtime соединение с роботом.
        """
        if self._disable_realtime:
            return

        if self._realtime_controller.is_connected():
            self._realtime_controller.disconnect()

    def _start_state_handler(self) -> bool:
        """
        Инициализация фонового потока обработки состояний робота.
        Поток проверяет и логирует в 'DEBUG' режиме текущие состояния
        контроллера, безопасности и типа движения. При получении критического
        статуса поток закрывает исполнение программы в аварийном режиме.

        Returns:
            bool: True если обработчик состояний инициализирован успешно.
        """
        if self._state_handler.is_running():
            return True
        if not self._rtd_receiver.is_connected():
            self._write_log(
                "error",
                "Can't initialize state handler - RTD receiver is not connected",
            )
            return False
        self._state_handler.start(error_callback=self._shutdown_from_thread)
        return True

    def _stop_state_handler(self):
        """
        Отключить фоновый поток обработки состояний робота.
        """
        if not self._state_handler.is_running():
            return
        self._state_handler.stop()

    def _exit_program(
        self,
        exc_type=None,
        exc_value=None,
        exc_traceback=None,
    ):
        """
        Вызывается при любой ошибке главного потока и при завершении всей
        программы.

        Args:
            exc_type: Тип ошибки.
            exc_value: Значение ошибки.
            exc_traceback: Содержание ошибки и хвост.
        """
        if not self._connection_state.is_connected():
            return
        self._shutdown(graceful=exc_type is None)

    def _shutdown(self, graceful: bool = True):
        """
        Основная логика отключения от Робота в главном потоке.
        """
        self._write_log("info", f"Disconnecting from Robot at [{self._ip}]")
        if not self._connection_state.is_connected():
            return
        self._shutdown_api_sockets(graceful=graceful)
        self._connection_state.disconnect()

    def _shutdown_from_thread(self, exc_info: ExcInfoType = None):
        """
        Основная логика отключения от Робота из не главного потока. При передаче
        информации об ошибке отключение происходит в аварийном режиме.

        Args:
            exc_info: ExcInfoType - информация об ошибке.
        """
        self._write_log("info", f"Disconnecting from Robot at [{self._ip}]")
        if not self._connection_state.is_connected():
            return
        if exc_info is not None:
            self._handle_exception(exc_info)

        graceful = exc_info is None
        self._shutdown_api_sockets(graceful=graceful)
        self._connection_state.disconnect()
        if (
            self._disconnection_callback is not None
            and not self._disconnection_callback_was_called
        ):
            self._write_log("info", "Calling disconnection callback")
            self._disconnection_callback_was_called = True
            self._disconnection_callback()

    def _handle_exception(self, exc_info: ExcInfoType):
        """
        Обработать полученное исключение.
        Args:
            exc_info: ExcInfoType - сообщение об ошибке.
        """
        if isinstance(exc_info, BaseException):
            exc_info = (
                type(exc_info),
                exc_info,
                exc_info.__traceback__,
            )
        elif not isinstance(exc_info, tuple):
            exc_info = sys.exc_info()

        logger = self._get_logger()
        if logger is not None:
            logger.exception(
                "Uncaught exception:", exc_info[1], exc_info=exc_info
            )
        else:
            print("Uncaught exception:")
            traceback.print_exception(exc_info[0], exc_info[1], exc_info[2])
        self._write_log("warning", "API connection emergency shutdown")

    def _shutdown_api_sockets(self, graceful: bool = True):
        """
        Отчистить все сетевые ресурсы и связанные с ними классы.
        """
        self._write_log(
            "debug",
            f"Shutdown of api sockets {'gracefully' if graceful else ''}",
        )
        self._stop_state_handler()
        self._stop_realtime_controller()
        self._stop_controller(graceful=graceful)
        self._stop_rtd_receiver()

    def _validate_api_version(self):
        """
        Проверить совместимость версии API с версией ядра.
        """
        if (
            server := self._get_server_version()
        ) == Version().get_full_version():
            self._write_log("info", "API is fully compatible with Controller")
        else:
            self._write_log(
                "warning",
                f"Client: [{RobotInfo.client_version}] != Server: [{server}]",
            )
            server_proto_version = self._get_server_proto_version()
            if server_proto_version != Version.proto_version:
                self._write_log(
                    "error",
                    f"API is fully incompatible with Controller, "
                    f"API proto version: {hex(Version.proto_version)}, "
                    f"controller: {hex(server_proto_version or 0)}",
                )
                raise ControllerUnlockError(
                    f"API is fully incompatible with Controller, "
                    f"API proto version: {hex(Version.proto_version)}, "
                    f"controller: {hex(server_proto_version or 0)}",
                )

        if not self._unlock_connection(Version.proto_version):
            self._shutdown_api_sockets()
            raise ControllerUnlockError("Most probably server is busy")
        self._write_log("info", "Successfully authorized in Robot")

    def _unlock_connection(self, proto_version: int) -> bool:
        """
        Отправить команду контроллеру на открытие соединения.

        Returns:
            bool: True если соединение было открыто.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error",
                "Can't unlock connection - controller is not connected",
            )
            return False
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_UNLOCK, proto_version
        )
        if not response or response[0] == 0:
            self._write_log(
                "error", f"Failed to unlock connection, response: {response}"
            )
            return False
        return True

    def _ping_loop(self, iterations: int) -> bool:
        """
        Пинг-команда. Высылается циклически.
        Так как сервер не уведомляет пользователя об
        отказе на соединение, для корректного уведомления пользователя, после
        первичного (обычно успешного) подключения к сокету, высылается данная
        команда в течении некоторого времени.

        Args:
            iterations: Количество отправленных команд.

        Returns:
            bool:
                True — в случае корректно работающего соединения с сервером.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error", "Can't run ping loop - controller is not connected"
            )
            return False
        try:
            for _ in range(iterations):
                self._controller.request(
                    ControllerCommands.CTRLR_COMS_GET_PROTO_VERSION
                )
                time.sleep(0.02)
        except Exception as e:
            self._shutdown_api_sockets()
            raise ServerPingError(
                f"Failed to ping server, probably, access denied; error: {e}"
            )
        return True

    def _get_server_version(self) -> str:
        """
        Получить версию сервера контроллера в строковом представлении.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error", "Can't run ping loop - controller is not connected"
            )
            return ""
        core_version = self._get_server_core_version()
        proto_version = self._get_server_proto_version()
        if core_version is None or proto_version is None:
            self._write_log("error", "Failed to get server version")
            return ""
        return Version(
            core_version=core_version,
            proto_version=proto_version,
        ).get_full_version()

    def _get_server_proto_version(self) -> int | None:
        """
        Получить версию 'rt_data' протокола робота.

        Returns:
            str: Версия 'rt_data' протокола.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error", "Can't run ping loop - controller is not connected"
            )
            return None
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_PROTO_VERSION
        )
        return response[0] if response else None

    def _get_server_core_version(self) -> str | None:
        """
        Получить версию ядра контроллера робота.

        Returns:
            str: Версия ядра контроллера робота.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error", "Can't run ping loop - controller is not connected"
            )
            return None
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_SW_VERSION
        )
        return (
            "".join([str(item) + "." for item in response])[:-1]
            if response
            else None
        )

    def _get_robot_model(self) -> str | None:
        """
        Получить название модели робота.

        Returns:
            str: Модель робота.
        """
        if not self._controller.is_connected():
            self._write_log(
                "error", "Can't run ping loop - controller is not connected"
            )
            return None
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_ROBOT_VIEW_INFO
        )
        return (
            "".join(
                [
                    item.decode("utf-8") if item != b"\x00" else ""
                    for item in response[1:]
                ]
            )
            if response
            else None
        )

    def __enter__(self):
        """
        Служебный метод для использования RobotAPI с контекстным
        менеджером with.
        """
        if not self._connection_state.is_connected() and self._autoconnect:
            self.connect(read_only=self._read_only)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Служебный метод для использования RobotAPI с контекстным
        менеджером with.
        """
        self._api_shutdown_was_called = False
        self._exit_program(exc_type, exc_val, exc_tb)

    def __del__(self):
        if self.is_connected():
            self.disconnect()
