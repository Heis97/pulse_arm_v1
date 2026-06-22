from __future__ import annotations

import logging
import time
from collections.abc import Callable
from threading import Lock, Thread, main_thread

from api.robot_api_rc5v15.API.source.core.exceptions.status_error import (
    ControllerFailureError,
    ControllerFaultError,
    EmergencyStopError,
    SafetyViolationError,
)
from api.robot_api_rc5v15.API.source.core.network import (
    RTDReceiver,
)
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.models.classes.data_classes.rtd_structure import RTD
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    InComingControllerState,
    InComingMotionMode,
    InComingSafetyStatus,
    MotionWarning,
    WristMode,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import ExcInfoType


class StateHandler(LoggerMixin):
    """
    Класс, реализующий проверку состояния контроллера робота.
    """

    _CYCLE_TIME: float = 0.01  # sec
    _rtd_receiver: RTDReceiver
    _logger: logging.Logger | None

    def __init__(
        self,
        rtd_receiver: RTDReceiver,
        ignore_errors: bool,
        logger: logging.Logger | None = None,
    ):
        """
        Создать новый объект класса.

        Args:
            rtd_receiver (RTDReceiver): объект класса RTDReceiver;
            ignore_errors (bool): флаг игнорирования ошибок контроллера;
            logger (optional, logging.Logger): если задан, то логи будут писаться.
        """
        self._rtd_receiver: RTDReceiver = rtd_receiver
        self._ignore_errors: bool = ignore_errors
        self._set_logger(logger)

        self._controller_state = -1
        self._motion_mode = -1
        self._safety_status = -1
        self._safety_warning = False
        self._controller_warning = False
        self._wrist_mode = -1

        self._rtd_receiver_snapshot: RTD = RTD()

        self._thread: Thread | None = None
        self._lock = Lock()

        self._is_running: bool = False

    def start(
        self,
        error_callback: Callable[[ExcInfoType], None] | None = None,
        thread_name: str = "StateHandler-loop",
    ):
        """
        Запустить цикл проверки состояния контроллера в отдельном потоке.

        Args:
            error_callback: функция, которая будет вызвана при возникновении
                недопустимого состояния контроллера или ошибки при приеме данных;
            thread_name: имя потока, в котором будет запущен цикл проверки
                состояния.
        """
        self._write_log("debug", "Starting state controller checker loop")
        self._thread = Thread(
            target=self._checking_loop,
            args=(error_callback,),
            daemon=False,
            name=thread_name,
        )
        self._thread.start()

    def stop(self):
        """
        Остановить цикл проверки состояния контроллера.
        """
        self._write_log("debug", "Stopping state controller checker loop")
        if self._thread is None:
            return
        with self._lock:
            self._is_running = False
        try:
            self._thread.join(2)
        except Exception:
            pass

    def is_running(self) -> bool:
        """
        Получить статус работы цикла приема данных.

        Returns:
            bool: True если цикл запущен.
        """
        with self._lock:
            is_running = self._is_running
        return is_running and self._rtd_receiver.is_connected()

    def _checking_loop(
        self, error_callback: Callable[[ExcInfoType], None] | None
    ):
        self._is_running = True
        self._write_log("debug", "State handler loop was started")

        try:
            while (
                self._is_running
                and self._rtd_receiver.is_connected()
                and main_thread().is_alive()
            ):
                try:
                    self._check_all()
                    time.sleep(self._CYCLE_TIME)
                except (
                    ControllerFailureError,
                    ControllerFaultError,
                    EmergencyStopError,
                    SafetyViolationError,
                ) as error:
                    if not self._ignore_errors:
                        raise error
                    time.sleep(self._CYCLE_TIME)

            self._is_running = False
            self._write_log("debug", "State handler loop was stopped")
        except Exception as error:
            self._write_log(
                "error", f"State handler loop failed with error: {error}"
            )
            self._write_log(
                "error",
                f"Current RTD structure: {self._rtd_receiver_snapshot}",
            )
            self._is_running = False
            if error_callback is not None:
                error_callback(error)

    def _check_all(self):
        self._check_controller_state()
        self._check_motion_mode()
        self._check_safety_status()
        self._check_wrist_mode()

    def _check_wrist_mode(self):
        self._rtd_receiver_snapshot = self._rtd_receiver.get_data()
        if self._rtd_receiver_snapshot.wrist_mode != self._wrist_mode:
            self._write_log(
                "info",
                f"Robot wrist-mode: "
                f"{WristMode(int(self._rtd_receiver_snapshot.wrist_mode)).name}",
            )
            self._wrist_mode = self._rtd_receiver_snapshot.wrist_mode

    def _check_motion_mode(self):
        self._rtd_receiver_snapshot = self._rtd_receiver.get_data()
        if self._rtd_receiver_snapshot.motion_mode != self._motion_mode:
            self._write_log(
                "info",
                f"Robot motion-mode: "
                f"{InComingMotionMode(int(self._rtd_receiver_snapshot.motion_mode)).name}",
            )
            self._motion_mode = self._rtd_receiver_snapshot.motion_mode
            if self._motion_mode == InComingMotionMode.pause:
                if self._rtd_receiver_snapshot.state_flags:
                    self._write_log(
                        "warning",
                        f"Robot motion warning: "
                        f"{MotionWarning(int(self._rtd_receiver_snapshot.state_flags)).name}",
                    )

    def _check_controller_state(self):
        self._rtd_receiver_snapshot = self._rtd_receiver.get_data()
        if int(self._rtd_receiver_snapshot.state) != self._controller_state:
            state = int(self._rtd_receiver_snapshot.state)
            if self._controller_state != -1:
                self._write_log(
                    "info",
                    f"Robot controller-state: "
                    f"{InComingControllerState(state).name}",
                )
            if state == InComingControllerState.failure:
                if self._controller_state == -1:
                    if self._controller_warning:
                        return
                    else:
                        self._controller_warning = True
                        return self._write_log(
                            "warning",
                            "Controller inner error happened. Controller reboot needed.",
                        )
                else:
                    raise ControllerFailureError
            if self._controller_warning:
                self._controller_warning = False
            self._controller_state = int(self._rtd_receiver_snapshot.state)

    def _check_safety_status(self):  # noqa: C901
        self._rtd_receiver_snapshot = self._rtd_receiver.get_data()
        if self._rtd_receiver_snapshot.safety != self._safety_status:
            status = int(self._rtd_receiver_snapshot.safety)
            if status == InComingSafetyStatus.emergency_stop:
                if self._safety_status == -1:
                    if self._safety_warning:
                        return
                    else:
                        self._safety_warning = True
                        return self._write_log(
                            "warning",
                            "Robot emergency stop (1-category stop event)",
                        )
                else:
                    raise EmergencyStopError
            elif status == InComingSafetyStatus.fault:
                if self._safety_status == -1:
                    if self._safety_warning:
                        return
                    else:
                        self._safety_warning = True
                        return self._write_log(
                            "warning",
                            "Robot software fault (0-category stop event)."
                            " Check core log-files",
                        )
                else:
                    raise ControllerFaultError
            elif status == InComingSafetyStatus.violation:
                if self._safety_status == -1:
                    if self._safety_warning:
                        return
                    else:
                        self._safety_warning = True
                        return self._write_log(
                            "warning",
                            "Robot safety violation (0-category stop event)",
                        )

                else:
                    raise SafetyViolationError
            if self._safety_warning:
                self._safety_warning = False
            self._write_log(
                "info",
                f"Robot safety-status: {InComingSafetyStatus(status).name}",
            )
            self._safety_status = self._rtd_receiver_snapshot.safety

    def __del__(self):
        if self.is_running():
            self.stop()
