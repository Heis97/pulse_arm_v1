import tkinter as tk
from collections.abc import Callable


class ClipboardWatcher:
    """
    Класс для отслеживания изменений в буфере обмена.
    """

    def __init__(
        self,
        parent: tk.Tk,
        callback: Callable[[str], None],
        check_interval: int = 100,
    ):
        self._parent = parent
        self._callback = callback
        self._check_interval = check_interval

        self._last_content = ""
        self._after_id = None
        self._is_running: bool = False

    def start(self):
        """
        Запустить отслеживание.
        """
        self._is_running = True
        self._check_clipboard()

    def stop(self):
        """
        Остановить отслеживание.
        """
        if self._is_running:
            if self._after_id:
                self._parent.after_cancel(self._after_id)
                self._after_id = None
            self._is_running = False

    def _check_clipboard(self):
        if not self._is_running:
            return
        try:
            current = self._parent.clipboard_get()
        except tk.TclError:
            current = ""

        if current != self._last_content:
            self._last_content = current
            self._callback(current)

        self._parent.after(self._check_interval, self._check_clipboard)
