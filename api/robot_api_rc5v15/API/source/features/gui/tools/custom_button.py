from __future__ import annotations

from tkinter import Button, Event, Frame
from typing import Any
from collections.abc import Callable


class PixelButton(Frame):
    """
    Кнопка с фиксированным размером в пикселях.
    Внешне ведёт себя как Button, но размер задаётся точно.
    """

    def __init__(
        self,
        parent,
        width: int = 0,
        height: int = 0,
        text: str = "",
        fg: str | None = None,
        bg: str | None = None,
        pressed_fg: str | None = None,
        pressed_bg: str | None = None,
        hover_bg: str | None = None,
        font: Any = None,
        repeat_sig: bool = False,
        checkable: bool = False,
        checked: bool = False,
        **kwargs,
    ):
        super().__init__(
            parent,
            width=width,
            height=height,
            **kwargs,
        )
        self.pack_propagate(False)
        self.grid_propagate(False)

        self._fg = fg
        self._bg = bg
        self._original_bg = bg
        self._pressed_fg = pressed_fg
        self._pressed_bg = pressed_bg
        self._hover_bg = hover_bg or self._lighten_color(self._pressed_bg, 0.5)

        self._repeat_sig = repeat_sig
        self._checkable = checkable
        self._checked = checked
        self._pressed = False
        self._hovered = False
        self._press_func = None
        self._release_func = None

        self._button = Button(
            self,
            text=text,
            font=font,
            relief="flat",
            borderwidth=0,
            highlightthickness=0,
        )
        self._button.pack(expand=True, fill="both")
        self.config(
            highlightthickness=1,
            highlightbackground="gray",
            highlightcolor="gray",
        )
        self._original_bg = self._button["bg"]
        self._update_appearance(checked)

        self._button.bind("<ButtonPress-1>", self._on_press)
        self._button.bind("<ButtonRelease-1>", self._on_release)
        self._button.bind("<Enter>", self._on_enter)
        self._button.bind("<Leave>", self._on_leave)

    def _on_press(self, _: Event) -> None:
        if self._button["state"] == "disabled":
            return

        if self._checkable:
            self._checked = not self._checked
            self._update_appearance(self._checked)
            if self._checked:
                self._generate_signal()
            else:
                if self._release_func:
                    self._release_func()
            self.update_idletasks()
        else:
            if not self._pressed:
                self._pressed = True
                self._update_appearance(True)
                self._generate_signal()

    def _on_release(self, _: Event) -> None:
        if self._button["state"] == "disabled":
            return

        if not self._checkable and self._pressed:
            self._pressed = False
            self._update_appearance(False)
            if self._release_func:
                self._release_func()

    def _on_enter(self, event):
        self._hovered = True
        if not self._checked:
            self._update_appearance(False)

    def _on_leave(self, _: Event) -> None:
        self._hovered = False
        if not self._checked:
            self._update_appearance(False)
        if not self._checkable and self._pressed:
            self._pressed = False
            self._update_appearance(False)

    def _generate_signal(self) -> None:
        if self._button["state"] == "disabled":
            return
        if self._press_func:
            self._press_func()
        if self._repeat_sig and (
            self._checkable and self._checked or self._pressed
        ):
            self._repeat_job = self.after(25, self._generate_signal)

    def _update_appearance(self, is_pressed: bool):
        fg = self._pressed_fg if is_pressed else self._fg
        bg = self._pressed_bg if is_pressed else self._original_bg
        if self._hovered and not is_pressed:
            fg = self._fg
            bg = self._hover_bg
        self._button.config(
            fg=fg,  # type: ignore
            bg=bg,  # type: ignore
            activeforeground=fg,  # type: ignore
            activebackground=bg,  # type: ignore
        )

    def _lighten_color(
        self, color: str | None, factor: float = 0.1
    ) -> str | None:
        if not color:
            return None
        color = color.lstrip("#")
        r, g, b = tuple(int(color[i : i + 2], 16) for i in (0, 2, 4))
        r = min(255, int(r + (255 - r) * factor))
        g = min(255, int(g + (255 - g) * factor))
        b = min(255, int(b + (255 - b) * factor))
        return f"#{r:02x}{g:02x}{b:02x}"

    # === Публичный API ===
    def on_press(self, func: Callable):
        self._press_func = func

    def on_release(self, func: Callable):
        self._release_func = func

    def disconnect(self) -> None:
        self._press_func = None
        self._release_func = None

    def is_pressed(self) -> bool:
        return self._pressed

    def is_checked(self) -> bool:
        return self._checked

    def set_checked(self, checked: bool):
        self._checked = checked
        self._update_appearance(False)

    def config(self, **kwargs):
        if "text" in kwargs:
            self._button.config(text=kwargs.pop("text"))
        if "fg" in kwargs:
            self._button.config(fg=kwargs.pop("fg"))
        if "font" in kwargs:
            self._button.config(font=kwargs.pop("font"))
        if "checkable" in kwargs:
            param = kwargs.pop("checkable")
            self._checkable = param
            if self._checked:
                self._checked = False
                self._update_appearance(False)
        super().config(**kwargs)
