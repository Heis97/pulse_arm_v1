import tkinter as tk
from dataclasses import dataclass

from .custom_button import PixelButton


@dataclass
class CoordinateEntryGroup:
    entries: tuple[tk.Entry, ...] = None  # type: ignore
    move_button: PixelButton = None  # type: ignore
    paste_button: PixelButton = None  # type: ignore
    clear_button: PixelButton = None  # type: ignore


@dataclass
class RTDGroup:
    labels: tuple[tk.Label, ...] = None  # type: ignore
    copy_button: PixelButton = None  # type: ignore
