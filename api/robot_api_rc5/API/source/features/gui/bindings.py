from dataclasses import dataclass
from tkinter import Entry, Label

from source.features.gui.custom_button import _CustomButton


BACKSPACE = 8


class Sequences:
    empty: set = set()

    j0_max: set = {49}      # 0 (positive direction)
    j0_min: set = {49, 32}  # 0 space (negative direction)
    j1_max: set = {50}      # 1
    j1_min: set = {50, 32}  # 1 space
    j2_max: set = {51}      # 2
    j2_min: set = {51, 32}  # 2 space
    j3_max: set = {52}      # 3
    j3_min: set = {52, 32}  # 3 space
    j4_max: set = {53}      # 4
    j4_min: set = {53, 32}  # 4 space
    j5_max: set = {54}      # 5
    j5_min: set = {54, 32}  # 5 space

    x_max: set = {87}  # w (movement)
    x_min: set = {83}  # s
    y_max: set = {68}  # d
    y_min: set = {65}  # a
    z_max: set = {69}  # e
    z_min: set = {81}  # q

    rx_max: set = {87, 32}  # w space (rotation)
    rx_min: set = {83, 32}  # s space
    ry_max: set = {68, 32}  # d space
    ry_min: set = {65, 32}  # a space
    rz_max: set = {69, 32}  # e space
    rz_min: set = {81, 32}  # q space

    ctrl_j_c: set = {17, 67, 74}  # ctrl c j (copy joint pose)
    ctrl_l_c: set = {17, 67, 76}  # ctrl l j (copy linear pose)

    move: set = {77}  # m
    offset: set = {78}

    free_drive: set = {70}  # f


@dataclass
class Binding:
    name: str
    button: _CustomButton | None
    # sequence: set


class Bindings:
    j0_max: Binding
    j0_min: Binding
    j1_max: Binding
    j1_min: Binding
    j2_max: Binding
    j2_min: Binding
    j3_max: Binding
    j3_min: Binding
    j4_max: Binding
    j4_min: Binding
    j5_max: Binding
    j5_min: Binding

    x_max: Binding
    x_min: Binding
    y_max: Binding
    y_min: Binding
    z_max: Binding
    z_min: Binding

    rx_max: Binding
    rx_min: Binding
    ry_max: Binding
    ry_min: Binding
    rz_max: Binding
    rz_min: Binding

    ctrl_j_c: Binding
    ctrl_l_c: Binding

    move: Binding

    free_drive: Binding


@dataclass
class CoordinateEntryGroup:
    entries: tuple[Entry, ...] = None
    move_btn: _CustomButton = None
    shift_btn: _CustomButton = None


@dataclass
class RTDGroup:
    name: str
    type_: str
    labels: tuple[Label, ...] = None
    copy_btn: _CustomButton = None
    paste_btn: _CustomButton = None
