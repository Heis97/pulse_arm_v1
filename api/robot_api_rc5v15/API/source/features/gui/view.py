from __future__ import annotations

import tkinter as tk
from collections.abc import Callable, Iterable
from tkinter import (
    Checkbutton,
    Entry,
    Frame,
    IntVar,
    Label,
    Scale,
    Tk,
    Widget,
    font,
    messagebox,
    ttk,
)
from typing import Literal

from api.robot_api_rc5v15.API.source.ap_interface.motion import CoordinateSystem
from api.robot_api_rc5v15.API.source.features.gui.tools.data_classes import (
    CoordinateEntryGroup,
    RTDGroup,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    AngleUnitTypes,
    GUICoordinateSystem,
)
from api.robot_api_rc5v15.API.source.models.constants import JOINTS_COUNT

from .tools.constants import (
    CAPTIONS_FONT_SIZE,
    CELL_HEIGHT,
    CELL_WIDTH,
    COLOR_MAP,
    CONTROL_BUTTON_HEIGHT,
    CONTROL_BUTTON_WIDTH,
    COORDS_COUNT,
    COORDS_NAMES,
    HEADERS_FONT_SIZE,
    JOINTS_NAMES,
    MAIN_FONT_SIZE,
)
from .tools.custom_button import PixelButton


class MainWindow(Tk):
    _joints_entry_group: CoordinateEntryGroup
    _move_entry_group: CoordinateEntryGroup
    _offset_entry_group: CoordinateEntryGroup
    _joints_pose_group: RTDGroup
    _tcp_pose_group: RTDGroup

    def __init__(
        self,
        coordinate_system: CoordinateSystem | None,
    ):
        self._coordinate_system = coordinate_system

        super().__init__()
        self._init_view()
        self.protocol("WM_DELETE_WINDOW", self._close_window)

    # Handlers
    ## First column
    def on_joint_0_plus_button_click(self, press: Callable, release: Callable):
        self.j0_max_btn.on_press(press)
        self.j0_max_btn.on_release(release)

    def on_joint_0_minus_button_click(
        self, press: Callable, release: Callable
    ):
        self.j0_min_btn.on_press(press)
        self.j0_min_btn.on_release(release)

    def on_joint_1_plus_button_click(self, press: Callable, release: Callable):
        self.j1_max_btn.on_press(press)
        self.j1_max_btn.on_release(release)

    def on_joint_1_minus_button_click(
        self, press: Callable, release: Callable
    ):
        self.j1_min_btn.on_press(press)
        self.j1_min_btn.on_release(release)

    def on_joint_2_plus_button_click(self, press: Callable, release: Callable):
        self.j2_max_btn.on_press(press)
        self.j2_max_btn.on_release(release)

    def on_joint_2_minus_button_click(
        self, press: Callable, release: Callable
    ):
        self.j2_min_btn.on_press(press)
        self.j2_min_btn.on_release(release)

    def on_joint_3_plus_button_click(self, press: Callable, release: Callable):
        self.j3_max_btn.on_press(press)
        self.j3_max_btn.on_release(release)

    def on_joint_3_minus_button_click(
        self, press: Callable, release: Callable
    ):
        self.j3_min_btn.on_press(press)
        self.j3_min_btn.on_release(release)

    def on_joint_4_plus_button_click(self, press: Callable, release: Callable):
        self.j4_max_btn.on_press(press)
        self.j4_max_btn.on_release(release)

    def on_joint_4_minus_button_click(
        self, press: Callable, release: Callable
    ):
        self.j4_min_btn.on_press(press)
        self.j4_min_btn.on_release(release)

    def on_joint_5_plus_button_click(self, press: Callable, release: Callable):
        self.j5_max_btn.on_press(press)
        self.j5_max_btn.on_release(release)

    def on_joint_5_minus_button_click(
        self, press: Callable, release: Callable
    ):
        self.j5_min_btn.on_press(press)
        self.j5_min_btn.on_release(release)

    ## Second column
    ### Movement joystick
    def on_x_plus_button_click(self, press: Callable, release: Callable):
        self.x_max_btn.on_press(press)
        self.x_max_btn.on_release(release)

    def on_x_minus_button_click(self, press: Callable, release: Callable):
        self.x_min_btn.on_press(press)
        self.x_min_btn.on_release(release)

    def on_y_plus_button_click(self, press: Callable, release: Callable):
        self.y_max_btn.on_press(press)
        self.y_max_btn.on_release(release)

    def on_y_minus_button_click(self, press: Callable, release: Callable):
        self.y_min_btn.on_press(press)
        self.y_min_btn.on_release(release)

    def on_z_plus_button_click(self, press: Callable, release: Callable):
        self.z_max_btn.on_press(press)
        self.z_max_btn.on_release(release)

    def on_z_minus_button_click(self, press: Callable, release: Callable):
        self.z_min_btn.on_press(press)
        self.z_min_btn.on_release(release)

    ### Rotation joystick
    def on_rx_plus_button_click(self, press: Callable, release: Callable):
        self.rx_max_btn.on_press(press)
        self.rx_max_btn.on_release(release)

    def on_rx_minus_button_click(self, press: Callable, release: Callable):
        self.rx_min_btn.on_press(press)
        self.rx_min_btn.on_release(release)

    def on_ry_plus_button_click(self, press: Callable, release: Callable):
        self.ry_max_btn.on_press(press)
        self.ry_max_btn.on_release(release)

    def on_ry_minus_button_click(self, press: Callable, release: Callable):
        self.ry_min_btn.on_press(press)
        self.ry_min_btn.on_release(release)

    def on_rz_plus_button_click(self, press: Callable, release: Callable):
        self.rz_max_btn.on_press(press)
        self.rz_max_btn.on_release(release)

    def on_rz_minus_button_click(self, press: Callable, release: Callable):
        self.rz_min_btn.on_press(press)
        self.rz_min_btn.on_release(release)

    ### Settings
    def on_coordinate_system_changed(self, callback: Callable):
        self.coordinate_system_combobox.bind("<<ComboboxSelected>>", callback)

    def on_orientation_units_changed(self, callback: Callable):
        self.orientation_units_combobox.bind("<<ComboboxSelected>>", callback)

    def on_scale_setup_value_changed(self, callback: Callable):
        self.speed_scale.bind("<ButtonRelease-1>", callback)

    ## Third column
    def on_seven_pose_button_click(self, press: Callable, release: Callable):
        self.seven_pose_btn.on_press(press)
        self.seven_pose_btn.on_release(release)

    def on_free_drive_button_click(self, press: Callable, release: Callable):
        self.free_drive_btn.on_press(press)
        self.free_drive_btn.on_release(release)

    def on_hold_free_drive_switch_toggle(self, callback: Callable):
        self.free_drive_switch.bind(
            "<ButtonRelease-1>",
            lambda e: self.free_drive_switch.after_idle(callback),
        )

    def on_copy_joints_pose_button_click(self, callback: Callable):
        self._joints_pose_group.copy_button.on_press(callback)

    def on_copy_tcp_pose_button_click(self, callback: Callable):
        self._tcp_pose_group.copy_button.on_press(callback)

    def on_clear_movej_pose_button_click(self, callback: Callable):
        self._joints_entry_group.clear_button.on_press(callback)

    def on_movej_to_point_button_click(
        self, press: Callable, release: Callable
    ):
        self._joints_entry_group.move_button.on_press(press)
        self._joints_entry_group.move_button.on_release(release)

    def on_paste_movej_pose_button_click(self, callback: Callable):
        self._joints_entry_group.paste_button.on_press(callback)

    def on_clear_move_pose_button_click(self, callback: Callable):
        self._move_entry_group.clear_button.on_press(callback)

    def on_move_to_point_button_click(
        self, press: Callable, release: Callable
    ):
        self._move_entry_group.move_button.on_press(press)
        self._move_entry_group.move_button.on_release(release)

    def on_paste_move_pose_button_click(self, callback: Callable):
        self._move_entry_group.paste_button.on_press(callback)

    def on_clear_offset_pose_button_click(self, callback: Callable):
        self._offset_entry_group.clear_button.on_press(callback)

    def on_offset_to_point_button_click(
        self, press: Callable, release: Callable
    ):
        self._offset_entry_group.move_button.on_press(press)
        self._offset_entry_group.move_button.on_release(release)

    def on_paste_offset_pose_button_click(self, callback: Callable):
        self._offset_entry_group.paste_button.on_press(callback)

    # Getters and setters
    ## Second column
    def get_coordinate_system(self) -> GUICoordinateSystem:
        cs = self.coordinate_system_combobox.get()
        return GUICoordinateSystem(cs)

    def set_coordinate_system(self, cs: GUICoordinateSystem):
        self.coordinate_system_combobox.set(cs.value)

    def get_angle_units(self) -> AngleUnitTypes:
        cs = self.orientation_units_combobox.get()
        return AngleUnitTypes(cs)

    def set_angle_units(self, cs: AngleUnitTypes):
        self.orientation_units_combobox.set(cs.value)

    def get_scale_setup(self) -> float:
        return self.speed_scale.get()

    def set_scale_setup(self, value: float):
        self.speed_scale.set(value)

    def set_clipboard_value(self, value: str):
        self.clipboard_label.configure(text=value)

    ## Third column
    def set_free_drive_btn_checkable(self, is_checkable: bool):
        self.free_drive_btn.config(checkable=is_checkable)

    def set_move_to_7_btn_checkable(self, is_checkable: bool):
        self.seven_pose_btn.config(checkable=is_checkable)

    def is_hold_free_drive_checked(self) -> bool:
        return bool(self.hold_free_drive.get())

    def set_hold_free_drive_checked(self, is_checked: bool):
        self.hold_free_drive.set(is_checked)

    def set_joints_pose(self, values: Iterable[str]):
        for value, label in zip(values, self._joints_pose_group.labels):
            label.configure(text=value)

    def set_tcp_pose(self, values: Iterable[str]):
        for value, label in zip(values, self._tcp_pose_group.labels):
            label.configure(text=value)

    def get_movej_pose(self) -> tuple[float, ...]:
        return tuple(
            [float(entry.get()) for entry in self._joints_entry_group.entries]
        )

    def set_movej_pose(self, pose: Iterable[str]):
        for value, entry in zip(pose, self._joints_entry_group.entries):
            entry.delete(0, tk.END)
            entry.insert(0, value)

    def set_movej_to_pose_btn_checkable(self, is_checkable: bool):
        self._joints_entry_group.move_button.config(checkable=is_checkable)

    def get_move_pose(self) -> tuple[float, ...]:
        return tuple(
            [float(entry.get()) for entry in self._move_entry_group.entries]
        )

    def set_move_pose(self, pose: Iterable[str]):
        for value, entry in zip(pose, self._move_entry_group.entries):
            entry.delete(0, tk.END)
            entry.insert(0, value)

    def set_move_to_pose_btn_checkable(self, is_checkable: bool):
        self._move_entry_group.move_button.config(checkable=is_checkable)

    def get_offset_pose(self) -> tuple[float, ...]:
        return tuple(
            [float(entry.get()) for entry in self._offset_entry_group.entries]
        )

    def set_offset_pose(self, pose: Iterable[str]):
        for value, entry in zip(pose, self._offset_entry_group.entries):
            entry.delete(0, tk.END)
            entry.insert(0, value)

    def set_offset_to_pose_btn_checkable(self, is_checkable: bool):
        self._offset_entry_group.move_button.config(checkable=is_checkable)

    # Service
    def show_message(
        self,
        level: Literal["info", "warning", "error"],
        message: str,
        title: str = "Ошибка",
    ):
        if level == "info":
            messagebox.showinfo(
                title,
                message,
            )
        elif level == "warning":
            messagebox.showwarning(
                title,
                message,
            )
        else:
            messagebox.showerror(
                title,
                message,
            )

    # View construction
    def _init_view(self):
        self.title("Simple joystick")
        self.resizable(False, False)
        self.main_frame = Frame(self, padx=20, pady=20)
        self.main_frame.pack(
            expand=True,
            fill="both",
            side="right",
            padx=(10, 10),
            pady=(10, 10),
        )
        label = Label(
            self.main_frame,
            text="Угловое перемещение робота",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )
        label.grid(row=1, column=1, sticky="n")
        self.motor_frame = self._construct_joints_motion_frame(self.main_frame)
        self.motor_frame.grid(row=2, column=1, rowspan=2, sticky="n")

        ttk.Separator(self.main_frame, orient="vertical").grid(
            row=1, column=2, rowspan=10, sticky="ns", padx=5
        )

        label = Label(
            self.main_frame,
            text="Линейное перемещение робота",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )
        label.grid(row=1, column=3, columnspan=2, sticky="n")
        self.movement_frame = self._construct_movement_joystick_frame(
            self.main_frame
        )
        self.movement_frame.grid(row=2, column=3, sticky="n")
        self.rotation_frame = self._construct_rotation_joystick_frame(
            self.main_frame
        )
        self.rotation_frame.grid(row=2, column=4, sticky="n")
        self.settings_frame = self._construct_settings_frame(self.main_frame)
        self.settings_frame.grid(
            row=3, column=3, columnspan=2, pady=60, sticky="n"
        )

        ttk.Separator(self.main_frame, orient="vertical").grid(
            row=1, column=5, rowspan=10, sticky="ns", padx=5
        )

        self.control_frame = self._construct_control_frame(self.main_frame)
        self.control_frame.grid(row=1, column=6, rowspan=3, sticky="n")

        self.main_frame.grid_rowconfigure(0, weight=0)
        self.main_frame.grid_rowconfigure(1, weight=0)
        self.main_frame.grid_rowconfigure(2, weight=0)
        self.main_frame.grid_rowconfigure(3, weight=1)

    def _construct_joints_motion_frame(self, parent: Frame) -> Frame:
        frame = Frame(parent, borderwidth=0, relief="solid", padx=2, pady=2)
        label = Label(
            frame,
            text="Положение сочленения",
            font=font.Font(size=MAIN_FONT_SIZE),
        )
        label.grid(row=1, column=1, columnspan=3, sticky="n")

        frame_font = font.Font(size=MAIN_FONT_SIZE)
        self.j0_min_btn, self.j0_max_btn = self._construct_joint_motion_row(
            f"Основание ( {JOINTS_NAMES[0]})", frame_font, frame
        )
        self.j1_min_btn, self.j1_max_btn = self._construct_joint_motion_row(
            f"Плечо ( {JOINTS_NAMES[1]})", frame_font, frame
        )
        self.j2_min_btn, self.j2_max_btn = self._construct_joint_motion_row(
            f"Локоть ( {JOINTS_NAMES[2]})", frame_font, frame
        )
        self.j3_min_btn, self.j3_max_btn = self._construct_joint_motion_row(
            f"Запястье 1 ( {JOINTS_NAMES[3]})", frame_font, frame
        )
        self.j4_min_btn, self.j4_max_btn = self._construct_joint_motion_row(
            f"Запястье 2 ( {JOINTS_NAMES[4]})", frame_font, frame
        )
        self.j5_min_btn, self.j5_max_btn = self._construct_joint_motion_row(
            f"Запястье 3 ( {JOINTS_NAMES[5]})",
            frame_font,
            frame,
            add_separator=False,
        )
        return frame

    @staticmethod
    def _construct_joint_motion_row(
        text: str,
        font: font.Font,
        parent: Widget,
        add_separator: bool = True,
    ) -> tuple[PixelButton, PixelButton]:
        row_number = parent.grid_size()[1] + 1
        button_left = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="➖",
        )
        button_left.grid(row=row_number, column=1, padx=2, pady=2)
        label = Label(
            parent,
            font=font,
            text=text,
            width=15,
            height=2,
            borderwidth=1,
        )
        label.grid(row=row_number, column=2, padx=2, pady=2)
        button_right = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["red"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["red"],
            text="➕",
        )
        button_right.grid(row=row_number, column=3, padx=2, pady=2)

        if add_separator:
            ttk.Separator(parent, orient="horizontal").grid(
                row=row_number + 1, column=1, columnspan=3, sticky="ew", pady=1
            )

        return button_left, button_right

    @staticmethod
    def _construct_joystick(
        top_left_text: str,
        top_right_text: str,
        top_center_text: str,
        bottom_center_text: str,
        center_left_text: str,
        center_right_text: str,
        font: font.Font,
        parent: Widget,
    ) -> tuple[
        PixelButton,
        PixelButton,
        PixelButton,
        PixelButton,
        PixelButton,
        PixelButton,
    ]:
        row_number = parent.grid_size()[1] + 1
        top_left = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text=top_left_text,
        )
        top_left.grid(row=row_number + 1, column=1, padx=2, pady=2)

        top_right = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text=top_right_text,
        )
        top_right.grid(row=row_number + 1, column=3, padx=2, pady=2)

        center_right = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["green"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["green"],
            text=center_right_text,
        )
        center_right.grid(row=row_number + 2, column=3, padx=2, pady=2)
        center_left = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["green"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["green"],
            text=center_left_text,
        )
        center_left.grid(row=row_number + 2, column=1, padx=2, pady=2)
        top_center = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["red"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["red"],
            text=top_center_text,
        )
        top_center.grid(row=row_number + 1, column=2, padx=2, pady=2)
        bottom_center = PixelButton(
            parent,
            width=CONTROL_BUTTON_WIDTH,
            height=CONTROL_BUTTON_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["red"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["red"],
            text=bottom_center_text,
        )
        bottom_center.grid(row=row_number + 3, column=2, padx=2, pady=2)

        return (
            top_left,
            top_right,
            top_center,
            bottom_center,
            center_left,
            center_right,
        )

    def _construct_movement_joystick_frame(self, parent: Frame) -> Frame:
        frame = Frame(parent, borderwidth=0, padx=2, pady=2)

        label = Label(frame, text="Перемещение")
        label.grid(row=1, column=1, columnspan=3, sticky="n")

        (
            self.z_min_btn,
            self.z_max_btn,
            self.x_max_btn,
            self.x_min_btn,
            self.y_min_btn,
            self.y_max_btn,
        ) = self._construct_joystick(
            top_left_text="Z ➖",
            top_right_text="Z ➕",
            top_center_text="X ➕",
            bottom_center_text="X ➖",
            center_left_text="Y ➖",
            center_right_text="Y ➕",
            font=font.Font(size=MAIN_FONT_SIZE),
            parent=frame,
        )
        return frame

    def _construct_rotation_joystick_frame(self, parent: Frame) -> Frame:
        frame = Frame(parent, borderwidth=0, padx=2, pady=2)

        label = Label(frame, text="Вращение")
        label.grid(row=1, column=1, columnspan=3, sticky="n")

        (
            self.rz_min_btn,
            self.rz_max_btn,
            self.rx_max_btn,
            self.rx_min_btn,
            self.ry_min_btn,
            self.ry_max_btn,
        ) = self._construct_joystick(
            top_left_text="RZ ➖",
            top_right_text="RZ ➕",
            top_center_text="RX ➕",
            bottom_center_text="RX ➖",
            center_left_text="RY ➖",
            center_right_text="RY ➕",
            font=font.Font(size=MAIN_FONT_SIZE),
            parent=frame,
        )
        return frame

    def _construct_settings_frame(self, parent: Frame) -> Frame:
        frame = Frame(parent, borderwidth=0, padx=2, pady=2)

        # Coordinate system
        label = Label(frame, text="Система координат:", anchor="w")
        label.grid(row=1, column=1, sticky="w", padx=2, pady=2)

        combobox_values = (
            [
                GUICoordinateSystem.GLOBAL.value,
                GUICoordinateSystem.LOCAL.value,
                GUICoordinateSystem.CTI.value,
            ]
            if self._coordinate_system
            else [
                GUICoordinateSystem.GLOBAL.value,
                GUICoordinateSystem.CTI.value,
            ]
        )
        self.coordinate_system_combobox = ttk.Combobox(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            values=combobox_values,
            state="readonly",
        )
        self.coordinate_system_combobox.grid(
            row=1, column=2, columnspan=2, padx=2, pady=2, sticky="ew"
        )
        self.coordinate_system_combobox.set(combobox_values[0])

        # Orientation units
        label = Label(frame, text="Единицы измерения:", anchor="w")
        label.grid(row=2, column=1, sticky="w", padx=2, pady=2)

        self.orientation_units_combobox = ttk.Combobox(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            values=[
                AngleUnitTypes.DEG.value,
                AngleUnitTypes.RAD.value,
            ],
            state="readonly",
        )
        self.orientation_units_combobox.grid(
            row=2, column=2, columnspan=2, padx=2, pady=2, sticky="ew"
        )
        self.orientation_units_combobox.set(AngleUnitTypes.DEG)

        # Scale setup
        label = Label(frame, text="Скорость:", anchor="w")
        label.grid(row=3, column=1, sticky="w", padx=2, pady=2)

        value_label = Label(frame, text="0.01")

        def on_scale_change(value):
            formatted = f"{float(value):.2f}"
            value_label.config(text=formatted)

        self.speed_scale = Scale(
            frame,
            orient="horizontal",
            from_=0.01,
            to=1.0,
            resolution=0.01,
            command=on_scale_change,
            showvalue=False,
        )
        value_label.grid(row=3, column=2, sticky="e", padx=2, pady=2)
        self.speed_scale.grid(row=3, column=3, sticky="ew", padx=2, pady=2)
        self.speed_scale.set(0.1)

        frame.grid_columnconfigure(1, minsize=180)
        frame.grid_columnconfigure(2, weight=0)
        frame.grid_columnconfigure(3, weight=1)

        return frame

    def _construct_control_frame(self, parent: Frame) -> Frame:
        frame = Frame(parent, borderwidth=0, padx=2, pady=2)

        # Seven pose
        self.seven_pose_btn = PixelButton(
            frame,
            width=CELL_WIDTH * 3,
            height=CELL_HEIGHT,
            repeat_sig=False,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Положение «Семерка»",
            checkable=True,
        )
        self.seven_pose_btn.grid(
            row=1,
            column=1,
            columnspan=3,
            padx=2,
            pady=2,
        )

        # Free drive
        self.free_drive_btn = PixelButton(
            frame,
            width=CELL_WIDTH * 3,
            height=CELL_HEIGHT,
            repeat_sig=True,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Свободный привод",
            checkable=True,
        )
        self.free_drive_btn.grid(
            row=1,
            column=4,
            columnspan=3,
            padx=2,
            pady=2,
        )
        self.hold_free_drive = IntVar()
        self.free_drive_switch = Checkbutton(
            frame,
            text="Удержание кнопок\nдвижения",
            variable=self.hold_free_drive,
        )
        self.free_drive_switch.grid(row=1, column=7, columnspan=3, rowspan=2)

        self._construct_joints_angle_values_group(frame=frame)
        self._construct_tcp_pose_group(frame=frame)
        self._construct_joint_entry_group(frame=frame)
        self._construct_waypoint_entry_group(frame=frame)
        self._construct_offset_entry_group(frame=frame)

        # Clipboard
        row_number = frame.grid_size()[1]
        label = Label(frame, text="Буфер обмена:", anchor="w")
        label.grid(
            row=row_number + 1,
            column=1,
            columnspan=2,
            sticky="w",
            padx=2,
            pady=2,
        )

        label_frame, self.clipboard_label = self._create_label_frame(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            width=1,  # Not affect
            height=CELL_HEIGHT,
        )
        label_frame.grid(
            row=row_number + 1,
            column=3,
            columnspan=20,
            padx=2,
            pady=15,
            sticky="ew",
        )
        self.clipboard_label.configure(text="")

        return frame

    def _construct_joints_angle_values_group(self, frame: Frame):
        label = Label(
            frame,
            text="Углы поворотов моторов",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )

        rtd_group = RTDGroup()
        row_number = frame.grid_size()[1]

        label.grid(
            row=row_number + 1,
            column=1,
            columnspan=JOINTS_COUNT + 1,
            sticky="w",
            pady=(10, 0),
        )

        labels = []

        for j in range(JOINTS_COUNT):
            name_label = Label(
                frame,
                text=JOINTS_NAMES[j],
                font=font.Font(size=CAPTIONS_FONT_SIZE),
            )

            cell_frame, label = self._create_label_frame(
                parent=frame,
                font=font.Font(size=MAIN_FONT_SIZE),
                width=CELL_WIDTH,
                height=CELL_HEIGHT,
            )
            name_label.grid(row=row_number + 2, column=j + 1, padx=0, pady=0)
            cell_frame.grid(row=row_number + 3, column=j + 1, padx=2, pady=0)
            labels.append(label)

        copy_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(3.2 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Копировать",
        )
        copy_btn.grid(
            row=row_number + 3,
            column=JOINTS_COUNT + 1,
            columnspan=3,
            padx=2,
            pady=2,
        )

        rtd_group.labels = tuple(labels)
        rtd_group.copy_button = copy_btn
        self._joints_pose_group = rtd_group

    def _construct_tcp_pose_group(self, frame: Frame):
        label = Label(
            frame,
            text="Положение ЦТИ",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )

        rtd_group = RTDGroup()

        row_number = frame.grid_size()[1]

        label.grid(
            row=row_number + 1,
            column=1,
            columnspan=COORDS_COUNT + 1,
            sticky="w",
            pady=(10, 0),
        )

        labels = []

        for j in range(COORDS_COUNT):
            name_label = Label(
                frame,
                text=COORDS_NAMES[j],
                font=font.Font(size=CAPTIONS_FONT_SIZE),
            )

            cell_frame, label = self._create_label_frame(
                parent=frame,
                font=font.Font(size=MAIN_FONT_SIZE),
                width=CELL_WIDTH,
                height=CELL_HEIGHT,
            )

            name_label.grid(row=row_number + 2, column=j + 1, padx=0, pady=0)
            cell_frame.grid(row=row_number + 3, column=j + 1, padx=2, pady=0)
            labels.append(label)

        copy_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(3.2 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Копировать",
        )
        copy_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 1,
            columnspan=3,
            padx=2,
            pady=2,
        )

        rtd_group.labels = tuple(labels)
        rtd_group.copy_button = copy_btn
        self._tcp_pose_group = rtd_group

    def _construct_joint_entry_group(self, frame: Frame):
        label = Label(
            frame,
            text="Угловое перемещение",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )

        row_number = frame.grid_size()[1]

        label.grid(
            row=row_number + 1,
            column=1,
            columnspan=COORDS_COUNT + 1,
            sticky="w",
            pady=(10, 0),
        )

        entries = []

        for j in range(JOINTS_COUNT):
            name_label = Label(
                frame,
                text=JOINTS_NAMES[j],
                font=font.Font(size=CAPTIONS_FONT_SIZE),
            )

            cell_frame, entry = self._create_entry_frame(
                parent=frame,
                font=font.Font(size=MAIN_FONT_SIZE),
                width=CELL_WIDTH,
                height=CELL_HEIGHT,
            )

            name_label.grid(row=row_number + 2, column=j + 1, padx=0, pady=0)
            cell_frame.grid(row=row_number + 3, column=j + 1, padx=2, pady=0)
            entries.append(entry)

        clear_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(0.5 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["red"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["red"],
            text="🠴",
        )
        clear_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 1,
            padx=2,
            pady=2,
        )

        paste_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=CELL_WIDTH,
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Вставить",
        )
        paste_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 2,
            padx=2,
            pady=2,
        )

        move_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(1.5 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Переместить",
            checkable=True,
        )
        move_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 3,
            padx=2,
            pady=2,
        )

        entry_group = CoordinateEntryGroup(
            entries=tuple(entries),
            move_button=move_btn,
            paste_button=paste_btn,
            clear_button=clear_btn,
        )
        self._joints_entry_group = entry_group

    def _construct_waypoint_entry_group(self, frame: Frame):
        label = Label(
            frame,
            text="Линейное перемещение",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )

        row_number = frame.grid_size()[1]

        label.grid(
            row=row_number + 1,
            column=1,
            columnspan=COORDS_COUNT + 1,
            sticky="w",
            pady=(10, 0),
        )

        entries = []

        for j in range(COORDS_COUNT):
            name_label = Label(
                frame,
                text=COORDS_NAMES[j],
                font=font.Font(size=CAPTIONS_FONT_SIZE),
            )

            cell_frame, entry = self._create_entry_frame(
                parent=frame,
                font=font.Font(size=MAIN_FONT_SIZE),
                width=CELL_WIDTH,
                height=CELL_HEIGHT,
            )

            name_label.grid(row=row_number + 2, column=j + 1, padx=0, pady=0)
            cell_frame.grid(row=row_number + 3, column=j + 1, padx=2, pady=0)
            entries.append(entry)

        clear_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(0.5 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["red"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["red"],
            text="🠴",
        )
        clear_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 1,
            padx=2,
            pady=2,
        )

        paste_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=CELL_WIDTH,
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Вставить",
        )
        paste_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 2,
            padx=2,
            pady=2,
        )

        move_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(1.5 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Переместить",
            checkable=True,
        )
        move_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 3,
            padx=2,
            pady=2,
        )

        entry_group = CoordinateEntryGroup(
            entries=tuple(entries),
            move_button=move_btn,
            paste_button=paste_btn,
            clear_button=clear_btn,
        )
        self._move_entry_group = entry_group

    def _construct_offset_entry_group(self, frame: Frame):
        label = Label(
            frame,
            text="Линейное смещение",
            font=font.Font(size=HEADERS_FONT_SIZE),
        )

        row_number = frame.grid_size()[1]

        label.grid(
            row=row_number + 1,
            column=1,
            columnspan=COORDS_COUNT + 1,
            sticky="w",
            pady=(10, 0),
        )

        entries = []

        for j in range(COORDS_COUNT):
            name_label = Label(
                frame,
                text=COORDS_NAMES[j],
                font=font.Font(size=CAPTIONS_FONT_SIZE),
            )

            cell_frame, entry = self._create_entry_frame(
                parent=frame,
                font=font.Font(size=MAIN_FONT_SIZE),
                width=CELL_WIDTH,
                height=CELL_HEIGHT,
            )

            name_label.grid(row=row_number + 2, column=j + 1, padx=0, pady=0)
            cell_frame.grid(row=row_number + 3, column=j + 1, padx=2, pady=0)
            entries.append(entry)

        clear_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(0.5 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["red"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["red"],
            text="🠴",
        )
        clear_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 1,
            padx=2,
            pady=2,
        )

        paste_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=CELL_WIDTH,
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Вставить",
        )
        paste_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 2,
            padx=2,
            pady=2,
        )

        move_btn = PixelButton(
            frame,
            font=font.Font(size=MAIN_FONT_SIZE),
            repeat_sig=False,
            width=int(1.5 * CELL_WIDTH),
            height=CELL_HEIGHT,
            fg=COLOR_MAP["blue"],
            pressed_fg=COLOR_MAP["white"],
            pressed_bg=COLOR_MAP["blue"],
            text="Сместить",
            checkable=True,
        )
        move_btn.grid(
            row=row_number + 3,
            column=COORDS_COUNT + 3,
            padx=2,
            pady=2,
        )

        entry_group = CoordinateEntryGroup(
            entries=tuple(entries),
            move_button=move_btn,
            paste_button=paste_btn,
            clear_button=clear_btn,
        )
        self._offset_entry_group = entry_group

    @staticmethod
    def _create_entry_frame(
        parent: Frame, font: font.Font, width: int, height: int
    ) -> tuple[Frame, Entry]:
        cell_frame = Frame(
            parent,
            width=width,
            height=height,
        )
        validate_symbols = cell_frame.register(Validation.gui_entry)
        cell_frame.pack_propagate(False)

        entry = Entry(
            cell_frame,
            font=font,
            validate="key",
            validatecommand=(validate_symbols, "%P"),
            justify="center",
            borderwidth=1,
            relief="raised",
        )
        entry.delete(0, tk.END)
        entry.insert(0, "0")
        entry.pack(expand=True, fill="both")
        return cell_frame, entry

    @staticmethod
    def _create_label_frame(
        parent: Frame, font: font.Font, width: int, height: int
    ) -> tuple[Frame, Label]:
        cell_frame = Frame(
            parent,
            width=width,
            height=height,
            borderwidth=1,
            relief="solid",
        )
        cell_frame.pack_propagate(False)

        label = Label(
            cell_frame,
            text="000.00",
            anchor="center",
            font=font,
        )
        label.pack(expand=True, fill="both")
        return cell_frame, label

    def update_task(
        self,
        cycle_time: int,
        function: Callable,
    ):
        self._update_task = self.after(cycle_time, function)

    def _close_window(self):
        self.after_cancel(self._update_task)
        self.destroy()
