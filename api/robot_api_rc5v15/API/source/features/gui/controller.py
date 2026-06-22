from __future__ import annotations

from api.robot_api_rc5v15.API.source.ap_interface.motion import (
    CoordinateSystem,
    Motion,
)
from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import ConnectionError
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    AddWaypointError,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    AngleUnitTypes,
    GUICoordinateSystem,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import AngleUnits

from .model import ActiveCoordinateSystem, Model
from .tools.clipboard_watcher import ClipboardWatcher
from .view import MainWindow


class SimpleJoystickController:
    def __init__(
        self,
        motion: Motion,
        coordinate_system: CoordinateSystem | None,
    ):
        super().__init__()
        self._view = MainWindow(coordinate_system)
        self._model = Model(motion, user_cs=coordinate_system)
        self._clipboard_watcher = ClipboardWatcher(
            parent=self._view, callback=self.handle_clipboard_value_changed
        )

        self._active_units: AngleUnits = self._model.get_units()
        self._active_cs: ActiveCoordinateSystem = "base"

        if self._active_units == "rad":
            self._view.set_angle_units(AngleUnitTypes.RAD)
        else:
            self._view.set_angle_units(AngleUnitTypes.DEG)
        self._model.set_velocity_scale(self._view.get_scale_setup())

        self._view.set_free_drive_btn_checkable(False)
        self._view.set_move_to_7_btn_checkable(False)
        self._view.set_movej_to_pose_btn_checkable(False)
        self._view.set_move_to_pose_btn_checkable(False)
        self._view.set_offset_to_pose_btn_checkable(False)

        self._apply_callbacks()
        self._view.update_task(100, self.update_actual_pose)
        self._clipboard_watcher.start()
        self._view.mainloop()

    def _apply_callbacks(self):
        # First column
        self._view.on_joint_0_minus_button_click(
            press=lambda: self._model.joint_jog(0, "-"),
            release=self._model.stop,
        )
        self._view.on_joint_0_plus_button_click(
            press=lambda: self._model.joint_jog(0, "+"),
            release=self._model.stop,
        )

        self._view.on_joint_1_minus_button_click(
            press=lambda: self._model.joint_jog(1, "-"),
            release=self._model.stop,
        )
        self._view.on_joint_1_plus_button_click(
            press=lambda: self._model.joint_jog(1, "+"),
            release=self._model.stop,
        )

        self._view.on_joint_2_minus_button_click(
            press=lambda: self._model.joint_jog(2, "-"),
            release=self._model.stop,
        )
        self._view.on_joint_2_plus_button_click(
            press=lambda: self._model.joint_jog(2, "+"),
            release=self._model.stop,
        )

        self._view.on_joint_3_minus_button_click(
            press=lambda: self._model.joint_jog(3, "-"),
            release=self._model.stop,
        )
        self._view.on_joint_3_plus_button_click(
            press=lambda: self._model.joint_jog(3, "+"),
            release=self._model.stop,
        )

        self._view.on_joint_4_minus_button_click(
            press=lambda: self._model.joint_jog(4, "-"),
            release=self._model.stop,
        )
        self._view.on_joint_4_plus_button_click(
            press=lambda: self._model.joint_jog(4, "+"),
            release=self._model.stop,
        )

        self._view.on_joint_5_minus_button_click(
            press=lambda: self._model.joint_jog(5, "-"),
            release=self._model.stop,
        )
        self._view.on_joint_5_plus_button_click(
            press=lambda: self._model.joint_jog(5, "+"),
            release=self._model.stop,
        )

        # Second column
        ## Movement joystick
        self._view.on_x_minus_button_click(
            press=lambda: self._model.linear_jog("X", "-"),
            release=self._model.stop,
        )
        self._view.on_x_plus_button_click(
            press=lambda: self._model.linear_jog("X", "+"),
            release=self._model.stop,
        )
        self._view.on_y_minus_button_click(
            press=lambda: self._model.linear_jog("Y", "-"),
            release=self._model.stop,
        )
        self._view.on_y_plus_button_click(
            press=lambda: self._model.linear_jog("Y", "+"),
            release=self._model.stop,
        )
        self._view.on_z_minus_button_click(
            press=lambda: self._model.linear_jog("Z", "-"),
            release=self._model.stop,
        )
        self._view.on_z_plus_button_click(
            press=lambda: self._model.linear_jog("Z", "+"),
            release=self._model.stop,
        )

        ## Rotation joystick
        self._view.on_rx_minus_button_click(
            press=lambda: self._model.linear_jog("Rx", "-"),
            release=self._model.stop,
        )
        self._view.on_rx_plus_button_click(
            press=lambda: self._model.linear_jog("Rx", "+"),
            release=self._model.stop,
        )
        self._view.on_ry_minus_button_click(
            press=lambda: self._model.linear_jog("Ry", "-"),
            release=self._model.stop,
        )
        self._view.on_ry_plus_button_click(
            press=lambda: self._model.linear_jog("Ry", "+"),
            release=self._model.stop,
        )
        self._view.on_rz_minus_button_click(
            press=lambda: self._model.linear_jog("Rz", "-"),
            release=self._model.stop,
        )
        self._view.on_rz_plus_button_click(
            press=lambda: self._model.linear_jog("Rz", "+"),
            release=self._model.stop,
        )

        ## Settings
        self._view.on_coordinate_system_changed(
            self.handle_coordinate_system_changed
        )
        self._view.on_orientation_units_changed(
            self.handle_angle_units_changed
        )
        self._view.on_scale_setup_value_changed(
            self.handle_speed_scale_changed
        )

        # Third column
        self._view.on_seven_pose_button_click(
            press=self._model.move_to_7_pose, release=self._model.stop
        )
        self._view.on_free_drive_button_click(
            press=self._model.activate_free_drive, release=self._model.stop
        )
        self._view.on_hold_free_drive_switch_toggle(
            self.handle_hold_button_press_toggled
        )

        self._view.on_copy_joints_pose_button_click(
            self.handle_copy_joint_pose
        )
        self._view.on_copy_tcp_pose_button_click(self.handle_copy_tcp_pose)

        self._view.on_movej_to_point_button_click(
            press=self.handle_movej_to_point,
            release=self._model.stop,
        )
        self._view.on_paste_movej_pose_button_click(
            self.handle_paste_movej_pose
        )
        self._view.on_clear_movej_pose_button_click(
            self.handle_clear_movej_pose
        )

        self._view.on_move_to_point_button_click(
            press=self.handle_move_to_point,
            release=self._model.stop,
        )
        self._view.on_paste_move_pose_button_click(self.handle_paste_move_pose)
        self._view.on_clear_move_pose_button_click(self.handle_clear_move_pose)
        self._view.on_offset_to_point_button_click(
            press=self.handle_offset_to_point,
            release=self._model.stop,
        )
        self._view.on_paste_offset_pose_button_click(
            self.handle_paste_offset_pose
        )
        self._view.on_clear_offset_pose_button_click(
            self.handle_clear_offset_pose
        )

    # Handlers
    ## Second column
    def handle_angle_units_changed(self, *args):
        units = self._view.get_angle_units()
        self._active_units = "deg" if units == AngleUnitTypes.DEG else "rad"

    def handle_coordinate_system_changed(self, *args):
        cs_type = self._view.get_coordinate_system()
        if cs_type == GUICoordinateSystem.CTI:
            self._active_cs = "tcp"
            self._model.set_linear_jog_frame("tcp")
        elif cs_type == GUICoordinateSystem.LOCAL:
            self._active_cs = "user"
            self._model.set_linear_jog_frame("base")
        else:
            self._active_cs = "base"
            self._model.set_linear_jog_frame("base")

    def handle_speed_scale_changed(self, *args):
        scale_value = self._view.get_scale_setup()
        self._model.set_velocity_scale(scale_value)

    def handle_clipboard_value_changed(self, value: str):
        self._view.set_clipboard_value(value)

    ## Third column
    def handle_hold_button_press_toggled(self, *args):
        hold = self._view.is_hold_free_drive_checked()
        if not hold:
            self._model.stop()

        self._view.set_free_drive_btn_checkable(hold)
        self._view.set_move_to_7_btn_checkable(hold)
        self._view.set_movej_to_pose_btn_checkable(hold)
        self._view.set_move_to_pose_btn_checkable(hold)
        self._view.set_offset_to_pose_btn_checkable(hold)

    def handle_copy_tcp_pose(self):
        pose = self._model.get_tcp_pose(
            units=self._active_units,
            coord_system=self._active_cs,
        )

        self._view.clipboard_clear()
        self._view.clipboard_append(str([round(x, 6) for x in pose]))

    def handle_copy_joint_pose(self):
        pose = self._model.get_joints_pose(
            units=self._active_units,
            coord_system=self._active_cs,
        )

        self._view.clipboard_clear()
        self._view.clipboard_append(str([round(x, 6) for x in pose]))

    def handle_clear_movej_pose(self):
        self._view.set_movej_pose(("0",) * 6)

    def handle_paste_movej_pose(self):
        data = self._get_clipboard_data()
        if data is None:
            return
        data = [f"{x:.3f}" for x in data]
        self._view.set_movej_pose(data)

    def handle_clear_move_pose(self):
        self._view.set_move_pose(("0",) * 6)

    def handle_paste_move_pose(self):
        data = self._get_clipboard_data()
        if data is None:
            return
        data = [f"{x:.3f}" for x in data]
        self._view.set_move_pose(data)

    def handle_clear_offset_pose(self):
        self._view.set_offset_pose(("0",) * 6)

    def handle_paste_offset_pose(self):
        data = self._get_clipboard_data()
        if data is None:
            return
        data = [f"{x:.3f}" for x in data]
        self._view.set_offset_pose(data)

    def handle_movej_to_point(self):
        pose = self._view.get_movej_pose()
        self._model.move_to_joint_pose(pose=pose, units=self._active_units)

    def handle_move_to_point(self):
        pose = self._view.get_move_pose()
        try:
            self._model.move_to_tcp_pose(
                pose=pose,
                units=self._active_units,
                coord_system=self._active_cs,
            )
        except AddWaypointError:
            self._view.show_message(
                level="error",
                message="Не удалось переместиться в точку: указанная точка недостижима.",
                title="Ошибка",
            )

    def handle_offset_to_point(self):
        offset = self._view.get_offset_pose()
        try:
            self._model.offset_to_tcp_pose(
                offset=offset,
                units=self._active_units,
                coord_system=self._active_cs,
            )
        except AddWaypointError:
            self._view.show_message(
                level="error",
                message="Не удалось сместиться в точку: смещенная точка недостижима.",
                title="Ошибка",
            )

    def update_actual_pose(self):
        """
        Обновить текущее положение робота.
        """

        try:
            joints_pose = self._model.get_joints_pose(
                units=self._active_units,
                coord_system=self._active_cs,
            )
        except ConnectionError:
            self._view.show_message(
                level="error",
                message=(
                    "Не удалось получить RTD от робота: соединение с роботом "
                    "потеряно. Необходим перезапуск программы."
                ),
                title="Ошибка",
            )
            return

        if self._active_cs == "tcp":
            linear_pose = [0] * 6
        else:
            linear_pose = self._model.get_tcp_pose(
                units=self._active_units,
                coord_system=self._active_cs,
            )

        self._view.set_joints_pose([f"{x:.3f}" for x in joints_pose])
        self._view.set_tcp_pose([f"{x:.3f}" for x in linear_pose])
        self._view.update_task(50, self.update_actual_pose)

    def _get_clipboard_data(self) -> list[float] | None:
        raw_data = self._view.clipboard_get().strip("[]").strip("()")
        data = [data.strip() for data in raw_data.split(",")]
        if len(data) != 6:
            return None
        try:
            data = [float(x) for x in data]
        except Exception:
            return None
        return data
