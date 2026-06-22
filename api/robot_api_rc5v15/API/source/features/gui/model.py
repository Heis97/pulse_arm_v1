from __future__ import annotations

from typing import Literal

from api.robot_api_rc5v15.API.source.ap_interface.motion import CoordinateSystem, Motion
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    JogAxis,
    JogDirection,
    JointIndex,
    PositionOrientation,
    ReferenceFrame,
)

ActiveCoordinateSystem = Literal["base", "user", "tcp"]


class Model:
    def __init__(
        self, motion: Motion, user_cs: CoordinateSystem | None
    ) -> None:
        self._motion = motion
        self._user_cs = user_cs

    def get_units(self) -> AngleUnits:
        return MOTION_SETUP.units

    def set_velocity_scale(self, scale: float):
        self._motion.scale_setup.set(velocity=scale, acceleration=scale)

    def move_to_7_pose(self):
        self._motion.joint.add_new_waypoint(
            angle_pose=(0, -120, 120, -90, -90, 0),
            speed=100,
            accel=10,
            blend=0,
            units="deg",
        )
        self._motion.mode.set("move")

    def joint_jog(self, index: JointIndex, direction: JogDirection):
        self._motion.joint.jog_once(index, direction)

    def linear_jog(self, axis: JogAxis, direction: JogDirection):
        self._motion.linear.jog_once(axis, direction)

    def set_linear_jog_frame(self, frame: ReferenceFrame):
        self._motion.linear.set_jog_param_in_tcp(frame)

    def activate_free_drive(self):
        self._motion.free_drive(enable=True)

    def move_to_joint_pose(
        self,
        pose: PositionOrientation,
        units: AngleUnits,
    ):
        self._motion.joint.add_new_waypoint(
            angle_pose=pose,
            units=units,
        )
        self._motion.mode.set("move")

    def move_to_tcp_pose(
        self,
        pose: PositionOrientation,
        units: AngleUnits,
        coord_system: ActiveCoordinateSystem,
    ):
        cs = self._get_coord_system(coord_system)
        self._motion.linear.add_new_waypoint(
            tcp_pose=pose,
            orientation_units=units,
            coordinate_system=cs,
        )
        self._motion.mode.set("move")

    def offset_to_tcp_pose(
        self,
        offset: PositionOrientation,
        units: AngleUnits,
        coord_system: ActiveCoordinateSystem,
    ):
        cs = self._get_coord_system(coord_system)
        pose = self.get_tcp_pose(units=units, coord_system=coord_system)
        self._motion.linear.add_new_offset(
            waypoint=pose,
            offset=offset,
            orientation_units=units,
            coordinate_system=cs,
        )
        self._motion.mode.set("move")

    def stop(self):
        self._motion.mode.set("hold")

    def get_joints_pose(
        self, units: AngleUnits, coord_system: ActiveCoordinateSystem
    ) -> PositionOrientation:
        cs = self._get_coord_system(coord_system)
        return self._motion.get_actual_position(
            orientation_units=units,
            position_format="joints",
            coordinate_system=cs,
        )

    def get_tcp_pose(
        self, units: AngleUnits, coord_system: ActiveCoordinateSystem
    ) -> PositionOrientation:
        cs = self._get_coord_system(coord_system)
        return self._motion.get_actual_position(
            orientation_units=units,
            position_format="tcp",
            coordinate_system=cs,
        )

    def _get_coord_system(
        self, cs_type: ActiveCoordinateSystem
    ) -> CoordinateSystem:
        if cs_type == "user":
            return self._user_cs or CoordinateSystem((0, 0, 0, 0, 0, 0))
        if cs_type == "tcp":
            pose = self._motion.get_actual_position(
                orientation_units="rad",
                position_format="tcp",
                coordinate_system=CoordinateSystem((0, 0, 0, 0, 0, 0)),
            )
            return CoordinateSystem(pose, orientation_units="rad")
        return CoordinateSystem((0, 0, 0, 0, 0, 0))
