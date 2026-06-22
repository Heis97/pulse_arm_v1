from ..classes.enum_classes.controller_commands import ControllerCommands
from ..constants import (
    DIGITAL_IO_INDEX_COUNT,
    JOINTS_COUNT,
    WRIST_MAX_DIG_IN,
    WRIST_MAX_DIG_OUT,
)
from .command_registry import CommandRegistry, CommandSpec


def init_controller_commands() -> None:
    # Common
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_MOVE_ADD_WP,
            pack_fmt="i6d6d6d6BB7di0q",
            unpack_fmt="i6d",
        )
    )

    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_PAUSE,
            pack_fmt="B",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_ENABLE_REALTIME,
        )
    )

    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_POWER,
            pack_fmt="i",
        )
    )

    CommandRegistry.register(
        CommandSpec(ControllerCommands.CTRLR_COMS_CTRL_DISCONNECT)
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_JOG,
            pack_fmt="i6B6d6d6d6d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_JOINT_JOG,
            pack_fmt=f"i {JOINTS_COUNT}b2x",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_ZG,
            pack_fmt="B",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_CONFIRM_POSITION,
            unpack_fmt="i",
        ),
    )

    ## RPMP
    CommandRegistry.register(
        # Length of each rpmp add WP message should be equal to greatest one,
        # fill in the additional size of another messages with 'x'
        CommandSpec(
            ControllerCommands.CTRLR_COMS_RPMP_MOVE_ADD_WP,
            pack_fmt="@id 3d3ddddd 120x",
            unpack_fmt="i6d",
            variant_pack_formats={
                "l": "@id 3d3ddddd 120x",
                "p": "@id 3d3ddddd 120x",
                "c": "@id 3d3ddddd 3d3ddddd 40x",
                "j": "@id 6d6d6d 56x",
                "j_tcp": "@id 3d 3d 6d i 6d 6d",
            },
        )
    )

    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_UNLOCK,
            pack_fmt="I",
            unpack_fmt="B",
        )
    )

    # Getters
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_LAST_POS,
            unpack_fmt=f"{JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_MOVE_SCALE,
            unpack_fmt="dd",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_GRAVITY,
            unpack_fmt="3d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_SFTY_LIMITS,
            unpack_fmt=f"d {JOINTS_COUNT}d {JOINTS_COUNT * 2}d 4d" * 2,
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_PAYLOAD,
            unpack_fmt="4d",
        ),
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_TOOL,
            unpack_fmt="6d",
        ),
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_HOME_POSE,
            unpack_fmt=f"{JOINTS_COUNT}d",
        ),
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_JOINT_POS_LIMITS,
            unpack_fmt=f"{JOINTS_COUNT}d {JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_JOINT_VEL_LIMITS,
            unpack_fmt=f"{JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_JOINT_CURRENT_LIM,
            unpack_fmt=f"{JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_JOINT_NOMINAL_TRQ,
            unpack_fmt=f"{JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_JOINT_MAX_TRQ,
            unpack_fmt=f"{JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_AN_OUT_LIMITS,
            unpack_fmt="2d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_AN_IN_LIMITS,
            unpack_fmt="2d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_MAX_PAYLOAD,
            unpack_fmt="1d",
        )
    )

    ## Kinematics
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_FKINE,
            pack_fmt=f"{JOINTS_COUNT}d",
            unpack_fmt="i 6d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_IKINE,
            pack_fmt="6d",
            unpack_fmt=f"i {JOINTS_COUNT * 8}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_IKINE_OPTIMAL,
            pack_fmt=f"6d {JOINTS_COUNT}d",
            unpack_fmt=f"i {JOINTS_COUNT}d",
        )
    )

    ## Service
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_SW_VERSION,
            unpack_fmt="3I",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_PROTO_VERSION,
            unpack_fmt="I",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_ROBOT_VIEW_INFO,
            unpack_fmt="=L32c",
        )
    )

    ## IO and Wrist
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_DIG_INPUT_FUNC,
            unpack_fmt="i" * (DIGITAL_IO_INDEX_COUNT + WRIST_MAX_DIG_IN),
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_GET_DIG_OUTPUT_FUNC,
            unpack_fmt="i" * (DIGITAL_IO_INDEX_COUNT + WRIST_MAX_DIG_OUT),
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_CBOX_GET_SFTY_INPUT_FUNC,
            unpack_fmt="8i",
        )
    )

    # Setters
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_OUTPUTS,
            pack_fmt="24B4d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_MOVE_SCALE,
            pack_fmt="dd",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_GRAVITY,
            pack_fmt="3d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_SFTY_LIMITS,
            pack_fmt=f"d {JOINTS_COUNT}d {JOINTS_COUNT * 2}d 4d" * 2,
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_PAYLOAD,
            pack_fmt="4d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_TOOL,
            pack_fmt="6d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_JOG_PARAM,
            pack_fmt=f"B {JOINTS_COUNT}B {JOINTS_COUNT}d {JOINTS_COUNT}d {JOINTS_COUNT}d {JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_HOME_POSE,
            pack_fmt=f"{JOINTS_COUNT}d",
        )
    )
    CommandRegistry.register(
        CommandSpec(ControllerCommands.CTRLR_COMS_STORE_SETTINGS)
    )

    ## IO
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_DIG_INPUT_FUNC,
            pack_fmt="1b3xi",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_DIG_OUTPUT_FUNC,
            pack_fmt="1b3xi",
        )
    )

    ## Wrist
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_WRIST_IO,
            pack_fmt="7B",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_WRIST_DIG_INPUT_FUNC,
            pack_fmt="1b3xi",
        )
    )
    CommandRegistry.register(
        CommandSpec(
            ControllerCommands.CTRLR_COMS_SET_WRIST_DIG_OUTPUT_FUNC,
            pack_fmt="1b3xi",
        )
    )
