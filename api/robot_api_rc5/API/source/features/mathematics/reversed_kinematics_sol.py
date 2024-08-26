from math import sin, cos, atan2, fabs, pi, radians
import sys
from typing import Tuple, cast
from API.source.models.constants import IK_N_SOL, JOINTS_COUNT
from API.source.models.type_aliases import PositionOrientation


def vector_sub_min_angle(act_q: PositionOrientation, sol_i: PositionOrientation) -> PositionOrientation:
    return cast(PositionOrientation, tuple([min_angle(act_q[i] - sol_i[i]) for i in range(JOINTS_COUNT)]))


def min_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def vector_dot(vector: PositionOrientation) -> float:
    return sum([vector[i] ** 2 for i in range(JOINTS_COUNT)])


def normalise_angles(sol: PositionOrientation, act_q: PositionOrientation) -> PositionOrientation:
    for i in range(JOINTS_COUNT):
        if fabs(sol[i] - act_q[i]) > pi:
            sign = 1 if sol[i] > 0 else -1
            if i == 2:
                max_q = radians(170)
            else:
                max_q = radians(360)
            if fabs(sol[i] - (sign * 2 * pi)) <= max_q:
                sol[i] -= sign * 2 * pi
    return tuple(sol)


def ik_nearest(ik_solution: Tuple[PositionOrientation, ...], act_q: PositionOrientation):
    dqs_min = sys.maxsize
    res = None
    for i in range(IK_N_SOL):
        dqs = vector_dot(vector_sub_min_angle(act_q, ik_solution[i]))
        if dqs < dqs_min:
            dqs_min = dqs
            res = i
    return res


def calculate_solution(solution: Tuple[PositionOrientation, ...], act_q: PositionOrientation) -> PositionOrientation | None:
    sol_index = ik_nearest(solution, act_q)
    if sol_index and sol_index < 8:
        return normalise_angles(solution[sol_index], act_q)
