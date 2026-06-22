"""
Рассчитать параметры системы координат по трём известным точкам. Использует
модуль API, не зависящий от подключения к роботу.
"""

from API.coords import CoordinateSystem, calculate_plane_from_points


def get_coordinate_system() -> CoordinateSystem:
    """
    Рассчитать систему координат по трём точкам.
    """

    # Рассчитываем параметры системы координат по 3 известным точкам.
    calculated_coordinate_system = calculate_plane_from_points(
        pO=[-0.62949, -0.48229, 0.00739],
        pX=[-0.28411, -0.11155, 0.00842],
        pY=[-0.14781, -0.92954, 0.00827],
    )

    # Преобразуем полученные параметры в объект класса CoordinateSystem
    coordinate_system = CoordinateSystem(calculated_coordinate_system)
    return coordinate_system


if __name__ == "__main__":
    # Запуск определенной выше функции
    print(get_coordinate_system())
