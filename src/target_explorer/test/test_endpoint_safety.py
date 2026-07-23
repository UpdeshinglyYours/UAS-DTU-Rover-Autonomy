import math

from target_explorer.endpoint_safety import (
    ClearanceResult,
    combined_clearance,
    obstacle_clearance_in_grid,
)
from target_explorer.grid_snapshot import GridSnapshot


def grid_with_value(value, cell_x=17, cell_y=15):
    width = 40
    values = [0] * (width * width)
    values[cell_y * width + cell_x] = value
    return GridSnapshot.from_values(width, width, 0.1, values)


def test_clearance_is_measured_to_physical_occupied_cell_area():
    grid = grid_with_value(100)

    distance = obstacle_clearance_in_grid(
        grid,
        1.55,
        1.55,
        occupied_threshold=65,
        search_radius_m=1.7,
    )

    assert math.isclose(distance, 0.15, abs_tol=1e-9)


def test_unknown_cells_are_not_obstacles():
    grid = grid_with_value(-1)

    distance = obstacle_clearance_in_grid(
        grid,
        1.55,
        1.55,
        occupied_threshold=65,
        search_radius_m=1.7,
    )

    assert math.isinf(distance)


def test_map_probability_below_threshold_is_not_an_obstacle():
    grid = grid_with_value(64)

    distance = obstacle_clearance_in_grid(
        grid,
        1.55,
        1.55,
        occupied_threshold=65,
        search_radius_m=1.7,
    )

    assert math.isinf(distance)


def test_costmap_inflation_gradient_is_not_a_physical_obstacle():
    grid = grid_with_value(90)

    distance = obstacle_clearance_in_grid(
        grid,
        1.55,
        1.55,
        occupied_threshold=100,
        search_radius_m=1.7,
    )

    assert math.isinf(distance)


def test_clearance_equal_to_requirement_is_unsafe():
    result = ClearanceResult(True, 1.7, '/map')

    assert not result.is_safe(1.7)


def test_combined_clearance_uses_nearest_obstacle_source():
    map_grid = grid_with_value(100, cell_x=30)
    costmap = grid_with_value(100, cell_x=17)

    result = combined_clearance(
        (
            ('/map', map_grid, 1.55, 1.55, 65),
            ('/global_costmap/costmap', costmap, 1.55, 1.55, 100),
        ),
        search_radius_m=1.7,
    )

    assert math.isclose(result.distance, 0.15, abs_tol=1e-9)
    assert result.source == '/global_costmap/costmap'
