from target_explorer.frontier_search import FrontierSearch
from target_explorer.grid_snapshot import GridSnapshot


def make_grid(width, height, resolution, value=0):
    return GridSnapshot.from_values(
        width,
        height,
        resolution,
        [value] * (width * height),
    )


def test_groups_disconnected_unknown_boundaries():
    width = 40
    height = 40
    values = [0] * (width * height)
    for y in (8, 9):
        for x in (8, 9):
            values[y * width + x] = -1
    for y in (28, 29):
        for x in (25, 26):
            values[y * width + x] = -1
    grid = GridSnapshot.from_values(width, height, 0.1, values)

    result = FrontierSearch(
        minimum_frontier_size_m=0.1,
    ).search(grid, 2.0, 2.0)

    assert result.error == ''
    assert len(result.frontiers) == 2
    assert sorted(frontier.size for frontier in result.frontiers) == [4, 4]
    assert all(frontier.goal_options for frontier in result.frontiers)


def test_recovers_when_robot_starts_in_unknown_cell():
    width = 20
    values = [0] * (width * width)
    values[10 * width + 10] = -1
    grid = GridSnapshot.from_values(width, width, 0.1, values)

    result = FrontierSearch(
        minimum_frontier_size_m=0.1,
    ).search(grid, 1.05, 1.05)

    assert result.start_was_recovered
    assert result.reachable_cells == width * width - 1


def test_frontier_goal_options_are_known_free_cells():
    width = 20
    values = [-1] * (width * width)
    for y in range(5, 15):
        for x in range(5, 15):
            values[y * width + x] = 0
    grid = GridSnapshot.from_values(width, width, 0.2, values)

    result = FrontierSearch(
        minimum_frontier_size_m=0.1,
    ).search(grid, 2.0, 2.0)

    assert result.frontiers
    for frontier in result.frontiers:
        for x, y in frontier.goal_options:
            assert grid.value_at_world(x, y) == 0


def test_no_frontiers_on_fully_known_map():
    grid = make_grid(20, 20, 0.1)
    result = FrontierSearch(
        minimum_frontier_size_m=0.1,
    ).search(grid, 1.0, 1.0)

    assert result.frontiers == ()
