"""Reachable-space wavefront frontier detection and clustering."""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

from .grid_snapshot import GridSnapshot


@dataclass(frozen=True)
class Frontier:
    """A connected free/unknown boundary and free-side goal samples."""

    size: int
    centroid_x: float
    centroid_y: float
    goal_options: Tuple[Tuple[float, float], ...] = field(
        default_factory=tuple,
    )
    cells: Tuple[int, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class FrontierSearchResult:
    frontiers: Tuple[Frontier, ...]
    reachable_cells: int
    start_was_recovered: bool = False
    error: str = ''


class FrontierSearch:
    """Find frontiers adjacent to the robot's known-free map component."""

    def __init__(
        self,
        minimum_frontier_size_m: float = 0.5,
        maximum_goal_samples_per_frontier: int = 12,
        free_cell_maximum: int = 0,
        start_recovery_radius_m: float = 2.0,
    ) -> None:
        self.minimum_frontier_size_m = max(
            0.0,
            minimum_frontier_size_m,
        )
        self.maximum_goal_samples_per_frontier = max(
            1,
            maximum_goal_samples_per_frontier,
        )
        self.free_cell_maximum = max(0, free_cell_maximum)
        self.start_recovery_radius_m = max(0.0, start_recovery_radius_m)

    def search(
        self,
        grid: GridSnapshot,
        robot_x: float,
        robot_y: float,
    ) -> FrontierSearchResult:
        if grid.cell_count == 0 or len(grid.data) < grid.cell_count:
            return FrontierSearchResult((), 0, error='invalid map')

        start_index = grid.world_to_index(robot_x, robot_y)
        if start_index is None:
            return FrontierSearchResult((), 0, error='robot outside map')

        recovered = False
        if not self._is_free(grid, start_index):
            start_index = self._nearest_free(grid, start_index)
            recovered = True
        if start_index is None:
            return FrontierSearchResult(
                (),
                0,
                start_was_recovered=True,
                error='no nearby reachable free cell',
            )

        reachable = self._reachable_cells(grid, start_index)
        frontier_mask = self._frontier_mask(grid, reachable)
        clusters = self._clusters(grid, frontier_mask)
        frontiers = []
        for cells in clusters:
            if len(cells) * grid.resolution < self.minimum_frontier_size_m:
                continue
            free_side = self._free_side_cells(grid, reachable, cells)
            if not free_side:
                continue
            points = [grid.index_to_world(index) for index in cells]
            centroid_x = sum(point[0] for point in points) / len(points)
            centroid_y = sum(point[1] for point in points) / len(points)
            sampled = self._sample_goal_cells(
                grid,
                free_side,
                centroid_x,
                centroid_y,
                robot_x,
                robot_y,
            )
            frontiers.append(Frontier(
                size=len(cells),
                centroid_x=centroid_x,
                centroid_y=centroid_y,
                goal_options=tuple(
                    grid.index_to_world(index) for index in sampled
                ),
                cells=tuple(cells),
            ))

        return FrontierSearchResult(
            tuple(frontiers),
            int(np.count_nonzero(reachable)),
            recovered,
        )

    def _is_free(self, grid: GridSnapshot, index: int) -> bool:
        value = int(grid.data[index])
        return 0 <= value <= self.free_cell_maximum

    def _nearest_free(
        self,
        grid: GridSnapshot,
        start_index: int,
    ) -> Optional[int]:
        radius_limit = int(math.ceil(
            self.start_recovery_radius_m / grid.resolution
        ))
        start_x = start_index % grid.width
        start_y = start_index // grid.width
        for radius in range(1, radius_limit + 1):
            min_x = max(0, start_x - radius)
            max_x = min(grid.width - 1, start_x + radius)
            min_y = max(0, start_y - radius)
            max_y = min(grid.height - 1, start_y + radius)
            for x in range(min_x, max_x + 1):
                for y in (min_y, max_y):
                    index = grid.index(x, y)
                    if self._is_free(grid, index):
                        return index
            for y in range(min_y + 1, max_y):
                for x in (min_x, max_x):
                    index = grid.index(x, y)
                    if self._is_free(grid, index):
                        return index
        return None

    def _reachable_cells(
        self,
        grid: GridSnapshot,
        start_index: int,
    ) -> np.ndarray:
        values = np.frombuffer(grid.data, dtype=np.int8).reshape(
            grid.height,
            grid.width,
        )
        free = (values >= 0) & (values <= self.free_cell_maximum)
        runs = []
        parents: List[int] = []
        ranks: List[int] = []
        previous = []
        start_x = start_index % grid.width
        start_y = start_index // grid.width
        start_run = None

        def find(run_id: int) -> int:
            while parents[run_id] != run_id:
                parents[run_id] = parents[parents[run_id]]
                run_id = parents[run_id]
            return run_id

        def union(first: int, second: int) -> None:
            first_root = find(first)
            second_root = find(second)
            if first_root == second_root:
                return
            if ranks[first_root] < ranks[second_root]:
                first_root, second_root = second_root, first_root
            parents[second_root] = first_root
            if ranks[first_root] == ranks[second_root]:
                ranks[first_root] += 1

        padded = np.empty(grid.width + 2, dtype=np.bool_)
        for y, row in enumerate(free):
            padded[0] = False
            padded[-1] = False
            padded[1:-1] = row
            edges = np.flatnonzero(padded[1:] != padded[:-1])
            current = []
            for edge_index in range(0, len(edges), 2):
                x0 = int(edges[edge_index])
                x1 = int(edges[edge_index + 1] - 1)
                run_id = len(runs)
                runs.append((y, x0, x1))
                parents.append(run_id)
                ranks.append(0)
                current.append((x0, x1, run_id))
                if y == start_y and x0 <= start_x <= x1:
                    start_run = run_id

            previous_index = 0
            current_index = 0
            while (
                previous_index < len(previous)
                and current_index < len(current)
            ):
                prev_x0, prev_x1, prev_run = previous[previous_index]
                curr_x0, curr_x1, curr_run = current[current_index]
                if prev_x1 < curr_x0:
                    previous_index += 1
                elif curr_x1 < prev_x0:
                    current_index += 1
                else:
                    union(prev_run, curr_run)
                    if prev_x1 <= curr_x1:
                        previous_index += 1
                    if curr_x1 <= prev_x1:
                        current_index += 1
            previous = current

        reachable = np.zeros_like(free)
        if start_run is None:
            return reachable
        start_root = find(start_run)
        for run_id, (y, x0, x1) in enumerate(runs):
            if find(run_id) == start_root:
                reachable[y, x0:x1 + 1] = True
        return reachable

    def _frontier_mask(
        self,
        grid: GridSnapshot,
        reachable: np.ndarray,
    ) -> np.ndarray:
        adjacent_to_reachable = np.zeros_like(reachable)
        adjacent_to_reachable[1:] |= reachable[:-1]
        adjacent_to_reachable[:-1] |= reachable[1:]
        adjacent_to_reachable[:, 1:] |= reachable[:, :-1]
        adjacent_to_reachable[:, :-1] |= reachable[:, 1:]
        values = np.frombuffer(grid.data, dtype=np.int8).reshape(
            grid.height,
            grid.width,
        )
        return (values < 0) & adjacent_to_reachable

    def _clusters(
        self,
        grid: GridSnapshot,
        frontier_mask: np.ndarray,
    ) -> List[List[int]]:
        visited = np.zeros_like(frontier_mask)
        output: List[List[int]] = []
        for y, x in np.argwhere(frontier_mask):
            if visited[y, x]:
                continue
            visited[y, x] = True
            queue = deque([(int(x), int(y))])
            cluster = []
            while queue:
                cell_x, cell_y = queue.popleft()
                cluster.append(grid.index(cell_x, cell_y))
                for next_y in range(
                    max(0, cell_y - 1),
                    min(grid.height - 1, cell_y + 1) + 1,
                ):
                    for next_x in range(
                        max(0, cell_x - 1),
                        min(grid.width - 1, cell_x + 1) + 1,
                    ):
                        if (
                            frontier_mask[next_y, next_x]
                            and not visited[next_y, next_x]
                        ):
                            visited[next_y, next_x] = True
                            queue.append((next_x, next_y))
            output.append(cluster)
        return output

    def _free_side_cells(
        self,
        grid: GridSnapshot,
        reachable: np.ndarray,
        frontier_cells: Sequence[int],
    ) -> Set[int]:
        output: Set[int] = set()
        for index in frontier_cells:
            x = index % grid.width
            y = index // grid.width
            for nx, ny in (
                (x - 1, y),
                (x + 1, y),
                (x, y - 1),
                (x, y + 1),
            ):
                if (
                    0 <= nx < grid.width
                    and 0 <= ny < grid.height
                    and reachable[ny, nx]
                ):
                    output.add(grid.index(nx, ny))
        return output

    def _sample_goal_cells(
        self,
        grid: GridSnapshot,
        free_side: Set[int],
        centroid_x: float,
        centroid_y: float,
        robot_x: float,
        robot_y: float,
    ) -> List[int]:
        ordered = sorted(free_side)
        if len(ordered) <= self.maximum_goal_samples_per_frontier:
            return ordered

        nearest_robot = min(
            ordered,
            key=lambda index: _distance_from_index(
                grid,
                index,
                robot_x,
                robot_y,
            ),
        )
        nearest_centroid = min(
            ordered,
            key=lambda index: _distance_from_index(
                grid,
                index,
                centroid_x,
                centroid_y,
            ),
        )
        sample = [nearest_robot]
        if nearest_centroid != nearest_robot:
            sample.append(nearest_centroid)
        remaining = self.maximum_goal_samples_per_frontier - len(sample)
        stride = max(1, len(ordered) // max(1, remaining))
        for index in ordered[::stride]:
            if index not in sample:
                sample.append(index)
            if len(sample) == self.maximum_goal_samples_per_frontier:
                break
        return sample


def _distance_from_index(
    grid: GridSnapshot,
    index: int,
    x: float,
    y: float,
) -> float:
    world_x, world_y = grid.index_to_world(index)
    return math.hypot(world_x - x, world_y - y)
