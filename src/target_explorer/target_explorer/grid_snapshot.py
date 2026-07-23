"""Immutable, ROS-independent occupancy-grid snapshots."""

import math
from array import array
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class GridSnapshot:
    """The minimum grid state needed by the exploration worker."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str
    data: array

    @classmethod
    def from_occupancy_grid(cls, message) -> 'GridSnapshot':
        orientation = message.info.origin.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return cls(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=math.atan2(siny_cosp, cosy_cosp),
            frame_id=str(message.header.frame_id),
            data=array('b', message.data),
        )

    @classmethod
    def from_values(
        cls,
        width: int,
        height: int,
        resolution: float,
        values: Iterable[int],
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        frame_id: str = 'map',
    ) -> 'GridSnapshot':
        """Build a snapshot without ROS messages (used by tests/benchmarks)."""
        # ROS OccupancyGrid storage is signed int8, while Nav2 cost constants
        # are conventionally written as uint8 values 253--255. Accept either
        # spelling in tests and pure helpers without changing map semantics.
        signed_values = (
            value - 256 if 128 <= value <= 255 else value
            for value in values
        )
        return cls(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=0.0,
            frame_id=frame_id,
            data=array('b', signed_values),
        )

    @property
    def cell_count(self) -> int:
        return self.width * self.height

    def contains(self, mx: int, my: int) -> bool:
        return 0 <= mx < self.width and 0 <= my < self.height

    def index(self, mx: int, my: int) -> int:
        return my * self.width + mx

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.resolution <= 0.0:
            return None
        dx = x - self.origin_x
        dy = y - self.origin_y
        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        mx = math.floor(local_x / self.resolution)
        my = math.floor(local_y / self.resolution)
        if not self.contains(mx, my):
            return None
        return int(mx), int(my)

    def world_to_index(self, x: float, y: float) -> Optional[int]:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        return self.index(cell[0], cell[1])

    def cell_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        local_x = (mx + 0.5) * self.resolution
        local_y = (my + 0.5) * self.resolution
        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        return (
            self.origin_x + cos_yaw * local_x - sin_yaw * local_y,
            self.origin_y + sin_yaw * local_x + cos_yaw * local_y,
        )

    def index_to_world(self, index: int) -> Tuple[float, float]:
        return self.cell_to_world(index % self.width, index // self.width)

    def value_at_world(self, x: float, y: float) -> Optional[int]:
        index = self.world_to_index(x, y)
        return None if index is None else int(self.data[index])
