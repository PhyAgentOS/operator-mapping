"""Portable Nav2-compatible map snapshots from an already received ROS grid.

This is file serialization, not a mapping algorithm or a new Forge message type.
"""

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class GridSnapshot:
    data: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    frame_id: str
    timestamp_ns: int

    @classmethod
    def from_message(cls, message):
        info = message.info
        if not 0 < info.width * info.height <= 50_000_000:
            raise ValueError("invalid map dimensions")
        data = np.asarray(message.data, dtype=np.int16)
        if data.size != info.width * info.height or np.any((data < -1) | (data > 100)):
            raise ValueError("invalid occupancy grid cells")
        if not np.any(data >= 0):
            raise ValueError("map contains no observed cells")
        p, q = info.origin.position, info.origin.orientation
        if not all(math.isfinite(v) for v in (info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError("non-finite map geometry")
        if info.resolution <= 0 or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 1e-3:
            raise ValueError("invalid map resolution or origin quaternion")
        if abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
            raise ValueError("map origin must be planar")
        if message.header.frame_id != "map":
            raise ValueError("expected map-frame occupancy grid")
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cells = data.astype(np.int8).reshape(info.height, info.width).copy()
        cells.flags.writeable = False
        return cls(
            cells,
            float(info.resolution),
            (float(p.x), float(p.y), yaw),
            message.header.frame_id,
            message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
        )

    def write(self, folder):
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        height, width = self.data.shape
        # Match Nav2's trinary PGM convention. The YAML read thresholds are
        # intentionally fixed to preserve gray=205 as UNKNOWN, not FREE.
        pixels = np.full(self.data.shape, 205, dtype=np.uint8)
        pixels[(self.data >= 0) & (self.data <= 25)] = 254
        pixels[self.data >= 65] = 0
        image = folder / "map.pgm"
        image.write_bytes(
            f"P5\n# Forge occupancy grid snapshot\n{width} {height}\n255\n".encode()
            + np.flipud(pixels).tobytes()
        )
        metadata = {
            "image": image.name,
            "mode": "trinary",
            "resolution": self.resolution,
            "origin": list(self.origin),
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        (folder / "map.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
        # Retain probabilities as well as the navigation-ready trinary map.
        np.savez_compressed(
            folder / "occupancy_grid.npz",
            data=self.data,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            timestamp_ns=self.timestamp_ns,
        )
        return {
            "width": width,
            "height": height,
            "resolution": self.resolution,
            "grid_timestamp_ns": self.timestamp_ns,
            "known_cells": int((self.data >= 0).sum()),
        }
