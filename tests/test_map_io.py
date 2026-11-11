from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml
from PIL import Image

from mapping.map_io import GridSnapshot


def message(data=None):
    return NS(
        data=[0, 100, -1, 40, 25, 65] if data is None else data,
        header=NS(frame_id="map", stamp=NS(sec=123, nanosec=456)),
        info=NS(
            width=3,
            height=2,
            resolution=0.05,
            origin=NS(
                position=NS(x=-2.0, y=3.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
        ),
    )


def test_serialized_map_preserves_unknown_free_occupied_and_origin(tmp_path):
    snapshot = GridSnapshot.from_message(message())
    geometry = snapshot.write(tmp_path)
    config = yaml.safe_load((tmp_path / "map.yaml").read_text())
    image = np.asarray(Image.open(tmp_path / config["image"]))
    assert image.tolist() == [[205, 254, 0], [254, 0, 205]]
    # Emulate the official Nav2 trinary loader's thresholds. Unknown cells must
    # remain unknown after export, including the flipped bottom/upper ROS rows.
    occupancy = (255 - image.astype(float)) / 255
    restored = np.full(image.shape, -1)
    restored[occupancy >= config["occupied_thresh"]] = 100
    restored[occupancy <= config["free_thresh"]] = 0
    assert np.flipud(restored).tolist() == [[0, 100, -1], [-1, 0, 100]]
    assert config["origin"] == [-2.0, 3.0, 0.0]
    assert config["resolution"] == 0.05
    assert geometry["grid_timestamp_ns"] == 123000000456
    with np.load(tmp_path / "occupancy_grid.npz") as raw:
        assert raw["data"].tolist() == [[0, 100, -1], [40, 25, 65]]


def test_snapshot_cannot_change_when_another_map_arrives(tmp_path):
    source = message()
    snapshot = GridSnapshot.from_message(source)
    source.data[0] = 100
    assert snapshot.data[0, 0] == 0
    with pytest.raises(ValueError):
        snapshot.data[0, 0] = 100


@pytest.mark.parametrize("data", [[-1] * 6, [0] * 5, [0, 0, 0, 0, 0, 101]])
def test_invalid_or_unobserved_grid_cannot_be_reported_saved(data):
    with pytest.raises(ValueError):
        GridSnapshot.from_message(message(data))
