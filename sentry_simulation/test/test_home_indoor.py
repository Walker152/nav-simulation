"""Geometry contracts for the two-storey house, independent of ROS."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

from PIL import Image
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / "resource/models/home_indoor/model.sdf"


def test_house_is_registered_and_has_local_assets():
    catalog = yaml.safe_load((PACKAGE / "config/worlds.yaml").read_text())
    house = catalog["home_indoor"]
    assert (PACKAGE / house["world"]).is_file()
    assert (PACKAGE / house["map"]).is_file()
    assert house["icp"]["enable"] is False
    root = ET.parse(MODEL).getroot()
    assert root.findtext("model/static") == "true"
    for uri in root.iter("uri"):
        assert uri.text.startswith("model://home_indoor/")
        assert (MODEL.parent / uri.text.removeprefix("model://home_indoor/")).is_file()
    # Every visible item represents a real obstacle / support surface.
    for link in root.findall("model/link"):
        assert len(link.findall("visual")) == len(link.findall("collision"))


def test_ramp_surfaces_meet_landings_without_steps():
    # Read the actual packaged mesh, not the generator's constants.
    surfaces = []
    for name in ("ramp_lower", "ramp_upper"):
        path = MODEL.parent / "meshes" / f"{name}.stl"
        vertices = {tuple(map(float, line.split()[1:]))
                    for line in path.read_text().splitlines()
                    if line.strip().startswith("vertex ")}
        y0, y1 = min(v[1] for v in vertices), max(v[1] for v in vertices)
        ends = [max(v[2] for v in vertices if v[1] == y) for y in (y0, y1)]
        assert abs(math.atan2(ends[1] - ends[0], y1 - y0)) < math.radians(11)
        assert max(v[0] for v in vertices) - min(v[0] for v in vertices) == pytest.approx(1.2)
        surfaces.append((y0, y1, *ends))
    assert surfaces[0] == pytest.approx((1.6, 9.1, 0.0, 1.3))
    assert surfaces[1] == pytest.approx((1.6, 9.1, 2.6, 1.3))
    root = ET.parse(MODEL).getroot()
    for name, top in (("landing_middle", 1.3), ("landing_upper", 2.6)):
        link = root.find(f"model/link[@name='{name}']")
        pose = list(map(float, link.findtext("pose").split()))
        size = list(map(float, link.findtext("collision/geometry/box/size").split()))
        assert pose[2] + size[2] / 2 == pytest.approx(top)


@pytest.mark.parametrize("floor", [0, 1])
def test_floor_maps_leave_doors_and_spawn_clear_but_block_furniture(floor):
    metadata = yaml.safe_load((PACKAGE / f"maps/home_indoor{'_upper' if floor else ''}.yaml").read_text())
    grid = Image.open(PACKAGE / "maps" / metadata["image"])

    def pixel(x, y):
        return grid.getpixel((int(x / metadata["resolution"]),
                              grid.height - 1 - int(y / metadata["resolution"])))

    for x, y in ((5, 1.2), (4.2, 2.25), (4.2, 6.75), (5.8, 2.25), (5.8, 6.75), (10, 0.8)):
        # A 0.7 m wide footprint must fit through every door.
        for dx in (-0.35, 0, 0.35):
            for dy in (-0.35, 0, 0.35):
                assert pixel(x + dx, y + dy) == 254, (floor, x, y, dx, dy)
    assert pixel(4.2, 3.5) == 0  # solid partition
    assert pixel(1.1, 2.4) == 0  # sofa / bed
    assert pixel(10.8, 7) != 254  # a 2-D floor map must not flatten the ramp
