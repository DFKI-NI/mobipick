"""Wall stripe of a world copy: STL loading, wall faces in the band, and the visual-only SDF model."""
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from mobipick_gazebo import wall_stripe as ws  # noqa: E402

BAND = (1.9, 2.3)  # default band of a 2.5 m high room


def box_triangles(x0, x1, y0, y1, z0, z1):
    """12 triangles of an axis-aligned box, counter-clockwise seen from outside (STL convention)."""
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], dtype=float)
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return np.array([[v[a], v[b], v[c]] for a, b, c, d in quads for a, b, c in ((a, b, c), (a, c, d))])


def box_room():
    """A 4 x 3 m room of four 0.2 m thick, 2.5 m high walls on a floor slab, plus a 1 m high cabinet."""
    parts = [
        box_triangles(0, 4, 0, 0.2, 0, 2.5),  # south wall, full length
        box_triangles(0, 4, 2.8, 3.0, 0, 2.5),  # north wall, full length
        box_triangles(0, 0.2, 0.2, 2.8, 0, 2.5),  # west wall between them
        box_triangles(3.8, 4, 0.2, 2.8, 0, 2.5),  # east wall between them
        box_triangles(-0.5, 4.5, -0.5, 3.5, -0.05, 0),  # floor: its sides never reach the band
        box_triangles(1, 1.6, 1, 1.4, 0, 1.0),  # cabinet: below the band
    ]
    return np.concatenate(parts)


def write_binary_stl(path, triangles):
    with open(path, 'wb') as stream:
        stream.write(b'synthetic'.ljust(80, b' ') + struct.pack('<I', len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', *triangle.ravel()) + b'\0\0')


def write_ascii_stl(path, triangles):
    lines = ['solid room']
    for triangle in triangles:
        lines += ['  facet normal 0 0 0', '    outer loop']
        lines += [f'      vertex {x:.6f} {y:.6f} {z:.6f}' for x, y, z in triangle]
        lines += ['    endloop', '  endfacet']
    Path(path).write_text('\n'.join(lines + ['endsolid room', '']))


def visuals(sdf):
    root = ET.fromstring(sdf)
    return root, root.findall('.//visual')


def check_stripe_model(sdf, z_low):
    root, items = visuals(sdf)
    assert root.find('.//collision') is None and root.find('.//inertial') is None
    assert root.find('model/static').text == 'true'
    for visual in items:
        z = float(visual.find('pose').text.split()[2])
        height = float(visual.find('geometry/box/size').text.split()[2])
        assert z - height / 2 >= z_low - 1e-6
    return items


def test_binary_and_ascii_stl_load_the_same(tmp_path):
    room = box_room()
    write_binary_stl(tmp_path / 'room_bin.stl', room)
    write_ascii_stl(tmp_path / 'room_ascii.stl', room)
    assert np.allclose(ws.load_stl(tmp_path / 'room_bin.stl'), room, atol=1e-5)
    assert np.allclose(ws.load_stl(tmp_path / 'room_ascii.stl'), room, atol=1e-5)
    (tmp_path / 'bad.stl').write_text('not a mesh')
    with pytest.raises(ValueError):
        ws.load_stl(tmp_path / 'bad.stl')


def test_default_band_hangs_from_the_wall_top():
    assert ws.default_band(box_room()) == pytest.approx(BAND)


def test_box_room_stripes():
    boxes = ws.stripe_boxes(box_room())
    # 4 inner faces, 4 outer faces (the end caps of the long walls merge into the west and east outer faces) and the
    # 4 end caps of the short walls, hidden in the long walls; floor and cabinet stay bare
    assert len(boxes) == 12
    assert all(b.height == pytest.approx(0.4) and b.z == pytest.approx(2.1) for b in boxes)
    inner_south = [b for b in boxes if b.yaw == pytest.approx(math.pi / 2) and b.y == pytest.approx(0.211)]
    assert len(inner_south) == 1 and inner_south[0].length == pytest.approx(4.0) and inner_south[0].x == pytest.approx(2)
    outer_west = [b for b in boxes if abs(b.yaw) == pytest.approx(math.pi) and b.x == pytest.approx(-0.011)]
    assert len(outer_west) == 1 and outer_west[0].length == pytest.approx(3.0)
    inner_east = [b for b in boxes if abs(b.yaw) == pytest.approx(math.pi) and b.x == pytest.approx(3.789)]
    assert len(inner_east) == 1 and inner_east[0].length == pytest.approx(2.6)


def test_explicit_band_and_empty_band():
    boxes = ws.stripe_boxes(box_room(), z_low=0.5, z_high=0.9)
    assert len(boxes) == 12 + 4  # the cabinet's four sides now reach the band
    assert ws.stripe_boxes(box_room(), z_low=2.4, z_high=2.6) == []  # above the walls
    with pytest.raises(ValueError):
        ws.stripe_boxes(box_room(), z_low=2.0, z_high=2.0)


def test_stripe_sdf_is_visual_only_and_placed_at_the_copy():
    boxes = ws.stripe_boxes(box_room())
    sdf = ws.stripe_sdf(boxes, 'mobipick2__wall_stripe', 'Gazebo/Orange', offset_x=50.0)
    items = check_stripe_model(sdf, BAND[0])
    assert len(items) == 12
    root = ET.fromstring(sdf)
    assert root.find('model').get('name') == 'mobipick2__wall_stripe'
    assert root.find('model/pose').text.split()[:2] == ['50.0000', '0.0000']
    assert {v.find('material/script/name').text for v in items} == {'Gazebo/Orange'}
    assert all(v.find('cast_shadows').text == 'false' for v in items)


def find_pbr_mesh(name):
    """A mesh of the pbr_gazebo package (pbr_tools repository) if it is around: from ROS, or next to this repository
    in the same workspace src folder."""
    try:
        import rospkg

        candidates = [Path(rospkg.RosPack().get_path('pbr_gazebo'))]
    except Exception:
        candidates = []
    candidates.append(Path(__file__).resolve().parents[3] / 'pbr_tools' / 'pbr_gazebo')
    if os.environ.get('PBR_GAZEBO_DIR'):
        candidates.insert(0, Path(os.environ['PBR_GAZEBO_DIR']))
    for package in candidates:
        path = package / 'Media' / 'models' / name
        if path.is_file():
            return path
    pytest.skip(f'{name} of pbr_gazebo not found')


@pytest.mark.parametrize('name, top, min_boxes', [('pbr_robot_lab.stl', 2.6, 10), ('pbr_cic.stl', 2.078, 50)])
def test_real_world_meshes(name, top, min_boxes):
    triangles = ws.load_stl(find_pbr_mesh(name))
    z_low, z_high = ws.default_band(triangles)
    assert (z_low, z_high) == pytest.approx((top - 0.6, top - 0.2), abs=1e-3)
    boxes = ws.stripe_boxes(triangles)
    assert len(boxes) >= min_boxes
    assert all(b.z - b.height / 2 >= z_low - 1e-6 and b.length >= 0.05 for b in boxes)
    check_stripe_model(ws.stripe_sdf(boxes, 'mobipick__wall_stripe', 'Gazebo/Blue'), z_low)
