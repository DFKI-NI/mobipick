"""The painted cover textures of the robot colours: current with the colour table, the DFKI logo unchanged, the panel
around it in the robot's colour, and one cover mesh pair per colour."""
import importlib.util
from pathlib import Path

import numpy as np
from PIL import Image

DESCRIPTION = Path(__file__).resolve().parents[2] / 'mobipick_description'
spec = importlib.util.spec_from_file_location('covers', DESCRIPTION / 'scripts' / 'make_robot_colour_covers.py')
covers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(covers)


def texture(name=None):
    path = covers.MESHES / (covers.TEXTURE if name is None else f'robot_colours/cover_albedo_v2_{name}.png')
    return np.asarray(Image.open(path).convert('RGBA'))


def test_files_are_current_with_the_colour_table():
    assert covers.main(['--check']) == 0


def test_logo_unchanged_and_panel_in_the_colour():
    plain = texture()
    x0, y0, x1, y1 = covers.logo_box(plain)
    for colour in covers.colours():
        painted = texture(colour['name'])
        assert (painted[y0:y1, x0:x1] == plain[y0:y1, x0:x1]).all(), colour['name']
        assert tuple(painted[5, 5, :3]) == covers.rgb(colour), colour['name']   # a white corner of the panel
        assert (painted[..., 3] == plain[..., 3]).all()


def test_every_colour_has_its_meshes_and_material():
    for colour in covers.colours():
        name = colour['name']
        for side in covers.SIDES:
            mesh = (covers.OUT / f'{side}_{name}.obj').read_text()
            assert f'mtllib cover_{name}.mtl' in mesh
            assert [l for l in mesh.splitlines() if l.startswith('v ')] == \
                [l for l in (covers.MESHES / f'{side}.obj').read_text().splitlines() if l.startswith('v ')]
        assert f'map_Kd cover_albedo_v2_{name}.png' in (covers.OUT / f'cover_{name}.mtl').read_text()


def test_xacro_uses_the_coloured_visuals_and_no_material_override():
    model = (DESCRIPTION / 'urdf' / 'mobipick' / 'mobipick_v1_model.urdf.xacro').read_text()
    structure = (DESCRIPTION / 'urdf' / 'structure' / 'mobipick_structure.urdf.xacro').read_text()
    assert 'colour="${robot_colour}"' in model and 'gazebo_material' not in model
    assert structure.count('${cover_visual}cover_') == 4
