"""Robot colours by order: display names, the namespace to index mapping and the command line used by launch files."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from mobipick_gazebo import robot_colours as rc  # noqa: E402


def test_index_from_namespace():
    assert rc.robot_index('mobipick') == 1
    assert rc.robot_index('/mobipick/') == 1
    assert rc.robot_index('mobipick2') == 2
    assert rc.robot_index('/mobipick3') == 3


def test_display_names_by_order():
    assert [rc.display_name(ns) for ns in ('mobipick', 'mobipick2', 'mobipick3', 'mobipick4')] == [
        'blue Mobipick', 'orange Mobipick', 'green Mobipick', 'purple Mobipick']
    assert rc.display_name('/mobipick2/') == 'orange Mobipick'


def test_past_the_last_colour_the_namespace_stays():
    assert rc.robot_colour('mobipick5') is None
    assert rc.display_name('mobipick5') == 'mobipick5'
    assert rc.colour_icon('mobipick5') is None


def test_video_title_part():
    assert rc.title_slug('mobipick') == 'blue-mobipick'
    assert rc.title_slug('mobipick3') == 'green-mobipick'


def test_every_colour_has_a_gazebo_material_and_an_rgba():
    for colour in rc.COLOURS:
        assert colour.gazebo_material.startswith('Gazebo/')
        assert len(colour.rgba.split()) == 4 and all(0 <= float(v) <= 1 for v in colour.rgba.split())
        assert len(colour.hex) == 7 and colour.hex.startswith('#')


def test_command_line(capsys):
    assert rc.main(['mobipick2', 'gazebo_material']) == 0
    assert capsys.readouterr().out.strip() == 'Gazebo/Orange'
    assert rc.main(['3', 'rgba']) == 0
    assert capsys.readouterr().out.strip() == rc.COLOURS[2].rgba
    assert rc.main(['mobipick', 'title']) == 0
    assert capsys.readouterr().out.strip() == 'blue-mobipick'
    assert rc.main(['mobipick9', 'hex']) == 1


def test_table_comes_from_the_shared_yaml():
    path = rc.colours_yaml_path()
    assert path.is_file() and path.parent.parent.name == 'mobipick_description'
    assert rc.load_colours(path) == rc.COLOURS
    assert [c.name for c in rc.COLOURS] == ['blue', 'orange', 'green', 'purple']
    assert [c.gazebo_material for c in rc.COLOURS] == ['Gazebo/Blue', 'Gazebo/Orange', 'Gazebo/Green', 'Gazebo/Purple']


def test_load_colours_from_another_yaml(tmp_path):
    path = tmp_path / 'colours.yaml'
    path.write_text("colours:\n"
                    "  - {name: red, hex: '#ff0000', rgba: '1 0 0 1', gazebo_material: Gazebo/Red}\n"
                    "  - {name: grey, hex: '#808080', rgba: [0.5, 0.5, 0.5, 1], gazebo_material: Gazebo/Grey}\n")
    colours = rc.load_colours(path)
    assert colours[0] == rc.RobotColour('red', '#ff0000', '1 0 0 1', 'Gazebo/Red')
    assert colours[1].rgba == '0.5 0.5 0.5 1'
    empty = tmp_path / 'empty.yaml'
    empty.write_text('')
    assert rc.load_colours(empty) == ()
