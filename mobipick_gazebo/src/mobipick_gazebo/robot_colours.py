# Robot colours: the name people see for each robot of a multi-robot sim, by the robot's order, plus the colour that
# paints its body and the stripe on the walls of its room. The ROS namespace stays the robot's real name; the colour is
# only for display. Robot 1 is the namespace without a number (mobipick), robot k the namespace ending in k (mobipick2).
# Standalone use (launch files, scripts): python3 -m mobipick_gazebo.robot_colours <namespace> [field]
import re
import sys
from pathlib import Path
from typing import NamedTuple, Optional, Tuple


class RobotColour(NamedTuple):
    name: str  # colour word shown to people, e.g. 'blue'
    hex: str  # '#rrggbb' for GUI dots
    rgba: str  # 'r g b a' (0..1) for Gazebo/SDF materials
    gazebo_material: str  # material script name of gazebo.material


COLOURS_YAML = 'config/robot_colours.yaml'  # in mobipick_description, shared with the robot description (xacro)


def colours_yaml_path() -> Path:
    """mobipick_description/config/robot_colours.yaml: from the ROS package path when it is found there, else from
    this source tree (both packages live in the same repository), so it also works without ROS (unit tests)."""
    try:
        import rospkg

        path = Path(rospkg.RosPack().get_path('mobipick_description')) / COLOURS_YAML
        if path.is_file():
            return path
    except Exception:  # no rospkg, no ROS_PACKAGE_PATH or package not found
        pass
    return Path(__file__).resolve().parents[3] / 'mobipick_description' / COLOURS_YAML


def load_colours(path=None) -> Tuple[RobotColour, ...]:
    """The colour table, in robot order, from the yaml list 'colours' of {name, hex, rgba, gazebo_material}."""
    import yaml

    with open(path or colours_yaml_path()) as stream:
        entries = (yaml.safe_load(stream) or {}).get('colours') or []
    colours = []
    for entry in entries:
        rgba = entry['rgba']
        if not isinstance(rgba, str):  # also accept a list [r, g, b, a]
            rgba = ' '.join(str(v) for v in rgba)
        colours.append(RobotColour(str(entry['name']), str(entry['hex']), rgba, str(entry['gazebo_material'])))
    return tuple(colours)


COLOURS = load_colours()


def robot_index(namespace: str) -> int:
    """1 for a namespace without a trailing number ('mobipick', '/mobipick/'), k for one ending in k ('mobipick2')."""
    match = re.search(r'(\d+)$', namespace.strip('/'))
    return int(match.group(1)) if match else 1


def robot_colour(namespace_or_index) -> Optional[RobotColour]:
    """The colour of a robot by namespace or 1-based index; None past the last colour."""
    index = namespace_or_index if isinstance(namespace_or_index, int) else robot_index(namespace_or_index)
    return COLOURS[index - 1] if 1 <= index <= len(COLOURS) else None


def display_name(namespace: str) -> str:
    """'blue Mobipick' for 'mobipick', 'orange Mobipick' for 'mobipick2'; the namespace itself without a colour."""
    ns = namespace.strip('/')
    colour = robot_colour(ns)
    base = re.sub(r'\d+$', '', ns)
    return f'{colour.name} {base[:1].upper()}{base[1:]}' if colour and base else ns


def robot_label(namespace: str, robot_count: int) -> str:
    """What a GUI calls one of robot_count running robots: its colour name ('blue Mobipick') when there are two or
    more to tell apart; a single robot is shown by its namespace, with no colour anywhere."""
    return display_name(namespace) if robot_count > 1 else namespace.strip('/')


def title_slug(namespace: str) -> str:
    """Video title part: 'blue-mobipick' (a-z, 0-9 and '-' only)."""
    return re.sub(r'[^a-z0-9]+', '-', display_name(namespace).lower()).strip('-')


def colour_icon(namespace: str, size: int = 12):
    """A round dot in the robot's colour for a Qt combo or label (None without a colour); imports Qt lazily."""
    colour = robot_colour(namespace)
    if colour is None:
        return None
    from python_qt_binding import QtCore, QtGui

    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    painter.setBrush(QtGui.QColor(colour.hex))
    painter.setPen(QtCore.Qt.NoPen)
    painter.drawEllipse(0, 0, size - 1, size - 1)
    painter.end()
    return QtGui.QIcon(pixmap)


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args:
        print('usage: python3 -m mobipick_gazebo.robot_colours <namespace|index> [name|hex|rgba|gazebo_material|display_name|title]')
        return 2
    target = int(args[0]) if args[0].isdigit() else args[0]
    field = args[1] if len(args) > 1 else 'display_name'
    if field in ('display_name', 'title'):
        if not isinstance(target, str):
            return 2
        print(display_name(target) if field == 'display_name' else title_slug(target))
        return 0
    colour = robot_colour(target)
    if colour is None:
        return 1
    print(getattr(colour, field))
    return 0


if __name__ == '__main__':
    sys.exit(main())
