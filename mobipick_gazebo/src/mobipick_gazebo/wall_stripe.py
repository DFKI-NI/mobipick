# A coloured stripe along the walls of a static world mesh: thin visual-only boxes just in front of every vertical wall
# face, in a horizontal band high on the wall. In a multi-robot sim each robot's copy of the world gets the stripe in
# the robot's colour (robot_colours), so a camera image or a Gazebo view tells at once whose room it is.
# Pure functions here (STL loading, face finding, SDF building); nodes/spawn_wall_stripe.py spawns the model.
import math
import re
from pathlib import Path
from typing import List, NamedTuple, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import numpy as np

DEFAULT_BAND_BELOW_TOP = (0.6, 0.2)  # default band: from wall top - 0.6 m to wall top - 0.2 m
DEFAULT_INSET = 0.006  # gap between the wall face and the stripe [m]
DEFAULT_THICKNESS = 0.01  # stripe thickness [m]


class StripeBox(NamedTuple):
    """One stripe piece in the mesh frame: centre, yaw of its outward normal, size along normal/face/height."""
    x: float
    y: float
    z: float
    yaw: float
    thickness: float
    length: float
    height: float


def load_stl(path) -> np.ndarray:
    """Triangles (N, 3, 3) of a binary or ASCII STL file."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = int(np.frombuffer(data, dtype='<u4', count=1, offset=80)[0])
        if 84 + 50 * count == len(data):  # binary: 80 byte header, count, 50 bytes per triangle
            record = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            return np.frombuffer(data, dtype=record, count=count, offset=84)['vertices'].astype(np.float64)
    text = data.decode('ascii', errors='replace')
    numbers = re.findall(r'^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)', text, flags=re.MULTILINE)
    if not numbers or len(numbers) % 3:
        raise ValueError(f'{path}: neither a binary STL nor an ASCII STL with whole triangles')
    return np.array(numbers, dtype=np.float64).reshape(-1, 3, 3)


def resolve_mesh(uri: str) -> Path:
    """A file path for 'package://pkg/rel/path', 'file:///abs/path' or a plain path."""
    if uri.startswith('package://'):
        package, _, relative = uri[len('package://'):].partition('/')
        import rospkg

        return Path(rospkg.RosPack().get_path(package)) / relative
    if uri.startswith('file://'):
        return Path(uri[len('file://'):])
    return Path(uri)


def default_band(triangles: np.ndarray, below_top: Tuple[float, float] = DEFAULT_BAND_BELOW_TOP) -> Tuple[float, float]:
    """(z_low, z_high) of the default band, measured down from the mesh's highest point (the wall tops)."""
    top = float(triangles[:, :, 2].max())
    return top - below_top[0], top - below_top[1]


def wall_faces(triangles: np.ndarray, z_low: float, z_high: float, normal_tol: float = 0.02, plane_tol: float = 0.002,
               merge_tol: float = 0.002) -> List[Tuple[np.ndarray, float, float, float]]:
    """Vertical faces that cover the band [z_low, z_high], as (outward unit normal (2,), plane offset d, u_min, u_max)
    with u along the face (normal rotated +90 deg about z). The triangles of a face (two per rectangle, more with
    T-junction vertices) share a plane and are merged by overlapping extent; faces that only touch in one plane (a
    wall end cap flush with the wall it meets) merge too. Outward follows the STL winding (counter-clockwise)."""
    tri = np.asarray(triangles, dtype=np.float64)
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    keep = lengths > 1e-9  # drop degenerate triangles
    tri, normals = tri[keep], normals[keep] / lengths[keep, None]
    z_min, z_max = tri[:, :, 2].min(axis=1), tri[:, :, 2].max(axis=1)
    eps = 1e-4
    keep = (np.abs(normals[:, 2]) < normal_tol) & (z_min <= z_low + eps) & (z_max >= z_high - eps)
    tri, normals = tri[keep], normals[keep, :2]
    normals /= np.linalg.norm(normals, axis=1)[:, None]

    groups = []  # [normal, d, list of (u_min, u_max)]
    for triangle, normal in zip(tri, normals):
        d = float(normal @ triangle[0, :2])
        u_axis = np.array([-normal[1], normal[0]])
        u = triangle[:, :2] @ u_axis
        for group in groups:
            if group[0] @ normal > 1.0 - 1e-6 and abs(group[1] - d) < plane_tol:
                group[2].append((float(u.min()), float(u.max())))
                break
        else:
            groups.append([normal, d, [(float(u.min()), float(u.max()))]])

    faces = []
    for normal, d, intervals in groups:
        intervals.sort()
        start, end = intervals[0]
        for lo, hi in intervals[1:]:
            if lo <= end + merge_tol:
                end = max(end, hi)
            else:
                faces.append((normal, d, start, end))
                start, end = lo, hi
        faces.append((normal, d, start, end))
    return faces


def stripe_boxes(triangles: np.ndarray, z_low: Optional[float] = None, z_high: Optional[float] = None,
                 inset: float = DEFAULT_INSET, thickness: float = DEFAULT_THICKNESS,
                 min_length: float = 0.05) -> List[StripeBox]:
    """One box per wall face covering the band (default band: default_band), inset in front of the face along its
    outward normal; faces shorter than min_length are skipped."""
    band_low, band_high = default_band(triangles)
    z_low = band_low if z_low is None else z_low
    z_high = band_high if z_high is None else z_high
    if z_high <= z_low:
        raise ValueError(f'empty stripe band: z_low {z_low} >= z_high {z_high}')
    boxes = []
    for normal, d, u_min, u_max in wall_faces(triangles, z_low, z_high):
        length = u_max - u_min
        if length < min_length:
            continue
        u_axis = np.array([-normal[1], normal[0]])
        centre = normal * (d + inset + thickness / 2.0) + u_axis * (u_min + u_max) / 2.0
        boxes.append(StripeBox(float(centre[0]), float(centre[1]), (z_low + z_high) / 2.0,
                               math.atan2(normal[1], normal[0]), thickness, length, z_high - z_low))
    return boxes


def stripe_sdf(boxes: Sequence[StripeBox], model_name: str, gazebo_material: str, offset_x: float = 0.0,
               offset_y: float = 0.0) -> str:
    """A static SDF model with one visual per box (no collision, so lasers and physics never meet it), placed at the
    offset of the world copy, painted with a material script of Gazebo's gazebo.material."""
    material = ('<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
                f'<name>{escape(gazebo_material)}</name></script></material>')
    visuals = '\n'.join(
        f'      <visual name="stripe_{i}">\n'
        f'        <cast_shadows>false</cast_shadows>\n'
        f'        <pose>{b.x:.4f} {b.y:.4f} {b.z:.4f} 0 0 {b.yaw:.6f}</pose>\n'
        f'        <geometry><box><size>{b.thickness:.4f} {b.length:.4f} {b.height:.4f}</size></box></geometry>\n'
        f'        {material}\n'
        f'      </visual>'
        for i, b in enumerate(boxes))
    return (f'<?xml version="1.0"?>\n<sdf version="1.6">\n  <model name="{escape(model_name)}">\n'
            f'    <static>true</static>\n    <pose>{offset_x:.4f} {offset_y:.4f} 0 0 0 0</pose>\n'
            f'    <link name="stripe_link">\n{visuals}\n    </link>\n  </model>\n</sdf>\n')
