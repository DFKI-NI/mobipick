#!/usr/bin/env python3
"""Make the painted cover meshes of the robot colours of a multi-robot simulation.

For every colour of config/robot_colours.yaml this writes, next to the cover meshes in
meshes/structure/visual/v2/robot_colours/: the cover texture with its white panel painted in the colour (the DFKI logo,
the dark box with its letters, stays as it is), a material file using that texture, and copies of the two side cover
meshes using that material. The robot description picks them by the xacro arg robot_colour_index.

Run it again after changing the colour table or the cover texture; --check only reports whether the files are current.
Usage: make_robot_colour_covers.py [--check]
"""
import argparse
import io
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

PACKAGE = Path(__file__).resolve().parent.parent
MESHES = PACKAGE / 'meshes' / 'structure' / 'visual' / 'v2'
OUT = MESHES / 'robot_colours'
TEXTURE = 'cover_albedo_v2.png'
SIDES = ('cover_short_side', 'cover_long_side')
LOGO_MARGIN = 0  # pixels kept around the logo's dark box: none, its anti-aliased edge blends into the colour


def colours(path=PACKAGE / 'config' / 'robot_colours.yaml'):
    with open(path) as stream:
        return (yaml.safe_load(stream) or {}).get('colours') or []


def rgb(colour):
    """'#1f6fd1' -> (31, 111, 209)."""
    value = colour['hex'].lstrip('#')
    return tuple(int(value[i:i + 2], 16) for i in (0, 2, 4))


def logo_box(pixels):
    """(x0, y0, x1, y1) of the texture's dark pixels (the logo), widened by LOGO_MARGIN."""
    dark = pixels[..., :3].mean(axis=-1) < 128
    ys, xs = np.nonzero(dark)
    if not len(xs):
        raise ValueError(f'{TEXTURE} holds no dark logo pixels')
    h, w = dark.shape
    return (max(xs.min() - LOGO_MARGIN, 0), max(ys.min() - LOGO_MARGIN, 0),
            min(xs.max() + LOGO_MARGIN + 1, w), min(ys.max() + LOGO_MARGIN + 1, h))


def painted(pixels, colour):
    """The texture with every pixel outside the logo box tinted: white becomes the colour, grey a darker shade."""
    out = pixels.astype(float)
    x0, y0, x1, y1 = logo_box(pixels)
    keep = np.zeros(pixels.shape[:2], dtype=bool)
    keep[y0:y1, x0:x1] = True
    shade = out[..., :3].mean(axis=-1, keepdims=True) / 255.0
    tinted = shade * np.array(rgb(colour), dtype=float)
    out[..., :3] = np.where(keep[..., None], out[..., :3], tinted)
    return out.round().clip(0, 255).astype(np.uint8)


def png_bytes(pixels):
    buffer = io.BytesIO()
    Image.fromarray(pixels).save(buffer, format='PNG', optimize=True)
    return buffer.getvalue()


def material(name):
    return (f'# robot colour {name}: the cover material with the painted texture (make_robot_colour_covers.py)\n'
            f'newmtl Material\nNs 323.999994\nKa 1.000000 1.000000 1.000000\nKd 0.800000 0.800000 0.800000\n'
            f'Ks 0.500000 0.500000 0.500000\nKe 0.000000 0.000000 0.000000\nNi 1.450000\nd 1.000000\nillum 2\n'
            f'map_Kd cover_albedo_v2_{name}.png\n')


def mesh(side, name):
    """The side cover mesh with its material library line pointing at the colour's material."""
    lines = (MESHES / f'{side}.obj').read_text().splitlines()
    swapped = [f'mtllib cover_{name}.mtl' if line.startswith('mtllib ') else line for line in lines]
    if swapped == lines:
        raise ValueError(f'{side}.obj has no mtllib line')
    return '\n'.join(swapped) + '\n'


def wanted():
    """Published file name -> bytes, for every colour of the table."""
    pixels = np.asarray(Image.open(MESHES / TEXTURE).convert('RGBA'))
    files = {}
    for colour in colours():
        name = colour['name']
        files[f'cover_albedo_v2_{name}.png'] = png_bytes(painted(pixels, colour))
        files[f'cover_{name}.mtl'] = material(name).encode()
        for side in SIDES:
            files[f'{side}_{name}.obj'] = mesh(side, name).encode()
    return files


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true', help='only report whether the files are current')
    args = parser.parse_args(argv)
    files = wanted()
    stale = sorted(name for name, data in files.items()
                   if not (OUT / name).is_file() or (OUT / name).read_bytes() != data)
    extra = sorted(p.name for p in OUT.glob('*') if p.name not in files) if OUT.is_dir() else []
    if args.check:
        for name in stale:
            print(f'out of date: {name}')
        for name in extra:
            print(f'not from the colour table: {name}')
        return 1 if stale or extra else 0
    OUT.mkdir(parents=True, exist_ok=True)
    for name in stale:
        (OUT / name).write_bytes(files[name])
        print(f'wrote {OUT / name}')
    for name in extra:
        (OUT / name).unlink()
        print(f'removed {OUT / name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
