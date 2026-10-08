"""Geometry helpers of the cable model (see doc/cable_model.md for the math)."""
from typing import Optional, List, Sequence, Tuple

import numpy as np


def wrap_angle(a):
    """Angle(s) to (-pi, pi]."""
    return np.pi - np.mod(np.pi - np.asarray(a), 2.0 * np.pi)


def cylindrical(p: np.ndarray, origin: np.ndarray, axis: np.ndarray, ref: np.ndarray) -> Tuple[float, float, float]:
    """(rho, theta, h) of point p about the axis through `origin` along unit `axis`; theta = 0 along `ref`."""
    d = p - origin
    h = float(d @ axis)
    radial = d - h * axis
    e1 = ref - (ref @ axis) * axis
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    return float(np.linalg.norm(radial)), float(np.arctan2(radial @ e2, radial @ e1)), h


def cylinder_geodesic(rho_a: float, rho_b: float, dh: float, delta: float, r: float) -> float:
    """Shortest length of a string from A to B that goes around an infinite cylinder of radius r.

    A and B are at radial distances rho_a, rho_b from the axis, axial offset dh apart and at an *unwrapped*
    angular separation delta (it may exceed pi: the string cannot pass through the cylinder, so a string that
    was wound around it stays wound). Without contact the string is the straight line; with contact it is
    tangent segment + helix + tangent segment, which develops into a straight line of horizontal length
    sqrt(rho_a^2 - r^2) + r * wrap + sqrt(rho_b^2 - r^2) on the unrolled cylinder.
    """
    rho_a = max(rho_a, r * 1.0001)
    rho_b = max(rho_b, r * 1.0001)
    alpha_a = np.arccos(r / rho_a)
    alpha_b = np.arccos(r / rho_b)
    wrap = abs(delta) - alpha_a - alpha_b
    if wrap <= 0.0:
        planar_sq = rho_a * rho_a + rho_b * rho_b - 2.0 * rho_a * rho_b * np.cos(delta)
        return float(np.sqrt(max(planar_sq, 0.0) + dh * dh))
    planar = np.sqrt(rho_a * rho_a - r * r) + np.sqrt(rho_b * rho_b - r * r) + r * wrap
    return float(np.sqrt(planar * planar + dh * dh))


def point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance of points p (N,3) to segment a-b."""
    ab = b - a
    denom = float(ab @ ab)
    t = np.zeros(len(p)) if denom < 1e-12 else np.clip((p - a) @ ab / denom, 0.0, 1.0)
    closest = a + t[:, None] * ab
    return np.linalg.norm(p - closest, axis=1)


def closest_on_segments(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Closest points (N,M,3) of points p (N,3) on M segments a (M,3) - b (M,3)."""
    ab = b - a                                                     # (M,3)
    denom = np.maximum(np.einsum("mk,mk->m", ab, ab), 1e-12)       # (M)
    t = np.clip(np.einsum("nmk,mk->nm", p[:, None, :] - a[None], ab) / denom, 0.0, 1.0)
    return a[None] + t[..., None] * ab[None]


def segment_segment_distance(p0: np.ndarray, p1: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> float:
    """Minimum distance between segments p0-p1 and q0-q1 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= eps:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm((p0 + d1 * s) - (q0 + d2 * t)))


def point_core_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray, core: float) -> np.ndarray:
    """distance of points p (N,3) from a flat cylinder's core: the disc of radius `core` along a-b (0 inside)"""
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    w = p - (a + t[:, None] * ab)
    u = ab / max(float(np.linalg.norm(ab)), 1e-12)
    along = w @ u
    across = np.linalg.norm(w - along[:, None] * u, axis=1)
    return np.hypot(along, np.maximum(across - core, 0.0))


def point_box_distance(p: np.ndarray, centre: np.ndarray, box: np.ndarray) -> np.ndarray:
    """distance of points p (N,3) from a box core (; box: 3 half sizes + rotation row-major, its columns the box
    axes; 0 inside)"""
    local = (p - centre) @ box[3:].reshape(3, 3)
    return np.linalg.norm(np.maximum(np.abs(local) - box[:3], 0.0), axis=1)


def segment_clears(p: np.ndarray, q: np.ndarray, caps_a: np.ndarray, caps_b: np.ndarray, caps_r: np.ndarray,
                   caps_core: Optional[np.ndarray] = None, caps_box: Optional[np.ndarray] = None,
                   tol: float = 0.0) -> bool:
    """True when the straight segment p-q keeps at least radius - tol away from every shape: from the axis of a
    capsule, from the core of a flat cylinder (caps_core > 0; checked on points every 4 mm along p-q where the
    distance from the axis cannot rule it out)."""
    core = caps_core if caps_core is not None else np.zeros(len(caps_r))
    box = caps_box if caps_box is not None else np.zeros((len(caps_r), 12))
    for a, b, r, c, bx in zip(caps_a, caps_b, caps_r, core, box):
        d = segment_segment_distance(p, q, a, b)
        if bx[0] > 0.0:   # a rounded box: a is its centre; the core lies within its half diagonal of it
            if d - float(np.linalg.norm(bx[:3])) < r - tol:
                n = max(2, int(np.ceil(np.linalg.norm(q - p) / 0.004)) + 1)
                pts = p + np.linspace(0.0, 1.0, n)[:, None] * (q - p)
                if point_box_distance(pts, a, bx).min() < r - tol:
                    return False
        elif c <= 0.0:
            if d < r - tol:
                return False
        elif d - c < r - tol:   # the core lies within c of its axis
            n = max(2, int(np.ceil(np.linalg.norm(q - p) / 0.004)) + 1)
            pts = p + np.linspace(0.0, 1.0, n)[:, None] * (q - p)
            if point_core_distance(pts, a, b, c).min() < r - tol:
                return False
    return True


def string_pull(points: Sequence[np.ndarray], caps_a: np.ndarray, caps_b: np.ndarray, caps_r: np.ndarray,
                caps_core: Optional[np.ndarray] = None, caps_box: Optional[np.ndarray] = None,
                tol: float = 0.0) -> List[np.ndarray]:
    """Taut version of a polyline: drop every interior point whose neighbours see each other past the shapes.

    What stays are the points where the string is really held by an obstacle (wrapped or hooked), so the length
    of the result is (a lower bound of) the length the cable needs in its current topology.
    """
    pts = list(points)
    changed = True
    while changed and len(pts) > 2:
        changed = False
        i = 1
        while i < len(pts) - 1:
            if segment_clears(pts[i - 1], pts[i + 1], caps_a, caps_b, caps_r, caps_core, caps_box, tol):
                del pts[i]
                changed = True
            else:
                i += 1
    return pts


def polyline_length(points: Sequence[np.ndarray]) -> float:
    pts = np.asarray(points)
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()) if len(pts) > 1 else 0.0
