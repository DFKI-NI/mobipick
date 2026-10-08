"""Loads the compiled solver of the carrier link chain.

Looked for in this order: the file named by $CABLE_DISC_CHAIN_LIB, `libdisc_chain.so` on the loader path (a catkin
build puts it in the workspace's devel/lib), and a build of the source with g++ kept in the user's cache directory
(offline scripts and tests on a machine without a catkin build; rebuilt when the source changes).
"""
import ctypes
import hashlib
import os
import subprocess
from typing import Optional

SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "native", "disc_chain.cpp")
# layout of the arguments; native/disc_chain.cpp returns the same from disc_chain_abi_version(). 2: 8 numbers
# per shape (core ends, sweep radius, core radius) instead of 7. 3: rope_tighten, and a shape cull per chain. 4: 20 numbers per
# shape (rounded boxes)
ABI_VERSION = 4
_lib = None
_error: Optional[str] = None


class Params(ctypes.Structure):
    _fields_ = [("substeps", ctypes.c_int), ("passes", ctypes.c_int), ("gravity", ctypes.c_double),
                ("keep", ctypes.c_double), ("turn", ctypes.c_double), ("self_distance", ctypes.c_double),
                ("clamp_taper", ctypes.c_double), ("friction", ctypes.c_double)]


def _cached_build() -> str:
    source = os.path.abspath(SOURCE)
    with open(source, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()[:16]
    cache = os.path.join(os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache"),
                         "mobipick_cable_entanglement")
    out = os.path.join(cache, f"libdisc_chain_{digest}.so")
    if not os.path.exists(out):
        os.makedirs(cache, exist_ok=True)
        tmp = f"{out}.{os.getpid()}.tmp"
        subprocess.run(["g++", "-O2", "-shared", "-fPIC", "-std=c++11", source, "-o", tmp], check=True,
                       capture_output=True)
        os.replace(tmp, out)   # other processes building at the same time end up with the same file
    return out


def load():
    """the library (ctypes.CDLL) with disc_chain_step typed; raises RuntimeError when it cannot be found or built"""
    global _lib, _error
    if _lib is not None:
        return _lib
    if _error is not None:
        raise RuntimeError(_error)
    tried = []
    for candidate in (lambda: os.environ["CABLE_DISC_CHAIN_LIB"], lambda: "libdisc_chain.so", _cached_build):
        try:
            lib = ctypes.CDLL(candidate())
            version = getattr(lib, "disc_chain_abi_version", None)
            if version is None or version() != ABI_VERSION:   # a library built from older source
                raise RuntimeError(f"ABI {None if version is None else version()} instead of {ABI_VERSION}: rebuild it")
            break
        except Exception as exc:  # noqa: BLE001 - try the next place
            tried.append(f"{type(exc).__name__}: {exc}")
    else:
        _error = ("the carrier link chain solver (native/disc_chain.cpp) is not built: build the package with "
                  "catkin, or "
                  "install g++; tried: " + "; ".join(tried))
        raise RuntimeError(_error)
    d, i = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
    lib.disc_chain_step.argtypes = [d, d, d, ctypes.c_int, i, i, d, i, ctypes.c_int, d, d, d, d, d, ctypes.c_int, d,
                                    ctypes.c_double, ctypes.POINTER(Params), d]
    lib.disc_chain_step.restype = ctypes.c_int
    lib.rope_tighten.argtypes = [d, ctypes.c_int, d, ctypes.c_int, ctypes.c_int]
    lib.rope_tighten.restype = None
    lib.rope_collide.argtypes = [d, i, ctypes.c_int, d, ctypes.c_int]
    lib.rope_collide.restype = None
    _lib = lib
    return lib


def available() -> bool:
    try:
        load()
        return True
    except RuntimeError:
        return False
