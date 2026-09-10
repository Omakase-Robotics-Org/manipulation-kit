"""DH116S MuJoCo description loader — right (vendor) and mirrored LEFT hand.

Only the RIGHT-hand model (DH116S-R000-A1) ships from the vendor. A true
left hand is its sagittal mirror image, NOT the right model reused/rotated
(that reads as a right hand on the left arm — Shu 2026-07-16): reflection
M = diag(-1, 1, 1) across the hand's local x = 0 plane produces
opposite-chirality parts while preserving the command→flex convention
(a reflection maps closing→closing). :func:`load_mjspec` therefore returns
the bundled MJCF as a fresh ``mujoco.MjSpec`` and, for ``side="left"``,
applies the mirror in place.

Mesh chirality is achieved with a NEGATIVE x mesh scale — MuJoCo ≥ 3.10
handles the flipped triangle winding of negative scales correctly; older
versions render/collide inside-out, so pin mujoco>=3.10 when using the left
hand.

``mujoco`` is NOT a dependency of this package (same pattern as the driver's
lazy ``can`` import): pass the imported module in as the first argument, or
pass ``None`` and it is imported lazily here.

(Migrated from d1-vr-teleop ``backends._mirror_hand`` 2026-07 — the mirror
math is verbatim. Robot-specific composition — wrist mount frames, adapter
offsets, YUBI replacement — stays in the robot repos; this module only
answers "give me a DH116S spec of the requested chirality".)
"""
from __future__ import annotations

from pathlib import Path

from . import description_path


def load_mjspec(mujoco=None, side: str = "right"):
    """Bundled DH116S model as a fresh ``mujoco.MjSpec``, ready to attach.

    ``mujoco``: the imported :mod:`mujoco` module, or ``None`` to import it
    lazily here (kept out of package deps — install ``mujoco>=3.10``).
    ``side``: ``"right"`` = the vendor R000 model as shipped; ``"left"`` =
    sagittal mirror (see module docstring).

    Mesh paths are rewritten to absolute (a spec re-rooted or attached into
    another spec loses its ``meshdir`` anchor), so the returned spec compiles
    and attaches from anywhere.
    """
    if mujoco is None:
        try:
            import mujoco  # noqa: PLC0415 — lazy: not a package dependency
        except ImportError as exc:
            raise ImportError(
                "load_mjspec needs the mujoco package (not a dependency of "
                "dx-manipulator): pip install 'mujoco>=3.10'") from exc
    if side not in ("right", "left"):
        raise ValueError(f"side must be 'right' or 'left', got {side!r}")
    xml = Path(str(description_path()))
    spec = mujoco.MjSpec.from_file(str(xml))
    meshdir = xml.parent / "meshes"
    for msh in spec.meshes:  # absolute paths — survive re-rooting/attaching
        msh.file = str(meshdir / Path(msh.file).name)
    if side == "left":
        _mirror_hand(mujoco, spec)
    return spec


def _mirror_hand(mujoco, hand):
    """Reflect a hand spec across its local sagittal plane (x = 0) IN PLACE
    → the opposite-chirality hand. Reflection M = diag(-1, 1, 1). The parent
    rotations telescope, so it applies per element in LOCAL frames:
    R_i'_local = M R_i_local M (quat (w,x,y,z)→(w,x,-y,-z)), t_local.x→-x,
    joint axis (ax,ay,az)→(ax,-ay,-az), mesh scale.x→-1. Body orientations
    are read from a throwaway COMPILE of the reference model (the spec keeps
    ``euler`` in ``.alt``, quat identity until compiled) then written back
    as quats. Command→flex convention preserved (a reflection maps
    closing→closing)."""
    ref = mujoco.MjSpec.from_file(str(description_path()))
    rm = ref.compile()
    qmap = {rm.body(i).name: rm.body(i).quat.copy() for i in range(rm.nbody)}
    for b in hand.bodies:
        if b.name in ("world", ""):
            continue
        b.pos[0] = -b.pos[0]
        q = qmap.get(b.name)
        if q is not None:
            w, x, y, z = (float(v) for v in q)
            b.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
            b.quat[:] = [w, x, -y, -z]
    for j in hand.joints:
        j.pos[0] = -j.pos[0]
        ax = j.axis
        j.axis[:] = [ax[0], -ax[1], -ax[2]]
    for m in hand.meshes:
        m.scale[0] = -m.scale[0]
    return hand
