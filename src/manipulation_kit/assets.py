"""Where the CAD geometry is, and what to do when it is not here.

**This package deliberately carries no Omakase or vendor CAD.** The URDFs are
here, complete and unedited; the STL geometry they reference is not. It lives
in the private companion repository ``manipulation-kit-assets`` because its
redistribution rights are unresolved (see ``LICENSE-STATUS.md``), and this
repository is meant to be publishable without waiting for that answer.

Third-party geometry that already carries a licence *stays*: the YUBI hand
(Apache-2.0, Toyota) and the DH116S hand (Apache-2.0, upstream
``leadtron_hand_descriptions``) ship in the package like any other file. Only
:data:`EXTERNAL_ASSET_DIRS` is external.

Two ways to complete a checkout, and they are deliberately different:

``MKIT_ASSETS_DIR``
    Points at an assets checkout and resolves *without copying anything*. Use
    it in CI and in tests: nothing is written into the package, so a run
    cannot leave unlicensed geometry behind in a work tree.

``mkit-urdf fetch-assets --from <path-or-git-url>``
    Copies the files into the package, permanently, so that RViz / MuJoCo /
    Isaac loading a URDF straight out of ``description/`` resolve the
    ``<mesh>`` references the way they always have. The destinations are in
    ``.gitignore``; they cannot be committed back by accident.

Everything that reads geometry goes through :func:`resolve`, so "the mesh is
absent" is one answer in one place rather than a ``FileNotFoundError`` from
five different callers.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, Optional

#: Anchor: the installed ``manipulation_kit/`` package directory.
PACKAGE_ROOT = Path(__file__).resolve().parent

#: The environment variable that points at an assets checkout. It names the
#: repository ROOT — the directory that contains ``manipulation_kit/`` — so the
#: assets repo mirrors this package's own layout and a path is a path.
ASSETS_ENV = "MKIT_ASSETS_DIR"

#: Directories of CAD geometry this package does NOT carry, relative to
#: :data:`PACKAGE_ROOT`. Every one of them is Omakase- or vendor-produced CAD
#: with no redistribution grant on file; every one of them is reproduced, at
#: the same relative path, in ``manipulation-kit-assets``.
#:
#: Keep this list and the assets repository in step: it is what
#: ``fetch-assets`` copies, what the exporter declares as ``absent_external``,
#: and what the description tests skip themselves over.
EXTERNAL_ASSET_DIRS = (
    "description/d1/meshes/body",
    "description/d1/meshes/gripper",
    "description/d1_arm/left/meshes",
    "description/d1_arm/right/meshes",
    "hands/d1/parallel_gripper/descriptions/meshes",
)

#: The decorative body visuals (~50 MB of ``.obj``). Absent for a different
#: reason — size, and the guard never opens one — and fetched from a d1-sdk
#: checkout by ``mkit-urdf fetch-visuals``. Listed here so ``fetch-assets``
#: will also complete it if the source happens to carry it.
OPTIONAL_VISUAL_DIR = "description/d1/meshes/body_hifi"

#: File suffixes ``fetch-assets`` moves. Geometry only: never a URDF, never a
#: README, so a fetch can never overwrite something this repository owns.
ASSET_SUFFIXES = (".STL", ".stl", ".obj", ".dae", ".ply", ".glb")


def assets_root() -> Optional[Path]:
    """The assets checkout named by ``$MKIT_ASSETS_DIR``, or ``None``.

    A value that does not exist is an error worth raising rather than a silent
    fallback to "no assets": setting the variable is a statement of intent, and
    a typo in it must not read as a clean mesh-free run.
    """
    raw = os.environ.get(ASSETS_ENV)
    if not raw:
        return None
    root = Path(raw).expanduser()
    if not root.is_dir():
        raise FileNotFoundError(
            f"${ASSETS_ENV} is set to {raw!r}, which is not a directory. "
            "Point it at a manipulation-kit-assets checkout, or unset it.")
    inner = root / "manipulation_kit"
    return inner if inner.is_dir() else root


def resolve(pkg_rel: str) -> Optional[Path]:
    """Locate one asset by its path relative to ``manipulation_kit/``.

    Prefers a file already in the package (a fetched checkout, or a
    class-licensed mesh that was never external) and falls back to the assets
    checkout. Returns ``None`` when neither has it — which is a normal state,
    not a failure.
    """
    local = PACKAGE_ROOT.joinpath(pkg_rel)
    if local.is_file():
        return local
    root = assets_root()
    if root is not None:
        remote = root.joinpath(pkg_rel)
        if remote.is_file():
            return remote
    return None


def is_external(path) -> bool:
    """True if ``path`` (absolute, or relative to the package) is CAD this
    repository does not carry — whether or not a copy happens to be present."""
    try:
        rel = Path(path).resolve().relative_to(PACKAGE_ROOT)
    except ValueError:
        return False
    return any(str(rel).startswith(d + "/") for d in EXTERNAL_ASSET_DIRS)


def external_dirs() -> Iterable[str]:
    """:data:`EXTERNAL_ASSET_DIRS` plus the optional visual layer."""
    return (*EXTERNAL_ASSET_DIRS, OPTIONAL_VISUAL_DIR)


def have_external_assets() -> bool:
    """True when every external directory resolves to real geometry.

    All-or-nothing on purpose: a half-fetched tree renders as half a robot,
    and a test that ran against half of it would be reporting on nothing.
    """
    for rel in EXTERNAL_ASSET_DIRS:
        local = PACKAGE_ROOT / rel
        if any(local.glob("*.STL")):
            continue
        root = assets_root()
        if root is not None and any((root / rel).glob("*.STL")):
            continue
        return False
    return True


#: The message every skipped description test prints. One string, so a reader
#: who has never seen this repository learns the whole story from any of them.
NO_ASSETS_REASON = (
    "the CAD geometry is not in this repository (see LICENSE-STATUS.md). "
    f"Set ${ASSETS_ENV} to a manipulation-kit-assets checkout, or run "
    "`mkit-urdf fetch-assets --from <path-or-git-url>`, to run this test.")

__all__ = [
    "ASSETS_ENV", "ASSET_SUFFIXES", "EXTERNAL_ASSET_DIRS",
    "NO_ASSETS_REASON", "OPTIONAL_VISUAL_DIR", "PACKAGE_ROOT",
    "assets_root", "external_dirs", "have_external_assets", "is_external",
    "resolve",
]
