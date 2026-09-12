"""Leadshine DH116S dexterous hand — geometry, coupling data and retargeting.

The CANFD driver and transport moved to ``d1-firmwared``; what remains is the
hand's SHAPE (:mod:`description`, ``descriptions/``), its measured linkage
:mod:`coupling` (``data/*.csv``), its physical :mod:`toolconfig`, and the
glove→hand :mod:`retarget` map. Axis vocabulary lives in :mod:`axes`, lifted
out of the retired driver so the retarget map keeps its names.
"""

from .axes import ACTIVE_ROM_DEG, AXIS_NAMES, NUM_AXES, POS_MAX  # noqa: F401


def description_path(name: str = "DH116S-R000-A1.xml"):
    """Absolute :class:`pathlib.Path` to a bundled description file.

    ``descriptions/`` here is the single source of truth for the DH116S
    vendor MJCF + decimated meshes (see ``descriptions/README.md`` for
    provenance). Resolved via :mod:`importlib.resources` so it works from
    both a checkout and an installed package — consumers must not hardcode
    repo-relative paths.
    """
    from importlib.resources import files  # noqa: PLC0415

    path = files(__name__) / "descriptions" / name
    if not path.is_file():
        raise FileNotFoundError(f"no bundled DH116S description named {name!r}")
    return path
