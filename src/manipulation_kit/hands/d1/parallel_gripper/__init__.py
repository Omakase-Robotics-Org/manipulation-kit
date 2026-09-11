"""D1 stock parallel gripper — geometry and physical registration data.

- :mod:`toolconfig` — physical registration data for the arm controller
  (TCP offset, mass, COM, inertia), resolved by
  :func:`manipulation_kit.hands.get_tool_config`.
- :mod:`description` — the vendor CAD URDF + meshes, resolved by
  :func:`description_path` / :func:`description.load_urdf`. Single source of
  truth for the gripper's SHAPE; :mod:`manipulation_kit.description` composes it
  onto the D1 wrist rather than keeping its own copy of the geometry.
- ``tools/`` — the vendoring and calibration scripts that PRODUCED
  ``descriptions/``: ``tools/vendoring/vendor_gripper_description.py`` and
  ``tools/vendoring/vendor_camera_plate.py`` at the repository root (outside
  the wheel — they need CAD that is not in this repository), plus
  :mod:`tools.calibrate_wrist_camera_mount` here.

The DRIVER is not here. On the robot this gripper is reachable only through the
D1 arm controller's CAN channel passthrough, and that passthrough — with the
force-limited grasp, the supervised preload hold and the thermal
self-protection grown against d1-2 — is now owned by ``d1-firmwared``. What is
left is what the gripper IS, which is what planners, sims and the URDF
generator need and what no daemon should have to answer.
"""

def description_path(name: str = "gripper.urdf"):
    """Absolute :class:`pathlib.Path` to a bundled description file.

    ``descriptions/`` here is the single source of truth for the gripper's
    vendor URDF + decimated meshes (see ``descriptions/README.md`` for
    provenance and the list of local changes). Resolved via
    :mod:`importlib.resources` so it works from both a checkout and an
    installed package — consumers must not hardcode repo-relative paths.
    """
    from importlib.resources import files  # noqa: PLC0415

    path = files(__name__) / "descriptions" / name
    if not path.is_file():
        raise FileNotFoundError(
            f"no bundled parallel-gripper description named {name!r}")
    return path
