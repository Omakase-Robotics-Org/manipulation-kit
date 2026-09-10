"""D1 stock parallel gripper URDF loader.

The vendor CAD description (``descriptions/gripper.urdf`` plus its meshes) is
the single source of truth for this gripper's geometry — robot repos compose
it onto a wrist, they do not keep their own copy of the shape.

Unlike the DH116S there is no chirality here: a two-finger parallel gripper is
its own mirror image about the jaw-travel axis, so the same description mounts
on either arm and only the MOUNT transform differs (which is robot⊕hand
composition and belongs to the robot repo, not here).

``xml.etree`` is in the standard library, so unlike ``dh116s.description``
this loader needs no optional third-party package.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from . import description_path

#: Jaw travel of ONE finger, metres. ``tcp_r_joint`` spans ``[0, STROKE]``
#: and ``tcp_l_joint`` mimics it with multiplier -1 over ``[-STROKE, 0]``.
JAW_STROKE_M = 0.035

#: Gap between the jaw faces at ``q = 0``. Both jaws close inward as ``|q|``
#: grows, so ``q = 0`` is OPEN and ``|q| = JAW_STROKE_M`` is CLOSED — the
#: opposite polarity to the CAN 2.0 position command, where 0.0 is closed.
JAW_OPEN_GAP_M = 2 * JAW_STROKE_M

#: The gap the DRIVEN gripper actually reaches. The mechanism travels
#: ~1.55 rad (= the CAD's 70 mm above), but the driver's commanded ceiling
#: is OPEN_RAD = 1.16 rad, and d1-3 measures 51.96 mm there (2026-08-24) —
#: the linear map is ~44.8 mm/rad, so both numbers are right and the gap
#: between them is UNUSED TRAVEL, not a modelling error. Planning and sim
#: must use THIS opening, not JAW_OPEN_GAP_M: a 48 mm tape leaves 11 mm of
#: clearance per side on paper and 2 mm in reality. Raising OPEN_RAD would
#: recover margin, but 1.16 was field-tuned on d1-2 — a hardware decision,
#: not one for this file.
DRIVEN_OPEN_GAP_M = 0.05196
#: The jaw joint value at the driven-open stop: q = (JAW_OPEN_GAP_M -
#: DRIVEN_OPEN_GAP_M) / 2 per finger. Sim "fully open" is this, not 0.
DRIVEN_OPEN_Q = (JAW_OPEN_GAP_M - DRIVEN_OPEN_GAP_M) / 2.0

#: Distance from the mounting flange (``base_link`` origin) to the jaw tips,
#: along the gripper's +Z approach axis. The REGISTERED TCP is 136 mm (see
#: :mod:`~manipulation_kit.hands.d1.parallel_gripper.toolconfig`), 7.5 mm short of the
#: tips, i.e. on the pad face rather than the extreme corner.
JAW_TIP_Z_M = 0.14350

#: The arm-end connection plate (V2.0, 2026-08-21) that carries the
#: wide-angle UVC wrist camera. It fills the first 8 mm of the 16.5 mm
#: flange gap the vendor gripper CAD leaves empty; its camera arm extends
#: along ``base_link`` +Y. See ``tools/vendor_camera_plate.py`` for the
#: derivation of every number and ``descriptions/README.md`` for provenance.
CAMERA_PLATE_THICKNESS_M = 0.008

#: Camera MOUNT-face frame in ``base_link``: origin at the centre of the
#: plate's 4-hole camera pattern, local +Z the face normal (15 deg from the
#: flange +Z toward -Y, i.e. toward the fingers), local +Y up the arm. The
#: ROS optical frame is this rotated pi about local Z (fingers at the image
#: bottom, as the real wrist streams show).
CAMERA_MOUNT_XYZ_M = (0.0, 0.079236, 0.014543)
CAMERA_TILT_RAD = 0.2617993877991494  # 15 deg

#: PER-ARM CLOCKING (robot composition, not encoded in the URDF): on the D1
#: the camera sits on top of the wrist on BOTH arms, so the physical LEFT
#: arm (SDK "_R" tree) mounts the description with yaw = pi about the flange
#: Z and the physical RIGHT ("_L") with yaw = 0. Measured 2026-08-21 from
#: the d1 teleop dataset (FK at a grasp frame vs the head-camera view).
CAMERA_ARM_YAW_RAD = {"left": 3.141592653589793, "right": 0.0}


def load_urdf(absolute_meshes: bool = True, camera: bool = False) -> ET.ElementTree:
    """The bundled gripper URDF as an :class:`xml.etree.ElementTree`.

    ``absolute_meshes`` (default) rewrites every ``<mesh filename=...>`` to an
    absolute path, so the tree can be serialised anywhere — into a composed
    robot URDF in another repository, a temp dir, a sim's asset cache — and
    still resolve. Pass ``False`` to get the file exactly as committed, whose
    mesh paths are relative to ``descriptions/``.

    ``camera=True`` returns ``gripper_with_camera.urdf`` instead: the same
    gripper plus the arm-end camera plate, the ``wrist_camera`` mount frame
    and the ``wrist_camera_optical`` ROS optical frame.
    """
    path = Path(str(description_path(_urdf_name(camera))))
    tree = ET.parse(path)
    if absolute_meshes:
        for mesh in tree.getroot().iter("mesh"):
            filename = mesh.get("filename")
            if filename is None:
                continue
            mesh.set("filename", str((path.parent / filename).resolve()))
    return tree


def mesh_paths(camera: bool = False) -> list[Path]:
    """Every mesh the URDF references, as absolute paths, in document order."""
    path = Path(str(description_path(_urdf_name(camera))))
    return [(path.parent / m.get("filename", "")).resolve()
            for m in ET.parse(path).getroot().iter("mesh")]


def _urdf_name(camera: bool) -> str:
    return "gripper_with_camera.urdf" if camera else "gripper.urdf"
