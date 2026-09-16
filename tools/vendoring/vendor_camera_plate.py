#!/usr/bin/env python3
"""Vendor the wrist-camera connection plate and generate the composed URDF.

The D1 gripper bolts to the arm tool flange through a two-plate stack that
the vendor gripper CAD leaves as an EMPTY 16.5 mm gap (the gripper body mesh
starts at base_link z = 16.5 mm; see ``descriptions/README.md``). The V2.0
arm-end plate (夹爪连接板 手臂端 V2.0, received 2026-08-21) fills the first
8 mm of that gap and carries an arm that holds the wide-angle UVC wrist
camera above the wrist, its mount face pitched 15 deg toward the fingers.

This script:

1. Converts the source STL from millimetres to metres and writes it to
   ``descriptions/meshes/camera_plate.STL`` (7 470 triangles, 365 KB — small
   enough to keep verbatim, no decimation).
2. Re-derives the camera-face geometry from the mesh and ASSERTS it matches
   the constants below (the same numbers exported by ``description.py``), so
   a future plate revision cannot silently drift away from the URDF.
3. Computes the plate inertia EXACTLY from the mesh (tetrahedron
   decomposition, uniform density) at the aluminium density below.
4. Generates ``descriptions/gripper_with_camera.urdf``: the committed
   ``gripper.urdf`` plus the plate link, the camera mount frame and the ROS
   optical frame. Like ``gripper.urdf`` itself, the output is generated —
   re-run this script instead of editing it.

Frames (all in the gripper ``base_link`` = arm tool flange frame):

* ``camera_plate`` — identity: the plate's own CAD origin is exactly the
  flange centre, disc face on the flange plane, +Z toward the gripper.
  The camera arm extends along +Y.
* ``wrist_camera`` — the camera MOUNT face: centre of the plate's 4-hole
  camera pattern, ``rpy = (+15 deg, 0, 0)`` so local +Z is the face normal
  (15 deg from the flange +Z toward -Y, i.e. toward the fingers) and local
  +Y points up the arm.
* ``wrist_camera_optical`` — ROS optical convention (+Z forward, +X image
  right, +Y image down). Image "down" points toward the fingers, which is
  what the real wrist streams show (jaws at the bottom edge of the frame),
  so optical = mount rotated pi about Z.

PER-ARM CLOCKING IS NOT ENCODED HERE. The plate makes the assembly chiral,
and which way the camera arm points is robot⊕hand composition (it belongs to
the robot repo, same as the gripper mount itself). Measured from the d1
teleop dataset (FK at a grasp frame vs the head-camera view, 2026-08-21):
the camera sits on TOP of the wrist on both arms — flange -Y on the physical
LEFT arm (SDK "_R" tree), flange +Y on the physical RIGHT ("_L"), i.e. the
left arm mounts this description with yaw = pi, the right with yaw = 0.

Source drop: ``夹爪连接板（手臂端）V2.0.stl`` (SolidWorks binary STL, mm)
SHA256 3d2ba4b71da48057b21ed86cf8cee6574ce8aa4a8d979ee7aaaf36ac00caea63

Needs only numpy (dev-only; not a dependency of this package).
"""
from __future__ import annotations

import argparse
import hashlib
import math
import pathlib
import struct
import sys

import numpy as np

from _kit_paths import parallel_gripper

HERE = pathlib.Path(__file__).resolve().parent
DEST = parallel_gripper() / "descriptions"

#: SHA256 of the source drop this script was written against. A different
#: hash is not an error (plates get revised) but is loudly reported so the
#: constants below are re-checked against the new geometry.
SOURCE_SHA256 = "3d2ba4b71da48057b21ed86cf8cee6574ce8aa4a8d979ee7aaaf36ac00caea63"

# --------------------------------------------------------------------------
# Geometry derived from the V2.0 mesh (mm, plate frame). The camera face is
# the large planar face whose normal is +Z pitched 15 deg toward -Y; the
# mount point is the centre of its 4-hole screw pattern (holes at roughly
# (+-14.1, 65.9) and (+-14.6, 94.7) in in-plane coordinates). verify()
# re-derives both from the mesh on every run.
# --------------------------------------------------------------------------
CAM_TILT_RAD = math.radians(15.0)
CAM_MOUNT_XYZ_M = (0.0, 0.079236, 0.014543)
PLATE_THICKNESS_M = 0.002          # MEASURED (d1-3, Shu, callipers, 2026-09-16)
CAD_PLATE_THICKNESS_M = 0.008      # what the V2.0 mesh models; the arm rises to z = 18.6 mm

#: The gripper-end plate (夹爪端) that fills the REMAINING 8.5 mm of the
#: vendor CAD's 16.5 mm flange gap is separate hardware with no CAD drop yet;
#: it is NOT modelled. Recorded so the visible gap is a known gap.
FLANGE_GAP_M = 0.0165

#: Machined aluminium plate. Density assumption, not a weighed part — the
#: mesh volume is 28.95 cm^3, so 2700 kg/m^3 puts it at 78 g. Weigh it when
#: it lands (the gripper CAD taught us to distrust unweighed masses, but a
#: solid machined part at book density is a far safer estimate than a
#: shell-only export).
ALUMINIUM_KG_M3 = 2700.0

#: Placeholder for the wide-angle UVC module (same unit as the d1-2 wrist
#: camera, which has no nameplate; board dimensions not yet measured). The
#: 30 x 30 x 16 mm visual box and 30 g are stand-ins to render and to keep
#: the dynamics roughly honest — replace with measured values.
CAMERA_MASS_KG = 0.030
CAMERA_BOX_M = (0.030, 0.030, 0.016)
CAMERA_BOX_CENTER_M = (0.0, 0.0, 0.008)


def read_stl(path: pathlib.Path) -> np.ndarray:
    """Binary STL -> (n, 3, 3) float64 vertex array."""
    blob = path.read_bytes()
    n = struct.unpack("<I", blob[80:84])[0]
    if len(blob) != 84 + 50 * n:
        raise SystemExit(f"{path} is not a plain binary STL")
    tris = np.frombuffer(blob, np.uint8, count=n * 50, offset=84)
    return tris.reshape(n, 50)[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def write_stl(path: pathlib.Path, tris: np.ndarray, comment: str) -> None:
    n = len(tris)
    header = comment.encode()[:80].ljust(80, b" ")
    rec = np.zeros((n, 50), np.uint8)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lens = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lens, out=np.zeros_like(normals), where=lens > 0)
    rec[:, 0:12] = normals.astype("<f4").view(np.uint8).reshape(n, 12)
    rec[:, 12:48] = tris.astype("<f4").reshape(n, 9).view(np.uint8).reshape(n, 36)
    path.write_bytes(header + struct.pack("<I", n) + rec.tobytes())


def mesh_mass_properties(tris: np.ndarray, density: float):
    """(mass, com, inertia-about-com dict) of a closed mesh, tetra method."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    vol6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    volume = vol6.sum() / 6.0
    com = ((a + b + c) / 4.0 * vol6[:, None]).sum(0) / vol6.sum()
    # covariance of each tetra (origin apex), canonical-tetra transform
    canon = np.full((3, 3), 1.0 / 120.0) + np.eye(3) * (1.0 / 120.0)
    cov = np.zeros((3, 3))
    for i in range(len(tris)):
        T = np.stack([a[i], b[i], c[i]])
        cov += vol6[i] * (T.T @ canon @ T)
    cov -= volume * np.outer(com, com)
    mass = density * volume
    cov *= density
    inertia = np.trace(cov) * np.eye(3) - cov
    return mass, com, {
        "ixx": inertia[0, 0], "ixy": inertia[0, 1], "ixz": inertia[0, 2],
        "iyy": inertia[1, 1], "iyz": inertia[1, 2], "izz": inertia[2, 2],
    }


def verify(tris_m: np.ndarray) -> None:
    """Re-derive the camera face from the mesh; die if the constants drift."""
    normals = np.cross(tris_m[:, 1] - tris_m[:, 0], tris_m[:, 2] - tris_m[:, 0])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    n_ref = np.array([0.0, -math.sin(CAM_TILT_RAD), math.cos(CAM_TILT_RAD)])
    on_face = normals @ n_ref > 0.99997   # within ~0.5 deg
    if on_face.sum() < 100:
        raise SystemExit("camera face not found: no large planar face at 15 deg")
    face = tris_m[on_face].reshape(-1, 3)
    # outermost of the (two) parallel planes = the mount surface
    d = face @ n_ref
    d_mount = d.max()
    plane_pts = face[d > d_mount - 1e-4]
    u = plane_pts @ np.array([1.0, 0.0, 0.0])
    v = plane_pts @ np.cross(n_ref, [1.0, 0.0, 0.0])
    center = (np.array([u.min() + u.max(), v.min() + v.max()]) / 2.0)
    mount = np.array(CAM_MOUNT_XYZ_M)
    if abs(mount @ n_ref - d_mount) > 5e-4:
        raise SystemExit(f"CAM_MOUNT_XYZ_M is {abs(mount @ n_ref - d_mount) * 1e3:.2f} mm "
                         "off the mount plane")
    if abs(center[0] - mount[0]) > 1e-3 or abs(center[1] - (mount @ np.cross(n_ref, [1, 0, 0]))) > 4e-3:
        raise SystemExit("CAM_MOUNT_XYZ_M is not at the centre of the camera face")
    lo, hi = tris_m.reshape(-1, 3).min(0), tris_m.reshape(-1, 3).max(0)
    if abs(hi[2] - lo[2] - 0.0186) > 1e-3 or abs(hi[0] - lo[0] - 0.072) > 1e-3:
        raise SystemExit("plate envelope changed — re-derive every constant")
    # the disc must stay inside the gripper's 16.5 mm flange gap where the
    # gripper body exists (|x| < 28.5 mm, |y| < 29 mm — the body footprint)
    pts = tris_m.reshape(-1, 3)
    body = (np.abs(pts[:, 0]) < 0.0285) & (np.abs(pts[:, 1]) < 0.029)
    if pts[body][:, 2].max() > FLANGE_GAP_M + 1e-6:
        raise SystemExit("plate protrudes into the gripper body envelope")


def _fmt(v: float) -> str:
    return f"{v:.12G}"


def _xyz(t) -> str:
    return " ".join(_fmt(v) for v in t)


def _inertial(mass, com, inertia, indent="    ") -> str:
    return (f"{indent}<inertial>\n"
            f"{indent}  <origin xyz=\"{_xyz(com)}\" rpy=\"0 0 0\" />\n"
            f"{indent}  <mass value=\"{_fmt(mass)}\" />\n"
            f"{indent}  <inertia " + " ".join(f'{k}="{_fmt(v)}"' for k, v in inertia.items())
            + f" />\n{indent}</inertial>\n")


HEADER_NOTE = """
  <!-- CAMERA PLATE + WRIST CAMERA, appended by
       tools/vendoring/vendor_camera_plate.py; do not hand
       edit, re-run the script. Provenance: descriptions/README.md.

       The plate (arm-end connection plate V2.0) is the first 2 mm of the
       flange stack: Shu measured it at 2 mm with callipers on d1-3
       (2026-09-16), against the 8 mm of the CAD drop. Behind it sits a 7 mm
       spacer block, also measured, which is why the gripper body starts
       at 9 mm. The plate MESH here still models the CAD's 8 mm disc; only a
       new CAD drop replaces a mesh, so the residual is recorded, not hidden.
       The camera arm extends along +Y; PER-ARM CLOCKING (which way +Y points
       on the robot) is robot composition and is NOT encoded here: on the D1
       the camera sits on top of the wrist on both arms, so the physical LEFT
       arm mounts this file with yaw = pi about the flange Z, the physical
       RIGHT with yaw = 0 (measured from the d1 teleop dataset, 2026-08-21).

       wrist_camera is the mount FACE (+Z = face normal, 15 deg toward the
       fingers; +Y up the arm). wrist_camera_optical is the ROS optical frame
       (+Z forward, +X image right, +Y image down): the real wrist streams
       show the jaws at the BOTTOM edge of the image, which fixes the roll to
       optical = mount rotated pi about Z. The camera box and its 30 g are
       PLACEHOLDERS until the module (same unit as the d1-2 wrist camera, no
       nameplate) is measured. -->
"""


def compose_urdf(plate_mass, plate_com, plate_inertia) -> str:
    base = (DEST / "gripper.urdf").read_text()
    tail = base.rindex("</robot>")
    cam_rpy = (CAM_TILT_RAD, 0.0, 0.0)
    cam_inertia = {
        "ixx": CAMERA_MASS_KG * (CAMERA_BOX_M[1] ** 2 + CAMERA_BOX_M[2] ** 2) / 12,
        "ixy": 0.0, "ixz": 0.0,
        "iyy": CAMERA_MASS_KG * (CAMERA_BOX_M[0] ** 2 + CAMERA_BOX_M[2] ** 2) / 12,
        "iyz": 0.0,
        "izz": CAMERA_MASS_KG * (CAMERA_BOX_M[0] ** 2 + CAMERA_BOX_M[1] ** 2) / 12,
    }
    parts = [base[:tail].replace('name="gripper"', 'name="gripper_with_camera"', 1)]
    parts.append(HEADER_NOTE)
    parts.append(
        '  <link name="camera_plate">\n'
        + _inertial(plate_mass, plate_com, plate_inertia)
        + '    <visual>\n'
          '      <origin xyz="0 0 0" rpy="0 0 0" />\n'
          '      <geometry>\n'
          '        <mesh filename="meshes/camera_plate.STL" />\n'
          '      </geometry>\n'
          '      <material name="">\n'
          '        <color rgba="0.75 0.75 0.78 1" />\n'
          '      </material>\n'
          '    </visual>\n'
          '    <collision>\n'
          '      <origin xyz="0 0 0" rpy="0 0 0" />\n'
          '      <geometry>\n'
          '        <mesh filename="meshes/camera_plate.STL" />\n'
          '      </geometry>\n'
          '    </collision>\n'
          '  </link>\n'
          '  <joint name="camera_plate_joint" type="fixed">\n'
          '    <origin xyz="0 0 0" rpy="0 0 0" />\n'
          '    <parent link="base_link" />\n'
          '    <child link="camera_plate" />\n'
          '  </joint>\n')
    parts.append(
        '  <link name="wrist_camera">\n'
        + _inertial(CAMERA_MASS_KG, CAMERA_BOX_CENTER_M, cam_inertia)
        + '    <visual>\n'
          f'      <origin xyz="{_xyz(CAMERA_BOX_CENTER_M)}" rpy="0 0 0" />\n'
          '      <geometry>\n'
          f'        <box size="{_xyz(CAMERA_BOX_M)}" />\n'
          '      </geometry>\n'
          '      <material name="">\n'
          '        <color rgba="0.15 0.15 0.15 1" />\n'
          '      </material>\n'
          '    </visual>\n'
          '    <collision>\n'
          f'      <origin xyz="{_xyz(CAMERA_BOX_CENTER_M)}" rpy="0 0 0" />\n'
          '      <geometry>\n'
          f'        <box size="{_xyz(CAMERA_BOX_M)}" />\n'
          '      </geometry>\n'
          '    </collision>\n'
          '  </link>\n'
          '  <joint name="wrist_camera_joint" type="fixed">\n'
          f'    <origin xyz="{_xyz(CAM_MOUNT_XYZ_M)}" rpy="{_xyz(cam_rpy)}" />\n'
          '    <parent link="camera_plate" />\n'
          '    <child link="wrist_camera" />\n'
          '  </joint>\n')
    parts.append(
        '  <link name="wrist_camera_optical" />\n'
        '  <joint name="wrist_camera_optical_joint" type="fixed">\n'
        f'    <origin xyz="0 0 0" rpy="0 0 {_fmt(math.pi)}" />\n'
        '    <parent link="wrist_camera" />\n'
        '    <child link="wrist_camera_optical" />\n'
        '  </joint>\n')
    parts.append("</robot>\n")
    return "".join(parts)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", type=pathlib.Path, default=None,
                    help="the vendor plate STL (mm); omit to only regenerate "
                         "the URDF from the committed mesh")
    args = ap.parse_args(argv)

    mesh_out = DEST / "meshes" / "camera_plate.STL"
    if args.source is not None:
        digest = hashlib.sha256(args.source.read_bytes()).hexdigest()
        print(f"source SHA256: {digest}")
        if digest != SOURCE_SHA256:
            print("WARNING: not the drop this script was written against — "
                  "re-check every constant in this file")
        tris = read_stl(args.source) * 1e-3
        write_stl(mesh_out, tris, "D1 gripper camera plate V2.0, metres")
        print(f"wrote {mesh_out} ({len(tris)} triangles)")
    tris = read_stl(mesh_out)
    verify(tris)
    mass, com, inertia = mesh_mass_properties(tris, ALUMINIUM_KG_M3)
    print(f"plate: {mass * 1e3:.1f} g at {ALUMINIUM_KG_M3:.0f} kg/m^3, "
          f"COM ({', '.join(f'{c * 1e3:.2f}' for c in com)}) mm")
    out = DEST / "gripper_with_camera.urdf"
    out.write_text(compose_urdf(mass, com, inertia))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
