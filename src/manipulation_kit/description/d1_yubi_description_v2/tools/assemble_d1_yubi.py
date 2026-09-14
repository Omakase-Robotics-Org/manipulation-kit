#!/usr/bin/env python3
"""Generate ``d1_yubi_description_v2/urdf/d1_yubi.urdf`` — the mesh-bearing
D1 dual-arm + YUBI hand model.

    python3 description/d1_yubi_description_v2/tools/assemble_d1_yubi.py
    python3 ... /assemble_d1_yubi.py --out-dir /tmp/x     # write elsewhere

**Edit this generator, never ``urdf/d1_yubi.urdf``.**
``devices/omakase_arm/pyguard/tests/test_description_consistency.py``
regenerates and diffs, so a hand-edited URDF fails the suite.

## Where the numbers come from

- **Arm chains + arm STL visuals/inertials** — read from the single vendor
  copy in this repository, ``description/d1_arm/{right,left}/
  d1_arm_{right,left}.urdf`` (SolidWorks export, 7 revolute joints
  ``Joint1..7_{R,L}`` + fixed ``JointTCP_{R,L}``). Nothing about the arm is
  retyped here. Mesh URIs are rewritten to this package's own mesh tree,
  whose STLs are byte-identical copies of the ``d1_arm`` ones (the
  consistency test asserts that, so the two trees cannot silently fork).
- **Torso column and shoulder mounts** — measured from the D1 STEP assembly:
  each arm base sits at y = ±0.037 m (74 mm shoulder separation) on the
  column, rolled ∓90° so the base axis points laterally outward. The CAD
  height is 0.50 m; the shoulders ride 53.829712 mm higher than that on the
  built robot, because its telescoping lift cover is longer than the CAD
  models it (see ``generate_d1_urdf.py``, MOVING LIFT COLUMN EXTENSION).
- **YUBI hand** — the AIRoA ``yubi_description`` ``yubi_hand`` macro
  reproduced inline, with two deliberate deviations: inertials are added
  (the macro is visual-only) and the ``<mimic>`` left finger is emitted as an
  independent revolute joint, because Genesis does not implement ``<mimic>``
  (its demo scripts drive left = −right).

## The hand mount — read before "fixing" the left/right asymmetry

Both D1 arms are the SAME physical arm, so the two TCP flanges land 180°
apart under mirror-symmetric joint values and the hand is physically mounted
180° rotated on the ``_L`` side. That is why ``YUBI_MOUNT_RPY['L']`` is not
``YUBI_MOUNT_RPY['R']``. Full derivation and measurements:
``description/d1-arm-notes.md``.

The vendor's own drop ships the SAME mount transform on both sides, which is
wrong for one of them: it puts the left wrist camera upside down. Measured in
the Genesis simulation at the D1's home pose (``d1-manip-sim``
``scripts/diag_gripper_dir.py`` / ``scan_mount_rpy.py``), with the mount's
local +x the finger/approach axis and local +z the camera axis:

    raw vendor rpy on R:  approach outward (dot +0.95), camera up   (+0.78)
    raw vendor rpy on L:  approach outward (dot +0.95), camera DOWN (−0.78)

A 180° flip about the mount's local **x** re-ups the left camera while
keeping the approach axis outward; a flip about local y or z reverses the
approach axis (fingers point back at the flange). The left offset's
y-component also flips sign, because the two TCP frames are mirror-symmetric
in position.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))          # d1_yubi_description_v2
DESCRIPTION = os.path.normpath(os.path.join(PKG, ".."))   # description/
OUT = os.path.join(PKG, "urdf", "d1_yubi.urdf")

PKG_NAME = "d1_yubi_description_v2"

# The single vendor copy of the arm, in this repository.
VENDOR_SIDE = {"R": "right", "L": "left"}
ARM_URDF = {
    s: os.path.join(DESCRIPTION, "d1_arm", VENDOR_SIDE[s],
                    f"d1_arm_{VENDOR_SIDE[s]}.urdf")
    for s in ("R", "L")
}
# This package's copy of those meshes (byte-identical; see module docstring).
ARM_MESH_URI = {s: f"package://{PKG_NAME}/d1_arm_yubi_description/meshes/"
                   f"d1_arm_{s.lower()}"
                for s in ("R", "L")}
YUBI_MESH_URI = f"package://{PKG_NAME}/yubi_description/meshes"

# Torso column + shoulder mounts, measured from the D1 STEP assembly, plus the
# MOVING LIFT COLUMN EXTENSION: the built robot's telescoping cover is
# 53.829712 mm longer than the CAD models, so the shoulders (and the stylised
# column below them) sit that much higher on it. One number, one source —
# generate_d1_urdf.py::LIFT_COLUMN_EXTENSION; test_description_consistency
# pins the two together.
LIFT_COLUMN_EXTENSION = 0.053829712
SHOULDER_Z = 0.50 + LIFT_COLUMN_EXTENSION
TORSO_MOUNT = {"R": (f"0 0.037 {SHOULDER_Z:.6g}", "-1.5708 0 0"),
               "L": (f"0 -0.037 {SHOULDER_Z:.6g}", "1.5708 0 0")}

# TCP_Link_{side} -> hand_root.  R keeps the vendor transform; L is the same
# mount rotated 180 deg about its own x axis with the y offset sign flipped.
YUBI_MOUNT_XYZ = {"R": "0 -0.019 0.055", "L": "0 0.019 0.055"}
_VENDOR_MOUNT_RPY = "1.5708 -1.5708 0"

# Official palm / camera / finger geometry (yubi_description yubi_hand macro).
PALM_SIZE = "0.044 0.067 0.0735"
PALM_ORIGIN = "-0.022 0 -0.02015"
CAM_JOINT_XYZ = "0 0 0.0426"
CAM_BOX_SIZE = "0.035 0.032 0.042"
CAM_BOX_ORIGIN = "-0.0175 0 0"
RIGHT_FINGER_XYZ = "-0.016 -0.015 0"
LEFT_FINGER_XYZ = "-0.016 0.015 0"
FINGER_LOWER = 0.0
FINGER_UPPER = 0.94

PREFIX = {"R": "right", "L": "left"}   # sim side -> yubi_hand macro hand_prefix
YUBI_GRAY = "1.0 1.0 1.0 1"


# ------------------------------------------------------------------ rotations
def _rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]

    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]

    return mm(Rz, mm(Ry, Rx))   # URDF fixed-axis XYZ = Rz . Ry . Rx


def _mat_to_rpy(R):
    p = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    y = math.atan2(R[1][0], R[0][0])
    r = math.atan2(R[2][1], R[2][2])
    return r, p, y


def _flip_local(rpy_str: str, axis: str) -> str:
    """Post-multiply a 180 deg rotation about the frame's OWN `axis`."""
    r, p, y = (float(v) for v in rpy_str.split())
    R = _rpy_to_mat(r, p, y)
    F = {"x": [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
         "y": [[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
         "z": [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]}[axis]
    Rn = [[sum(R[i][k] * F[k][j] for k in range(3)) for j in range(3)]
          for i in range(3)]
    return " ".join(f"{v:.4f}" for v in _mat_to_rpy(Rn))


YUBI_MOUNT_RPY = {"R": _VENDOR_MOUNT_RPY,
                  "L": _flip_local(_VENDOR_MOUNT_RPY, "x")}   # 1.5708 -1.5708 3.1416


# --------------------------------------------------------------------- bodies
def arm_body(side: str) -> str:
    """The vendor arm URDF's links and joints, mesh URIs repointed at this
    package's mesh tree and the ``<robot>`` wrapper stripped.  Link and joint
    names (``Base_{S}``, ``Link1..7_{S}``, ``TCP_Link_{S}``, ``Joint*_{S}``)
    are already side-suffixed in the vendor files."""
    with open(ARM_URDF[side]) as f:
        raw = f.read()
    raw = re.sub(rf'package://d1_arm_{VENDOR_SIDE[side]}/meshes',
                 ARM_MESH_URI[side], raw)
    root = ET.fromstring(raw)
    return "\n".join(ET.tostring(el, encoding="unicode") for el in root)


def yubi_subtree(side: str) -> str:
    p = f"yubi_{side}"
    hp = PREFIX[side]
    rfl = f"{YUBI_MESH_URI}/hand_{hp}_right_finger_link.STL"
    rfc = f"{YUBI_MESH_URI}/hand_{hp}_right_finger_link_col.STL"
    lfl = f"{YUBI_MESH_URI}/hand_{hp}_left_finger_link.STL"
    lfc = f"{YUBI_MESH_URI}/hand_{hp}_left_finger_link_col.STL"
    mount_rpy = YUBI_MOUNT_RPY[side]
    mount_xyz = YUBI_MOUNT_XYZ[side]
    return f"""
  <!-- ===== YUBI hand (yubi_description macro, hand_prefix={hp}) on arm {side}.
       Both D1 arms are the same physical arm, so the flanges land 180 deg apart
       and the hand is mounted 180 deg rotated on _L (about the mount's local x,
       y offset sign flipped) — that is what makes the two HANDS mirror-symmetric
       (fingers away from the flange, wrist camera up) on both arms.  See
       description/d1-arm-notes.md.
       TCP_Link_{side} -> hand_root  xyz="{mount_xyz}"  rpy="{mount_rpy}". ===== -->
  <joint name="{p}_flange" type="fixed">
    <origin xyz="{mount_xyz}" rpy="{mount_rpy}"/>
    <parent link="TCP_Link_{side}"/>
    <child link="{p}_hand_root"/>
  </joint>
  <link name="{p}_hand_root">
    <inertial>
      <origin xyz="-0.022 0 -0.02015" rpy="0 0 0"/>
      <mass value="0.26"/>
      <inertia ixx="2.5e-4" ixy="0" ixz="0" iyy="2.5e-4" iyz="0" izz="2.5e-4"/>
    </inertial>
    <visual>
      <origin xyz="{PALM_ORIGIN}" rpy="0 0 0"/>
      <geometry><box size="{PALM_SIZE}"/></geometry>
      <material name="yubi_gray"><color rgba="{YUBI_GRAY}"/></material>
    </visual>
    <collision>
      <origin xyz="{PALM_ORIGIN}" rpy="0 0 0"/>
      <geometry><box size="{PALM_SIZE}"/></geometry>
    </collision>
  </link>

  <joint name="{p}_camera_joint" type="fixed">
    <origin xyz="{CAM_JOINT_XYZ}" rpy="0 0 0"/>
    <parent link="{p}_hand_root"/>
    <child link="{p}_hand_cam_link"/>
  </joint>
  <link name="{p}_hand_cam_link">
    <inertial>
      <origin xyz="{CAM_BOX_ORIGIN}" rpy="0 0 0"/>
      <mass value="0.03"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
    <visual>
      <origin xyz="{CAM_BOX_ORIGIN}" rpy="0 0 0"/>
      <geometry><box size="{CAM_BOX_SIZE}"/></geometry>
      <material name="yubi_cam"><color rgba="0.10 0.45 0.95 1"/></material>
    </visual>
    <collision>
      <origin xyz="{CAM_BOX_ORIGIN}" rpy="0 0 0"/>
      <geometry><box size="{CAM_BOX_SIZE}"/></geometry>
    </collision>
  </link>

  <joint name="{p}_right_finger_joint" type="revolute">
    <origin xyz="{RIGHT_FINGER_XYZ}" rpy="0 0 0"/>
    <parent link="{p}_hand_root"/>
    <child link="{p}_right_finger_link"/>
    <axis xyz="0 0 -1"/>
    <limit lower="{FINGER_LOWER}" upper="{FINGER_UPPER}" effort="5" velocity="2"/>
  </joint>
  <link name="{p}_right_finger_link">
    <inertial>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <mass value="0.04"/>
      <inertia ixx="2e-5" ixy="0" ixz="0" iyy="2e-5" iyz="0" izz="2e-5"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="{rfl}"/></geometry>
      <material name="yubi_gray"><color rgba="{YUBI_GRAY}"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="{rfc}"/></geometry>
    </collision>
  </link>

  <!-- Left finger.  The macro mimics the right finger; Genesis has no <mimic>,
       so this stays an independent joint (demo scripts drive left = -right). -->
  <joint name="{p}_left_finger_joint" type="revolute">
    <origin xyz="{LEFT_FINGER_XYZ}" rpy="0 0 0"/>
    <parent link="{p}_hand_root"/>
    <child link="{p}_left_finger_link"/>
    <axis xyz="0 0 -1"/>
    <limit lower="{-FINGER_UPPER}" upper="{-FINGER_LOWER}" effort="5" velocity="2"/>
  </joint>
  <link name="{p}_left_finger_link">
    <inertial>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <mass value="0.04"/>
      <inertia ixx="2e-5" ixy="0" ixz="0" iyy="2e-5" iyz="0" izz="2e-5"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="{lfl}"/></geometry>
      <material name="yubi_gray"><color rgba="{YUBI_GRAY}"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="{lfc}"/></geometry>
    </collision>
  </link>
"""


def torso() -> str:
    return f"""
  <link name="dual_base"/>
  <link name="torso_column">
    <visual>
      <origin xyz="0 0 {SHOULDER_Z / 2:.6g}" rpy="0 0 0"/>
      <geometry><box size="0.09 0.11 {SHOULDER_Z:.6g}"/></geometry>
      <material name="torso_grey"><color rgba="0.35 0.38 0.42 1"/></material>
    </visual>
    <visual>
      <origin xyz="0 0 {SHOULDER_Z:.6g}" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="0.045" length="0.085"/></geometry>
      <material name="shoulder_grey"><color rgba="0.30 0.33 0.37 1"/></material>
    </visual>
  </link>
  <joint name="torso_mount" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="dual_base"/>
    <child link="torso_column"/>
  </joint>
  <joint name="mount_R" type="fixed">
    <origin xyz="{TORSO_MOUNT['R'][0]}" rpy="{TORSO_MOUNT['R'][1]}"/>
    <parent link="torso_column"/>
    <child link="Base_R"/>
  </joint>
  <joint name="mount_L" type="fixed">
    <origin xyz="{TORSO_MOUNT['L'][0]}" rpy="{TORSO_MOUNT['L'][1]}"/>
    <parent link="torso_column"/>
    <child link="Base_L"/>
  </joint>
"""


def build() -> str:
    body = torso()
    for s in ("R", "L"):
        body += "\n" + arm_body(s) + "\n" + yubi_subtree(s)
    return ('<?xml version="1.0" encoding="utf-8"?>\n'
            "<!-- AUTO-GENERATED by description/d1_yubi_description_v2/tools/\n"
            "     assemble_d1_yubi.py from description/d1_arm/.\n"
            "     Edit the generator, not this file. -->\n"
            '<robot name="d1_yubi">\n' + body + "\n</robot>\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", help="write d1_yubi.urdf here instead of "
                                      "d1_yubi_description_v2/urdf/")
    args = ap.parse_args()
    out = (os.path.join(args.out_dir, os.path.basename(OUT))
           if args.out_dir else OUT)
    text = build()
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(text)
    root = ET.parse(out).getroot()
    print(f"wrote {os.path.normpath(out)} ({len(text)} bytes, "
          f"{len(root.findall('link'))} links, {len(root.findall('joint'))} joints)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
