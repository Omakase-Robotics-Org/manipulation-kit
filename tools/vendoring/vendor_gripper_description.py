#!/usr/bin/env python3
"""Re-derive ``descriptions/`` from the vendor ``gripper`` ROS package drop.

The committed description is NOT hand-edited — it is the output of this
script, so a fresh vendor drop can be re-vendored the same way instead of
being patched by hand::

    python tools/vendoring/vendor_gripper_description.py \
        --source /path/to/gripper            # the unpacked ROS package

What it does, and why (each of these is a deliberate change from the raw
vendor drop — see ``descriptions/README.md``):

1. **Mesh paths** ``package://gripper/meshes/X.STL`` -> ``meshes/X.STL``.
   ``package://`` only resolves inside a ROS workspace; this package ships
   as Python package data and is resolved with :mod:`importlib.resources`,
   so the reference has to be relative to the URDF.
2. **``j6_Link.STL`` -> ``base_link.STL``.** The mesh keeps the name of the
   ARM link the gripper was extracted off (the vendor exported it out of a
   6-axis arm assembly); the link it actually belongs to is ``base_link``.
   A mesh named after a joint that does not exist in this file is a trap.
3. **``velocity="0"`` -> a real limit.** A zero velocity limit is a CAD
   export artefact. Planners and simulators that honour joint limits read it
   as "this joint cannot move" and refuse to plan/actuate the fingers.
4. **Meshes decimated**, component-aware: the six M3 cap screws (0.035 cm^3
   each, 60 k of the 113 k triangles between them, invisible at any sane
   render scale) are dropped, and the remaining shells are simplified with a
   per-component budget. Vertices are then clamped back inside the original
   CAD bounding box so the decimated shell can never stick out past the real
   part. Same policy as ``leadshine/dh116s/descriptions`` (visual quality
   only — kinematics and joint limits are untouched).
5. **Inertials corrected to the measured 1.5 kg.** The CAD sums to 0.3279 kg;
   Shu weighed the gripper at 1.5 kg (2026-07-29), confirming the value
   d1-sdk has always registered. The CAD export models outer shells only, so
   the missing 1.1721 kg is put on ``base_link`` — see MASS below for why it
   can go nowhere else — and its COM is solved so the ASSEMBLY COM lands on
   the hardware-validated 68 mm. Marked in the file as an estimate.
6. **Jaw joints retargeted to the MEASURED tool geometry.** Shu put callipers
   on d1-3 on 2026-09-16: the pads sit 71 … 129 mm from the flange face, so
   their CENTRE is 100 mm, not the CAD's 108.47 mm, and the pad faces open to
   64 mm, not the CAD's 70 mm. Both jaw joints are moved to the measured pad
   centre and their limits to ± 32 mm. This is the FIRST edit here that
   changes vendor kinematics rather than an export artefact, and it is made
   on purpose: a CAD number that disagrees with a calliper on the assembled
   robot is wrong about the robot. The jaw MESHES are untouched — they still
   reach 6 mm past the measured pad tip — because replacing a mesh needs a
   CAD drop, not a measurement. See GEOMETRY below.

Needs ``trimesh`` and ``fast-simplification`` (dev-only; neither is a
dependency of this package).
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys

from _kit_paths import parallel_gripper

HERE = pathlib.Path(__file__).resolve().parent
DEST = parallel_gripper() / "descriptions"

#: Total triangle budget for the base-link shell after decimation. 24 k keeps
#: the enclosed volume within 0.6 % of the CAD and the file near 1.2 MB.
FACE_BUDGET = 24_000

#: Components below this volume are dropped entirely (the cap screws).
MIN_COMPONENT_VOLUME_M3 = 1e-7

#: Finger-jaw travel speed, m/s, replacing the vendor's ``velocity="0"``.
#: NOT a measured spec — see ``descriptions/README.md``. It is the order of
#: magnitude of a 32 mm stroke closing in well under a second, chosen so the
#: limit is permissive rather than binding.
FINGER_VELOCITY_MPS = 0.05

# --------------------------------------------------------------------------
# MASS
#
# Shu weighed the gripper at the robot on 2026-07-29: 1.5 kg, confirming the
# value d1-sdk has registered all along. The CAD sums to 0.3279 kg — it is
# light by 1.1721 kg, a factor of 4.6.
#
# Where the missing mass goes is solved, not guessed:
#
#   * NOT on the jaws. Putting it there needs 15 065 kg/m^3, denser than lead.
#   * On ``base_link`` it is 1.4169 kg over the 143.2 cm^3 of MODELLED shell =
#     9 894 kg/m^3, denser than steel. So the mesh is incomplete too, which is
#     the actual finding: this is a shell-only export. Over the body's
#     bounding envelope (585.6 cm^3, only 24 % filled by the mesh) the same
#     mass is 2 420 kg/m^3 — entirely ordinary for a housing containing a
#     motor, gearbox, leadscrew and PCB that the export left out.
#   * The implied CAD densities confirm it: 1 709 kg/m^3 for the body and
#     998 kg/m^3 for the jaws. Neither is a metal, and 998 is water. These are
#     default/unassigned material values, not a weighed assembly.
#
# So the housing absorbs it, and ``base_link``'s COM is solved so the
# ASSEMBLY COM lands on the registered 68 mm — which is the one COM figure
# with hardware behind it (the wrist stopped sagging when the lever moved
# from 0 to 68 mm; see d1-sdk arm.h). Keeping the CAD's 34.6 mm would put the
# assembly COM at 38 mm, contradicting that by 30 mm.
#
# The jaw inertials are left VERBATIM: at 41.6 g each they are 5.5 % of the
# total, so their default-material density barely moves the assembly.
MEASURED_MASS_KG = 1.5                    # scale, Shu 2026-07-29
REGISTERED_COM_Z_M = 0.068                # d1-sdk defaultGripper(), validated
CAD_JAW_MASS_KG = 0.0415744002939907      # kept verbatim
CAD_BODY_MASS_KG = 0.244768569107893
CAD_BODY_INERTIA = {                      # about the CAD body COM
    "ixx": 5.95602136134452E-05, "ixy": 2.09446788564747E-07,
    "ixz": 1.58245845430587E-07, "iyy": 0.000149186167752524,
    "iyz": 5.69247390237629E-09, "izz": 0.000187224448225777,
}

# --------------------------------------------------------------------------
# GEOMETRY
#
# MEASURED on d1-3 2026-09-16 by Shu with callipers, along the tool axis from
# the Marvin arm flange face outward:
#
#     0 …   2 mm   camera mounting plate     (the CAD plate drop said 8 mm)
#     2 …   9 mm   spacer block              (was an ASSUMED 8 … 16.5 mm band)
#     9 …  51 mm   gripper body
#    51 …  71 mm   finger base plate
#    71 … 129 mm   pads, 58 mm of graspable depth
#                  pad CENTRE 100 mm, pad TIP 129 mm
#    maximum opening, pad face to pad face: 64 mm
#
# The vendor CAD hangs both jaw links at z = 108.47 mm and opens them to
# 70 mm. It is 8.47 mm long at the pad centre and 6 mm long at the tip (its
# jaw MESH reaches 143.5 mm against the measured 129 mm), and 6 mm wide at the
# open stop. The calliper wins for the kinematics; the mesh is left alone.
#
# Consequence for the mass solve below: the jaw COM rides the joint origin, so
# moving the origin 8.47 mm back moves base_link's SOLVED COM with it. The
# assembly COM stays on the hardware-validated 68 mm, which is the invariant
# that matters — see MASS above.
JAW_CENTRE_Z_M = 0.100          # flange face -> pad centre, MEASURED
JAW_TIP_Z_M = 0.129             # flange face -> pad tip, MEASURED
JAW_STROKE_M = 0.032            # one finger; 64 mm pad-to-pad, MEASURED
CAD_JAW_CENTRE_Z_M = 0.10847    # what the CAD said, kept for the record
CAD_JAW_STROKE_M = 0.035        # what the CAD said, kept for the record

#: Jaw COM in the base frame, at the MEASURED joint origin.
CAD_JAW_COM_Z_M = -0.00902677214251069 + JAW_CENTRE_Z_M

MESH_RENAMES = {"j6_Link.STL": "base_link.STL"}

#: Replaces the vendor's own header, which names a mesh this file no longer
#: references. The vendor text is quoted verbatim in descriptions/README.md.
#: NB: an XML comment may not contain a double hyphen anywhere in its body —
#: no parser will read the file if it does. Keep this text hyphen-pair free.
HEADER = """<?xml version="1.0" encoding="utf-8"?>
<!-- D1 stock parallel gripper, vendor CAD (SolidWorks URDF export).
     VENDORED by tools/vendoring/vendor_gripper_description.py
     from the vendor ROS package `gripper`; do not hand edit, re-run the
     script. Provenance, the exact source hashes and every local change:
     descriptions/README.md.

     Frame: base_link is the arm tool flange, fingers extend along +Z, the
     jaws travel along +/-X.

     GEOMETRY along +Z is MEASURED, not CAD: Shu put callipers on d1-3 on
     2026-09-16 and read 2 mm camera plate, 7 mm spacer, 42 mm body, 20 mm
     finger base plate, 58 mm pads. So the pads run 71 .. 129 mm from the
     flange face, the jaw links hang at their CENTRE (Z = 100 mm) and the
     registered TCP is the pad TIP, Z = 129 mm (see toolconfig.py). The
     earlier 108.5 mm centre, 143.5 mm tip and 136 mm TCP were vendor CAD and
     a d1-sdk default carried over with it, and were long by 8.5 / 14.5 / 7 mm.
     The jaw MESHES still reach the CAD's 143.5 mm; replacing a mesh needs a
     CAD drop, so that residual is recorded rather than hidden.

     Jaw convention: tcp_r_joint spans 0 .. 0.032 m and tcp_l_joint mimics it
     with multiplier minus one over 0.032 .. 0 m. Both jaws move inward as
     |q| grows, so q = 0 is the MEASURED 64 mm OPEN gap and |q| = 0.032 is
     fully CLOSED. This is the OPPOSITE polarity to the CAN 2.0 wire command,
     where 0.0 is closed. Do not wire one to the other without inverting.

     MASS: the link masses here sum to the MEASURED 1.5 kg (scale, Shu
     2026-07-29), not to the 0.3279 kg the CAD export claimed. That export
     models outer shells only and its implied densities are default material
     values, so it was light by a factor of 4.6. The missing mass is carried
     on base_link, whose inertial is therefore an ESTIMATE: the mass is
     measured, the COM is solved so the assembly COM lands on the
     hardware-validated 68 mm, and the tensor is the CAD tensor scaled by the
     mass ratio. The jaw inertials are the CAD's, verbatim. Full derivation:
     descriptions/README.md and toolconfig.py. -->
"""


def rewrite_urdf(text: str) -> str:
    """Apply the URDF edits (1), (2) and (3) above."""
    head, sep, body = text.partition("<robot")
    if not sep:
        raise SystemExit("no <robot> element in the vendor URDF")
    text = HEADER + sep + body
    for old, new in MESH_RENAMES.items():
        text = text.replace(f"package://gripper/meshes/{old}", f"meshes/{new}")
    text = text.replace("package://gripper/meshes/", "meshes/")
    if "package://" in text:
        raise SystemExit("unresolved package:// reference left in the URDF")
    text, n = re.subn(r'velocity="0"', f'velocity="{FINGER_VELOCITY_MPS}"', text)
    if n != 2:
        raise SystemExit(f"expected 2 zero velocity limits to patch, found {n}")
    return _rewrite_base_inertial(_rewrite_jaw_geometry(text))


def _rewrite_jaw_geometry(text: str) -> str:
    """Move the jaw joints onto the MEASURED pad centre and stroke (edit 6).

    Both jaws hang off one origin with one axis — they are one rail — so both
    origins and both limit pairs move together, and a drop that does not have
    exactly two of each is a drop this script has not been read against.
    """
    text, n = re.subn(rf'xyz="0 0 {CAD_JAW_CENTRE_Z_M:g}"',
                      f'xyz="0 0 {JAW_CENTRE_Z_M:g}"', text)
    if n != 2:
        raise SystemExit(
            f"expected 2 jaw joint origins at z = {CAD_JAW_CENTRE_Z_M} m to "
            f"retarget onto the measured {JAW_CENTRE_Z_M} m, found {n}")
    text, n = re.subn(rf'"(-?){CAD_JAW_STROKE_M:g}"',
                      rf'"\g<1>{JAW_STROKE_M:g}"', text)
    if n != 2:
        raise SystemExit(
            f"expected 2 jaw stroke limits of {CAD_JAW_STROKE_M} m to "
            f"retarget onto the measured {JAW_STROKE_M} m, found {n}")
    return text


def body_inertial():
    """``base_link``'s corrected inertial: (mass, com_z, inertia dict).

    Mass is measured (:data:`MEASURED_MASS_KG` minus the two CAD jaws). COM z
    is SOLVED so the assembly COM equals :data:`REGISTERED_COM_Z_M`. The
    tensor is the CAD tensor scaled by the mass ratio — a first-order estimate,
    since where exactly the un-modelled internals sit is unknown.
    """
    mass = MEASURED_MASS_KG - 2 * CAD_JAW_MASS_KG
    com_z = (MEASURED_MASS_KG * REGISTERED_COM_Z_M
             - 2 * CAD_JAW_MASS_KG * CAD_JAW_COM_Z_M) / mass
    scale = mass / CAD_BODY_MASS_KG
    return mass, com_z, {k: v * scale for k, v in CAD_BODY_INERTIA.items()}


def _rewrite_base_inertial(text: str) -> str:
    """Replace ``base_link``'s CAD inertial with the measured-mass one."""
    mass, com_z, inertia = body_inertial()
    start = text.index("<inertial>")          # base_link is the first link
    end = text.index("</inertial>", start) + len("</inertial>")
    block = (
        '<inertial>\n'
        '      <!-- ESTIMATE, not the CAD values. Mass is the MEASURED 1.5 kg\n'
        '           assembly (Shu 2026-07-29) minus the two CAD jaws; the CAD\n'
        '           export claimed 0.2448 kg for this link and models outer\n'
        '           shells only. COM z is solved so the ASSEMBLY COM lands on\n'
        '           the hardware validated 68 mm. The tensor is the CAD tensor\n'
        '           scaled by the mass ratio. descriptions/README.md has the\n'
        f'           derivation. CAD original: mass 0.244768569107893, COM\n'
        f'           (0.000188064059438092, -0.0101851841222577,\n'
        f'           0.0346226600011052). -->\n'
        f'      <origin\n        xyz="0 0 {com_z:.12G}"\n        rpy="0 0 0" />\n'
        f'      <mass\n        value="{mass:.12G}" />\n'
        '      <inertia\n'
        + "\n".join(f'        {k}="{v:.12G}"' for k, v in inertia.items())
        + ' />\n    </inertial>'
    )
    return text[:start] + block + text[end:]


def decimate(src_dir: pathlib.Path, dst_dir: pathlib.Path) -> None:
    import numpy as np
    import trimesh
    import fast_simplification

    dst_dir.mkdir(parents=True, exist_ok=True)
    for src in sorted(src_dir.glob("*.STL")):
        name = MESH_RENAMES.get(src.name, src.name)
        mesh = trimesh.load_mesh(src)
        if len(mesh.faces) <= FACE_BUDGET:
            mesh.export(dst_dir / name, file_type="stl")
            print(f"  {src.name} -> {name}: {len(mesh.faces)} faces (as-is)")
            continue
        keep = [c for c in mesh.split(only_watertight=False)
                if abs(c.volume) >= MIN_COMPONENT_VOLUME_M3]
        total = sum(len(c.faces) for c in keep)
        parts = []
        for comp in keep:
            target = max(200, int(FACE_BUDGET * len(comp.faces) / total))
            if target >= len(comp.faces):
                parts.append(comp)
                continue
            verts, faces = fast_simplification.simplify(
                np.asarray(comp.vertices, np.float32),
                np.asarray(comp.faces, np.int32),
                1.0 - target / len(comp.faces))
            parts.append(trimesh.Trimesh(vertices=verts, faces=faces, process=False))
        out = trimesh.util.concatenate(parts)
        out.vertices = np.clip(out.vertices, mesh.bounds[0], mesh.bounds[1])
        out.export(dst_dir / name, file_type="stl")
        print(f"  {src.name} -> {name}: {len(mesh.faces)} -> {len(out.faces)} faces, "
              f"volume error {abs(out.volume - mesh.volume) / mesh.volume * 100:.2f}%, "
              f"bbox growth {float((abs(out.bounds - mesh.bounds)).max()) * 1000:.3f} mm")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True, type=pathlib.Path,
                    help="unpacked vendor 'gripper' ROS package")
    args = ap.parse_args(argv)

    src_urdf = args.source / "urdf" / "gripper.urdf"
    src_meshes = args.source / "meshes"
    if not src_urdf.is_file() or not src_meshes.is_dir():
        raise SystemExit(f"{args.source} does not look like the vendor package")

    print("source SHA256:")
    for p in sorted(args.source.rglob("*")):
        if p.is_file():
            print(f"  {hashlib.sha256(p.read_bytes()).hexdigest()}  "
                  f"{p.relative_to(args.source)}")

    DEST.mkdir(parents=True, exist_ok=True)
    (DEST / "gripper.urdf").write_text(rewrite_urdf(src_urdf.read_text()))
    print(f"wrote {DEST / 'gripper.urdf'}")
    decimate(src_meshes, DEST / "meshes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
