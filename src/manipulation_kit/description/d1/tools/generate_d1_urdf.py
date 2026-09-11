#!/usr/bin/env python3
"""Generate the description/d1/*.urdf FULL-BODY D1 collision models.

Run from anywhere:  python3 description/d1/tools/generate_d1_urdf.py
(no arguments = regenerate every file in OUTPUTS; `--only <name>` for one)

PURPOSE
-------
A single URDF of the whole D1 robot (chassis + lift + torso + head + both
D1 arms + an end effector) whose COLLISION geometry is SIMPLE
PRIMITIVES ONLY (boxes / cylinders / spheres — no meshes).  It exists for
software collision checking (devices/omakase_arm/pyguard) and for loading
into sims/planners on machines that have no mesh assets.

Only d1.urdf carries no meshes at all, and a test enforces it: pyguard parses it
directly and it has to load with zero assets.  The two WHOLE-BODY files carry the
real vendor CAD as `<visual>` — body, both arms and the end effector — because
d1_wholebody_gripper.urdf is the AUTHORITATIVE whole-body asset other
repositories consume (d1-isaaclab references it rather than composing its own).
Their COLLISION geometry is still primitives; see the VISUAL MESHES block.

END EFFECTORS
-------------
The hand is not a part of the robot: dx-manipulator (`manipulation_kit.hands`) owns
each one's identity, physical data and GEOMETRY under
`hands/<maker>/<model>/`, and everything addresses it by that
`"<maker>/<model>"` id.  What lives here is the COMPOSITION — the flange
transform, the collision approximation, and the robot-plus-hand assembly other
repos load — which is meaningless in the hand's own repository.  See the
END_EFFECTORS registry below and OUTPUTS for which file wears which hand.

Adding one = an entry in END_EFFECTORS plus its definitions upstream in
dx-manipulator.  The gripper STLs under description/d1/meshes/gripper/ are
byte-identical VENDORED COPIES of dx-manipulator's, checked by a test — the
same pattern the arm meshes already use between d1_arm and
d1_yubi_description_v2.  Edit them upstream, never here.

FRAME
-----
Root link `dual_base`: z up, +x forward (robot's facing), +y robot's LEFT.
The two arm shoulders sit at y = ±0.037 m, z = 0.50 m — identical to the
`mount_R` / `mount_L` joints of d1_yubi_description_v2's d1_yubi.urdf (which is
what d1-manip-sim loads), omakaseos d1_dual_description, safety_zones.json
and include/omakase_arm/{safety_zones.h, collision_model.h}.  Keep them all in
sync.  (This used to name d1-manip-sim assets/d1_dual/d1_dual.urdf;
that file was deleted when d1-manip-sim stopped keeping a private copy of the
D1 geometry and started vendoring this directory's export instead.)

PROVENANCE OF NUMBERS
---------------------
* Arm kinematics (joint origins / axes / limits): byte-derived from the
  vendor URDFs in description/d1_arm/{right,left} (D1 arm
  D1 arm; L and R are kinematically identical).
* Arm mount transforms (mount_R / mount_L): measured shoulder half-width
  ±0.037 m from the D1 STEP assembly (see d1-face/extract_arm_mounts.py),
  first written down in d1-manip-sim's d1_dual.urdf and now carried by
  d1_yubi_description_v2/urdf/d1_yubi.urdf, which is generated from here.
* Per-link capsule radii: config/safety_zones.json (conservative bounds on
  the vendor collision meshes) — same numbers as collision_model.h.
* YUBI hand mount + palm/camera/finger geometry: d1-manip-sim
  assets/d1_yubi.urdf (merged main, PR #12) and
  description/d1_arm/yubi_description/urdf/yubi_hand.urdf.xacro.
  Finger boxes = bounding boxes of the yubi_description finger collision
  STLs (computed from hand_right_*_finger_link_col.STL).
* Torso / chassis boxes: MEASURED 2026-07-01 from the D1 CAD STEP
  assembly "omakase D1 assy PKG 260607" (local copy:
  ~/Downloads/cad/omakase_D1_assy_PKG_260607.stp, byte-identical to
  d1-face/d1_face.step).  Method: prune the STEP to the torso/chassis/head
  subtrees (d1-face/extract_head.py machinery), OCP/XCAF load, per-module
  located bounding boxes + z-band slices of the body shell mesh, in the
  assembly root frame; mapped to dual_base by
      x_db = -Y_root,   y_db = X_root + 0.003,   z_db = Z_root + 0.500
  (root frame: X lateral toward robot's left with shoulder midpoint at
  X = -3 mm, -Y forward, Z up with the shoulder line at Z = 0; from
  d1-face/extract_arm_mounts.py: R-arm base at X=-40 mm, L at X=+34 mm).
* Ground height, lift travel, neck PTU, head geometry: the VENDOR body
  URDF `urdf2026072302` (SolidWorks sw_urdf_exporter 1.6.0, exported
  2026-07-23 from "…装配URDF去双目的.SLDASM"; Drive: D1/URDF/
  urdf2026072302.zip).  See VENDOR BODY block below for each number and
  how it was derived.
* D1 stock parallel gripper (mount, jaw joints, boxes, masses): the vendor
  CAD in dx-manipulator `hands/d1/parallel_gripper/descriptions/`, received
  2026-07-29.  See the GRIPPER block below.

STILL-UNMEASURED (placeholders, marked TODO in the URDF):
  - shoulder-cover shell band: modeled but exempt from guard checks (arms
    pass through it by construction).
  - camera lens position WITHIN each housing, and the 180 deg roll about the
    lens axis. See the CAMERAS block: the frames are nominal, derived from
    vendor housing geometry, and are not calibrated extrinsics.
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

DESC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

# --------------------------------------------------------------------------
# VISUAL MESHES
#
# COLLISION geometry is primitives in every file here, on purpose: it is what
# the motion guard checks, what the C++ validators mirror, and what simulators
# want for fast, stable contact.  VISUAL geometry is the real CAD, because the
# question "what does the robot look like" has a different right answer from
# "what should it collide with".  Do not collapse the two.
#
# d1.urdf carries NO meshes at all — it is the guard model, it has to load on
# the robot with zero assets, and pyguard parses it directly.  The whole-body
# variants carry the full CAD.
#
# Paths are relative TO THE URDF FILE.  The arm and YUBI meshes are used IN
# PLACE from the packages that already hold them at full resolution, so nothing
# is duplicated; only the body CAD needed importing (see
# tools/vendoring/vendor_visual_meshes.py for its provenance and why none of it is
# decimated).
ARM_MESH_DIR = {"R": "../d1_arm/right/meshes",
                "L": "../d1_arm/left/meshes"}
YUBI_MESH_DIR = "../d1_yubi_description_v2/yubi_description/meshes"
BODY_MESH_DIR = "meshes/body"

ARM_GREY = "0.90 0.91 0.92 1"
BODY_GREY = "0.92 0.92 0.94 1"

# Which vendor body meshes ride on which link of THIS model.  merged_robot
# spins its wheels and articulates its head on joints this model does not have,
# so the wheels and cameras are welded onto the body they are bolted to.
BODY_MESH_HOSTS = (
    ("chassis_link", ("base_link", "rwheel_Link", "lwheel_Link",
                      "carcamer_Link")),
    ("torso_column", ("slider_Link", "fcamer_Link", "backcamer_Link")),
    ("neck_pan_link", ("dhead_Link",)),
    ("head_link", ("uphead_Link", "headcamera_Link")),
)
BODY_URDF = os.path.join(DESC_DIR, "meshes", "body", "merged_robot.urdf")

# The body placement is SOLVED, not transcribed: for each mesh we know where
# merged_robot puts it in ITS frame, we know how that frame maps onto dual_base
# (the vendor slider frame, offset by DUAL_BASE_IN_SLIDER — the same equality
# this file already uses for the lift and neck), and the host link's own
# transform carries it the rest of the way into link-local coordinates.  A
# frame-convention difference therefore cannot silently shift a mesh: either
# the whole chain agrees, or the arm-plate check below fails the build.
#
# THE CHECK: vendor `rarmbase_Link` is at y = -0.037 = the robot's physical
# RIGHT, which is our `_L` tree, and `larmbase_Link` is our `_R`.  Aligning on
# the matching NAME instead of the matching MOUNT mirrors the whole robot.
ARM_PLATE_CHECK = {"rarmbase_Link": ("L", (0.0, -0.037, 0.50)),
                   "larmbase_Link": ("R", (0.0, 0.037, 0.50))}
ARM_PLATE_TOLERANCE_M = 0.005

#: Every file this generator owns: name -> (wholebody?, end effector id).
#: Regenerate them all with no arguments; `--only <name>` writes just one.
#: (wholebody?, end effector id, visual meshes?)
OUTPUTS = {
    # The GUARD model: primitives only and NO meshes at all. pyguard parses it
    # directly and has to load on the robot with zero assets, the C++
    # validators mirror its numbers, and its end effector stays the YUBI hand
    # because that is what all of them were built against — swapping it is a
    # safety-behaviour change, not a regeneration. This is the deliberate
    # lightweight variant; it is not the "primitive version" being retired.
    "d1.urdf": (False, "omakase/yubi", False),
    # Whole body with the base, lift and neck as real joints, wearing the YUBI
    # hand, with real CAD visuals.
    "d1_wholebody.urdf": (True, "omakase/yubi", True),
    # Same again wearing the D1 stock parallel gripper. THE authoritative
    # whole-body asset: what d1-isaaclab and anything else should reference.
    "d1_wholebody_gripper.urdf": (True, "d1/parallel_gripper", True),
}

# --------------------------------------------------------------------------
# Arm chain (vendor D1 arm; identical L/R). SDK order J1..J7.
# (rpy, xyz) of each joint origin; every joint axis is local +z.
# --------------------------------------------------------------------------
ARM_CHAIN = [
    # name    rpy                    xyz                    lower    upper    effort
    ("Joint1", (0.0, 0.0, 0.0),      (0.0, 0.0, 0.1586),   -3.0194,  3.0194, 35),
    ("Joint2", (1.5708, 0.0, 0.0),   (0.0, 0.0, 0.0),      -2.0595,  2.0595, 35),
    ("Joint3", (-1.5708, 0.0, 0.0),  (0.0, 0.264, 0.0),    -3.0194,  3.0194, 18),
    ("Joint4", (-1.5708, 0.0, 3.1416), (0.018, 0.0, 0.0),  -2.5307,  0.7854, 18),
    ("Joint5", (-1.5708, 0.0, 3.1416), (0.018, -0.264, 0.0), -3.0194, 3.0194, 5.5),
    ("Joint6", (1.5708, -1.5708, 0.0), (0.0, 0.0, 0.0),    -1.0472,  1.0472, 5.5),
    ("Joint7", (1.5708, -1.5708, 0.0), (0.0, 0.0, 0.0),    -1.5708,  1.5708, 5.5),
]
TCP_XYZ = (0.0, -0.087, 0.0)
TCP_RPY = (1.5708, -1.5708, 0.0)

# Capsule radius per link segment (Base, Link1..Link7) — safety_zones.json.
CAP_RADII = {
    "Base": 0.055, "Link1": 0.05, "Link2": 0.045, "Link3": 0.04,
    "Link4": 0.04, "Link5": 0.035, "Link6": 0.03, "Link7": 0.03,
    "TCP_Link": 0.03,
}

# Child-joint origin (= capsule far endpoint) expressed in each link's frame.
LINK_SEGMENT = {
    "Base":   (0.0, 0.0, 0.1586),      # -> Joint1
    "Link1":  (0.0, 0.0, 0.0),         # -> Joint2 (degenerate -> sphere)
    "Link2":  (0.0, 0.264, 0.0),       # -> Joint3
    "Link3":  (0.018, 0.0, 0.0),       # -> Joint4
    "Link4":  (0.018, -0.264, 0.0),    # -> Joint5
    "Link5":  (0.0, 0.0, 0.0),         # -> Joint6 (sphere)
    "Link6":  (0.0, 0.0, 0.0),         # -> Joint7 (sphere)
    "Link7":  (0.0, -0.087, 0.0),      # -> JointTCP
    "TCP_Link": (0.0, -0.019, 0.055),  # -> yubi flange
}

# Arm mounts (see PROVENANCE OF NUMBERS).  SDK ArmSide::A = "_R" = physical LEFT
# (+y); ArmSide::B = "_L" tree = physical RIGHT (-y).
MOUNTS = {
    "R": ((0.0, 0.037, 0.50), (-1.5708, 0.0, 0.0)),
    "L": ((0.0, -0.037, 0.50), (1.5708, 0.0, 0.0)),
}

# YUBI flange mounts (d1-manip-sim assets/d1_yubi.urdf, merged main).
YUBI_FLANGE = {
    "R": ((0.0, -0.019, 0.055), (1.5708, -1.5708, 0.0)),
    "L": ((0.0, 0.019, 0.055), (1.5708, -1.5708, 3.1416)),
}

# Finger collision-mesh bounding boxes (m), in the finger link frame —
# computed from yubi_description/meshes/hand_right_{right,left}_finger_link_col.STL.
FINGER_BOX = {
    "right": ((-0.0128, -0.0284, -0.0204), (0.1033, 0.0166, 0.0124)),
    "left":  ((-0.0129, -0.0166, -0.0104), (0.1035, 0.0275, 0.0125)),
}

# --------------------------------------------------------------------------
# D1 STOCK PARALLEL GRIPPER — from the vendor CAD, which lives in
# dx-manipulator `hands/d1/parallel_gripper/descriptions/gripper.urdf`
# (`manipulation_kit.hands.d1.parallel_gripper.description.load_urdf()`).  That is the
# single source of truth for the gripper's SHAPE; what is transcribed here is
# only what a primitives-only collision model needs, with the same discipline
# as every other number in this file.
#
# MOUNT.  `TCP_Link_<S>` is the arm's TOOL FLANGE: the vendor D1 arm URDF gives
# it zero mass and an empty mesh, i.e. it is a pure frame at the end of the
# chain, and FK confirms its +z points straight out along the arm.  The
# gripper's `base_link` origin IS its mounting flange, so the mount translation
# is ZERO — no adapter thickness is modelled, and the CAD's own 16.5 mm of
# clearance before its shell begins covers the plate.  Cross-check that this is
# the right frame: the registered tool config puts the TCP 136 mm along flange
# +z, and this CAD puts the jaw tips at 143.5 mm, so 136 mm lands on the jaw
# pad face.  (For comparison the YUBI jaw tips sit at 148.4 mm.)
#
# HANDEDNESS.  A parallel gripper is its own mirror image about the jaw-travel
# axis, so unlike the YUBI hand there is no left/right part — but the two
# flanges are NOT in the same orientation.  FK at the zero pose:
#     TCP_Link_R  +z outward (+y),  +y DOWN (-z),  +x forward
#     TCP_Link_L  +z outward (-y),  +y UP   (+z),  +x forward
# So mounting the same part identically on both flanges would put it upside
# down on one arm.  The gripper's +y is taken as its "up": `_L` gets identity
# and `_R` a half turn about the approach axis, which lands +y up and the jaws
# travelling fore/aft on BOTH arms.  Rotating about the approach axis also
# swaps which jaw is which, but the jaws are symmetric, so nothing else moves.
# Remember `_R` is the vendor unit on the PHYSICAL LEFT arm.  Tested — the two
# per-arm rotations of a wrist mount have been swapped in this codebase before.
GRIPPER_FLANGE = {
    "R": ((0.0, 0.0, 0.0), (0.0, 0.0, 3.14159265)),
    "L": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
}

# Jaw joints, verbatim from the CAD URDF (`tcp_r_joint` / `tcp_l_joint`).
# Both jaws hang off the same origin with the same axis and travel inward as
# |q| grows: q = 0 is the 70 mm OPEN gap, |q| = 0.035 is CLOSED.  That is the
# OPPOSITE polarity to the CAN 2.0 wire command, where 0.0 is closed.
# The vendor couples the second jaw with <mimic>; this file emits both as
# independent joints, the same choice d1_yubi.urdf made and for the same
# reason (Genesis, and several other loaders, ignore <mimic>).
GRIPPER_JAW_ORIGIN = ((0.0, 0.0, 0.10847), (3.1416, -1.5708, 0.0))
GRIPPER_JAW_AXIS = (0.0, 0.0, -1.0)
GRIPPER_JAW_LIMITS = {"r": (0.0, 0.035), "l": (-0.035, 0.0)}
GRIPPER_JAW_EFFORT = 3.0
# The CAD shipped velocity="0" (an export artefact that reads as an immovable
# joint); dx-manipulator's vendoring replaced it with this permissive
# placeholder.  NOT a datasheet figure.
GRIPPER_JAW_VELOCITY = 0.05

# Boxes bounding the CAD meshes, in each link's own frame.  base_link is two
# boxes rather than one AABB because a single one would be a 160 x 58 x 64 mm
# slab, most of it air: the mesh splits into an actuator body and a wide jaw
# guide rail sitting across the top of it.
GRIPPER_BODY_BOXES = [
    # ASSUMED, not from the CAD.  The vendor mesh has ZERO vertices below
    # z = 16.5 mm: it starts at the gripper's own 57 mm mount plate and models
    # nothing reaching back to the arm, so a 16.5 mm void sits between the
    # D1 arm tool flange face (z = 0, where Link7's mesh ends exactly) and the
    # first modelled gripper feature.  That void is real hardware, not a
    # mounting error — the registered TCP of 136 mm and the YUBI jaw tips it
    # replaces (148.4 mm vs this CAD's 143.5 mm) both confirm the gripper sits
    # where it is; shifting it 16.5 mm flusher would put the TCP at ~119.5 mm
    # and contradict the value validated on the robot.
    #
    # The first 8 mm of that gap is now REAL: the arm-end camera plate V2.0
    # (GRIPPER_PLATE_BOXES / camera_plate.STL, on its own link).  What is
    # still assumed is only the remaining z = 8 .. 16.5 mm — the gripper-end
    # plate (夹爪端), separate hardware with no CAD drop yet.  The footprint
    # is the CAD's own mount-plate square (57 x 57 mm), which BOUNDS the
    # candidate shapes.  It carries no mass — the missing 1.17 kg is already
    # on this link's inertial.
    #
    # REPLACE THIS with the vendor's gripper-end plate STL when it arrives;
    # see description/d1/README.md for the exact part being requested.
    ("gripper_adapter_ASSUMED", (-0.0285, -0.0285, 0.008), (0.0285, 0.0285, 0.0165),
     "ASSUMED gripper-end plate spanning the camera plate (z=8mm) to the "
     "CAD's mount plate (z=16.5mm); footprint bounds a round collar or a "
     "square plate. The 2026-08-23 full-robot CAD models neither this band "
     "in the plate part nor in the claw part — a caliper on the physical "
     "spacer (or the vendor gripper CAD) replaces it"),
    ("gripper_body", (-0.0285, -0.0285, 0.0165), (0.0285, 0.0285, 0.0725),
     "actuator body + mount plate + jaw carriage (CAD components >= 3.6 cm3)"),
    ("gripper_rails", (-0.0800, -0.0288, 0.0615), (0.0800, 0.0288, 0.0800),
     "jaw guide rail bar spanning the full 160 mm of jaw travel"),
]
#: The assumed-adapter box, so the render shows the part rather than a hole.
#: Drawn in a distinct colour precisely because it is NOT vendor geometry.
GRIPPER_ADAPTER_VISUAL = ((-0.0285, -0.0285, 0.008), (0.0285, 0.0285, 0.0165))
GRIPPER_ADAPTER_COLOUR = ("gripper_adapter_assumed", "0.55 0.42 0.16 1")
# Jaw AABBs from tcp_{r,l}_Link.STL, each in its own link frame.
GRIPPER_JAW_BOX = {
    "r": ((-0.03297, -0.019, 0.035), (0.03503, 0.019, 0.075)),
    "l": ((-0.03297, -0.019, -0.075), (0.03503, 0.019, -0.035)),
}
# MASS.  Shu weighed the gripper at the robot 2026-07-29: 1.5 kg per side,
# confirming the value this SDK has always registered (defaultGripper()).  The
# vendor CAD claimed 0.3279 kg — a shell-only export, light by 4.6x; its implied
# densities (1709 kg/m3 body, 998 kg/m3 jaws) are default material values, not a
# weighed assembly.  So these are the CORRECTED masses dx-manipulator now ships
# in the description, and they make the URDF agree with the tool config the
# controller is given: same object, one mass.
#
# The jaws keep their CAD mass (41.6 g each, 5.5% of the total).  base_link
# carries the rest, with its COM SOLVED so the assembly COM lands on the
# hardware-validated 68 mm — the lever that stopped the wrist sagging.  Its
# inertia tensor is the CAD tensor scaled by the mass ratio: the total is
# measured, the per-link split is an ESTIMATE.  Derivation: dx-manipulator
# hands/d1/parallel_gripper/{toolconfig.py,descriptions/README.md}.
GRIPPER_TOTAL_MASS = 1.5            # measured
GRIPPER_JAW_MASS = 0.0415744003      # CAD, kept verbatim
GRIPPER_BODY_MASS = 1.4168511994     # 1.5 - 2 jaws, exact
GRIPPER_BODY_COM = (0.0, 0.0, 0.0661547)   # solved -> assembly COM z = 68 mm
GRIPPER_BODY_INERTIA = (0.000344766, 1.21239e-06, 9.16011e-07,
                        0.000863569, 3.29511e-08, 0.00108376)
# Jaw inertia (Ixx, Ixy, Ixz, Iyy, Iyz, Izz) and COM, CAD verbatim, in the jaw
# link frame; the l jaw is the r jaw mirrored in its local z.
GRIPPER_JAW_COM = {"r": (-0.00902677, 0.0, 0.0501543),
                   "l": (-0.00902676, 0.0, -0.0501543)}
GRIPPER_JAW_INERTIA = {
    "r": (8.52933e-06, -1.1004e-12, 2.91188e-06,
          1.23028e-05, 1.00691e-11, 1.16062e-05),
    "l": (8.52933e-06, 6.97442e-11, -2.91189e-06,
          1.23029e-05, 8.22193e-12, 1.16063e-05),
}

# Gripper VISUAL meshes.  Unlike the rest of this URDF family the gripper
# variant carries meshes, because it is the asset OTHER REPOSITORIES consume
# (d1-isaaclab references it rather than composing its own).  The paths are
# RELATIVE TO THE URDF FILE, not `package://`: copy the .urdf and its meshes/
# directory together and it resolves outside ROS, from any working directory.
# Collision stays primitives, so a collision-only consumer still needs no mesh
# assets.  The STLs are byte-identical copies of dx-manipulator
# hands/d1/parallel_gripper/descriptions/meshes/ (the single source of truth) —
# the same vendored-copy-plus-byte-identity-test pattern the arm meshes already
# use between d1_arm and d1_yubi_description_v2.
GRIPPER_MESH_DIR = "meshes/gripper"
GRIPPER_MESH = {"base": "base_link.STL", "r": "tcp_r_Link.STL",
                "l": "tcp_l_Link.STL"}
# The vendor CAD's own two material colours, carried over: dark grey housing,
# green jaws.  Keeping the split is not decoration — it is the only thing that
# tells the MOVING jaws apart from the fixed housing in a viewer, and both are
# one dark shape otherwise.
GRIPPER_COLOUR = {"base": ("gripper_housing", "0.180392 0.180392 0.180392 1"),
                  "jaw": ("gripper_jaw", "0 0.501961 0.250980 1"),
                  "plate": ("gripper_camera_plate", "0.75 0.75 0.78 1"),
                  "camera": ("gripper_wrist_camera", "0.15 0.15 0.15 1")}

# --------------------------------------------------------------------------
# GRIPPER WRIST-CAMERA PLATE — the arm-end connection plate V2.0
# (夹爪连接板 手臂端, received 2026-08-21; Drive D1/URDF/, vendored into
# dx-manipulator hands/d1/parallel_gripper by its tools/vendoring/vendor_camera_plate.py
# and copied byte-identical into meshes/gripper/ like the rest).
#
# This is the first REAL geometry inside the 16.5 mm flange gap: an 8 mm
# aluminium disc (diam 72 mm, 12-hole diam-24.9 bolt circle) whose CAD origin
# IS the flange centre, so it mounts on the gripper base frame at IDENTITY —
# and its arm rises along the gripper's +y ("up" on both arms, exactly the
# axis GRIPPER_FLANGE already normalises) to hold the wide-angle UVC wrist
# camera 79 mm up, its mount face pitched 15 deg toward the fingers.
#
# The frames were CROSS-CHECKED against the robot (2026-08-21): FK at a grasp
# frame of the d1 teleop dataset puts the camera on top of the wrist exactly
# where the head camera sees the real bracket, and the real wrist streams
# show the jaws at the BOTTOM edge of the image, which is what the 15 deg
# tilt plus the optical convention below produce.  Same caveats as every
# other camera here: the lens position within the module and the module's
# own housing are NOT measured (the unit on d1-2 has no nameplate), so the
# optical origin sits ON the plate's mount face and the housing box is a
# placeholder.
#
# Mass and tensor are computed EXACTLY from the mesh (28.95 cm^3) at
# aluminium book density 2700 kg/m^3 = 78 g — an assumed density on a solid
# machined part, not a weighed one.  The camera adds a 30 g placeholder.
# Neither is in the 1.5 kg the gripper weighed at, nor in the registered
# tool config; registering the extra ~108 g is a controller-facing decision
# deliberately not taken here.
GRIPPER_PLATE_MESH = "camera_plate_full.STL"
GRIPPER_PLATE_MASS = 0.1053                # mesh volume 39.0 cm^3 x 2700 kg/m^3
GRIPPER_PLATE_COM = (0.00004, 0.03011, 0.00721)
GRIPPER_PLATE_INERTIA = (1.6509e-04, -1.4487e-07, -3.0461e-09,
                         3.4439e-05, -1.6746e-05, 1.9094e-04)
# AABBs of the plate's two parts, gripper base frame, from the full-robot
# CAD of 2026-08-23 (Reo): the flange disc lost its +y tab, and the camera
# arm became a discrete 33 mm riser — the new camera position.
GRIPPER_PLATE_BOXES = [
    ("camera_plate_disc", (-0.036, -0.036, 0.0), (0.036, 0.036, 0.008),
     "arm-end plate V2.0: 8 mm flange disc, the first 8 mm of the 16.5 mm gap"),
    ("camera_plate_arm", (-0.021, 0.049, 0.0), (0.021, 0.0999, 0.033),
     "camera-mount riser up the gripper's +y — 33 mm tall in the 2026-08-23 CAD"),
]
#: Camera MOUNT-face centre (the plate's 4-hole pattern) in the gripper base
#: frame, and the face tilt toward the fingers.  RE-VERIFIED against the
#: 2026-08-23 full-robot CAD: the new plate's 4-hole mount face sits at
#: (0, 0.07894, 0.01446) with the same 15 deg tilt — 0.3 mm from these
#: values, so they stand.  The 33 mm riser is structure AROUND the camera
#: (a protective frame), not a moved mount.  (The vendor_camera_plate.py
#: this comment used to cite never landed anywhere; the check above was
#: done by 4-hole-pattern detection on the mesh facets.)
WRIST_CAM_XYZ = (0.0, 0.079236, 0.014543)
WRIST_CAM_TILT = math.radians(15.0)
#: Placeholder module housing (x is the lens axis, box extends forward of the
#: mount face) and mass — the d1-2 unit has no nameplate; measure and replace.
WRIST_CAM_BOX = (0.016, 0.030, 0.030)
WRIST_CAM_MASS = 0.030

# --------------------------------------------------------------------------
# VENDOR BODY (urdf2026072302, exported 2026-07-23) — chassis, lift, neck.
#
# The vendor tree is  base_link -(sliderjoint, prismatic z)-> slider_Link
#                     -(rarmbase/larmbase, fixed)-> arm mounting plates
#                     -(dheadjoint, revolute)-> dhead -(upheadjoint)-> uphead.
#
# GROUND: the wheel joints sit at base_link z = -0.18517 and the wheel mesh
# radius is 0.0853, so base_link is 0.27047 above the floor.  Cross-check:
# the base_link mesh bottoms out at ground z = +0.0002.
#
# DUAL_BASE vs the vendor slider frame: the vendor arm mounting plates are at
# slider (-0.0016171, ±0.037, 0.468) with roll ∓90°, and our dual_base frame
# puts the same arm bases at (0, ±0.037, 0.50) with the same roll signs — the
# ±0.037 half-width and both roll signs agree exactly with the 2026-07-01 CAD
# measurement.  Equating them gives dual_base = slider + (-0.0016171, 0,
# -0.032), i.e. dual_base is 32 mm below the vendor slider frame.
#
# NOTE ON SIDE NAMING: vendor `rarmbase` is at y = -0.037 = the robot's
# physical RIGHT, which is our "_L" tree.  Vendor r/l is physical; the SDK's
# ArmSide::A/"_R" is the physical LEFT.  Do not cross the wires.
#
# NOT ADOPTED from the vendor file: its inertias are junk (lidar_Link is
# 0.4 kg with Ixx = 12.7 and its CoM 0.55 m off-link; both armbase plates
# carry that same copy-pasted tensor), and rarmbase/larmbase/lidar ship as
# 80-byte STLs with zero triangles.  Only kinematics + the head/neck meshes
# are used here.
# --------------------------------------------------------------------------
GROUND_TO_BASE_LINK = 0.27047       # wheel axle 0.18517 + wheel radius 0.0853
BASE_LINK_TO_SLIDER = 0.28831       # vendor sliderjoint origin, lift = 0
DUAL_BASE_IN_SLIDER = (-0.0016171, 0.0, -0.032)   # from the arm-mount match

# dual_base above the floor with the lift fully retracted.
DUAL_BASE_GROUND_Z = (GROUND_TO_BASE_LINK + BASE_LINK_TO_SLIDER
                      + DUAL_BASE_IN_SLIDER[2])            # 0.52678

# Vendor sliderjoint: axis +z, 0 … 0.300 m, effort 80 N, 0.03 m/s.  The
# 0.300 stroke independently confirms the spec sheet (整机高度 1293–1593 mm).
# q_lift = 0 is the RETRACTED rail, matching what the actuator reports —
# it is NOT the old "CAD pose = 0" convention.
LIFT_RANGE = (0.0, 0.300)
LIFT_EFFORT, LIFT_VELOCITY = 80.0, 0.03

# Neck PTU, expressed in the dual_base frame (vendor origins shifted by
# -DUAL_BASE_IN_SLIDER).  Pan `dheadjoint`: slider (0, 0, 0.589), axis -z,
# ±1.57.  Tilt `upheadjoint`: pan-frame (0, 0.0285, 0.0555) rpy (-π/2, 0, 0),
# axis +z.  The tilt SIGN is now hardware-verified (2026-08-24): rendering
# the head camera at +0.65 reproduces a real d1-2 frame taken with the neck
# fallen to its down stop, so URDF neck_tilt is the MOTOR frame — POSITIVE
# PITCHES DOWN, the same convention as omakase_neck/src/limits.py (the neck
# safety SoT) and the OPPOSITE sign of the logical pitch that
# omakase_neck/config/config.yaml exposes (logical = -motor).  Limits come
# from that SoT: motor [-0.35 (20.1 deg up, widened from the vendor -0.18
# on 2026-07-25), +0.65 (37.2 deg down, the physical contact limit)].
# BOOT HOME IS NOT ZERO: the stack homes the neck to logical +0.18
# (10.3 deg up) = URDF neck_tilt -0.18, and the DM servos ZERO THEIR
# ENCODERS WHEREVER THEY BOOT, so a raw status read is meaningless until
# the neck has been homed — map real neck state through omakaseos, not the
# raw encoder.
NECK_PAN = ((0.0016171, 0.0, 0.621), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0),
            (-1.57, 1.57))
NECK_TILT = ((0.0, 0.0285, 0.0555), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0),
             (-0.35, 0.65))
NECK_EFFORT, NECK_VELOCITY = 3.0, 3.14

# Vendor head meshes as boxes, each in its own link frame:
# dhead_Link (pan link) and uphead_Link ∪ headcamera_Link (tilt link).
NECK_PAN_BOX = ((-0.0285, -0.0255, 0.0), (0.0285, 0.0255, 0.0840))
HEAD_TILT_BOX = ((-0.0861, -0.1269, -0.1235), (0.1147, 0.0581, 0.0665))

# --------------------------------------------------------------------------
# CAMERAS — where the lens is, and WHICH AXIS IS THE LENS.
#
# A camera link says where the camera BODY sits.  It does not say which of its
# axes looks out of the lens, and getting that wrong is silent: mount a
# renderer on `yubi_<S>_hand_cam_link` with the ROS optical convention
# (optical axis = +z) and it points at the ceiling, because that link's +z is
# world up at the HOME pose.  d1-isaaclab hit exactly that, rendered the
# inside of the head shell and two ceilings, scored a believable 0/4, and had
# to CALIBRATE both camera poses inside a task config (its PR #12).  A task
# config is the wrong home for a fact about the robot, so the frames live here.
#
# Each camera therefore gets an explicit OPTICAL FRAME child in the standard
# ROS optical convention (REP 103: +z out of the lens, +x image right, +y image
# down).  Mount a renderer on `*_optical_frame` with convention="ros" and no
# rotation is needed anywhere downstream.  The rotation below is the one that
# maps a link's +x onto the optical +z; both D1 camera families point their
# lens along their link's +x, so it is the same rotation for all of them.
#
# WHERE THE DIRECTIONS COME FROM — measured off the housings, not chosen:
#
# * HEAD.  The vendor body URDF's `headcamera_Link` is NOT a lens frame:
#   `headcamerjoint` has an identity origin, so that link is only the mesh
#   sharing `uphead_Link`'s frame (= our `head_link`).  The direction is read
#   off the HOUSING instead.  In head_link coordinates the mesh is a
#   27.4 x 25.7 x 89.7 mm bar: 90 mm wide ACROSS the robot (head_link +z is
#   dual_base +y) and only 27 mm deep.  A bar like that faces along its
#   shallow axis, and the front face is the +x one — dual_base +x, straight
#   ahead and level at the parked neck pose, 1.2765 m above the floor with the
#   lift down.  `head_camera_link` is placed at the CENTRE OF THAT FRONT FACE,
#   which is SOLVED from the mesh at generation time (see :func:`head_camera`),
#   not transcribed, so a vendor mesh revision moves the frame with it.
#
# * WRISTS.  The YUBI camera housing is a 35 x 32 x 42 mm box centred at
#   (-0.0175, 0, 0) in `yubi_<S>_hand_cam_link` — it extends BACKWARDS along
#   -x, so the link origin plane already IS the lens face — and the fingers
#   reach +x (tips at x = +0.109).  The lens looks along +x, down the approach
#   axis, which is what a wrist camera is for.  So the optical frame needs no
#   translation, only the rotation.
#
# STILL UNCALIBRATED.  These are NOMINAL frames from vendor geometry, not
# calibrated extrinsics, and two things are genuinely unknown:
#   1. the lens position WITHIN the housing.  The head housing is a 90 mm
#      multi-sensor bar (an Intel D435 is 90 x 25 x 25 mm) so the colour lens
#      is offset a centimetre or two from the housing centre, and the vendor
#      CAD does not say which sensor is which.  Front-face centre is right to
#      a couple of centimetres, which matters for pixel-accurate work and not
#      for "is the camera looking at the workspace".
#   2. the 180 deg ROLL about the lens axis — which way is up in the image.
#      The axis ASSIGNMENT is measured (optical x along the housing's wide
#      transverse axis, optical y along the narrow one); the SIGN is not, and
#      the D1 stack is already known to need a 180 deg rotation on the real
#      head camera stream somewhere.  Do not trust image-space left/right
#      from these frames until they are checked against a real frame.
# Both caveats are repeated in the emitted URDF comment and in
# description/d1/README.md.  A consumer must not read these as calibrated.
#
# NOT EMITTED IN d1.urdf.  That file is the safety model: pyguard enumerates
# its links and checks keep-out volumes, and a camera frame is neither a
# volume nor a safety fact.  Cameras are emitted in the WHOLE-BODY variants
# only, which are the sim/planner assets that need them.
#
# GRIPPER WRIST CAMERA.  d1_wholebody_gripper.urdf carries one per arm since
# the arm-end camera plate CAD arrived (2026-08-21): the plate bolts into the
# flange gap and holds the wide-angle UVC module on a 15 deg face over the
# fingers.  `gripper_<S>_wrist_cam_link` keeps the family convention (lens
# along +x) and gets the standard optical frame.  See the GRIPPER
# WRIST-CAMERA PLATE block for the frames, the dataset FK cross-check, and
# what is still placeholder (the module housing and lens offset — the d1-2
# unit has no nameplate).
CAMERA_OPTICAL_RPY = (0.0, math.pi / 2.0, 0.0)   # link +x -> optical +z
#: Downward pitch of the head camera, radians.  The vendor housing mesh the
#: mount is solved from is an axis-aligned bar, so the AABB cannot carry a
#: tilt — but the 2026-08-23 full-robot CAD models the D435 body as a slab
#: whose normal is (0.955, 0.005, -0.297) in robot coordinates: the lens
#: looks 17.3 deg BELOW horizontal at the parked neck pose.  Applied about
#: head_link +z (the lateral axis; head_link +y points down).
HEAD_CAMERA_PITCH = math.atan2(0.2966, 0.955)
HEAD_CAMERA_MESH = "headcamera_Link.STL"
#: Host link of the head camera, and the vendor mesh whose front face locates
#: it.  The mesh rides `head_link` at identity (BODY_MESH_HOSTS), so the AABB
#: computed in the mesh's own coordinates is already head_link-local.
HEAD_CAMERA_HOST = "head_link"
#: The AABB that solve produces, RECORDED, in ``head_link`` metres.
#:
#: The mesh is Omakase/vendor CAD and does not ship with this repository (see
#: ``LICENSE-STATUS.md`` and :mod:`manipulation_kit.assets`), so the generator
#: has to be able to run without it — otherwise a public checkout cannot
#: rebuild its own URDFs, and "the URDFs are generated" stops being true.
#:
#: This is a CACHE OF A SOLVE, not a hand-transcribed number: when the mesh IS
#: present :func:`_mesh_aabb_in_link` recomputes it and refuses to continue if
#: the two disagree, so a vendor revision that moves the housing still fails
#: loudly instead of being papered over. Regenerate by running the build with
#: the assets fetched and copying what the mismatch message prints.
HEAD_CAMERA_AABB = (
    (0.06923796981573105, -0.08604652434587462, -0.07312534004449843),
    (0.09665606170892715, -0.060358349233865585, 0.016526428982615467),
)
#: Torso cameras, measured off the 2026-08-23 full-robot CAD via the head-
#: D435 anchor (both expressed in torso_column, which rides the lift like
#: the shells they bolt through).  PHYSICALLY PRESENT ON THE ROBOT but kept
#: unplugged/off for USB bandwidth; a simulator should enable them.
#:   * chest D435: front of the torso, lens pitched 14.1 deg UP
#:     (CAD slab normal (0.970, 0.005, 0.243)).
#:   * back fisheye: rear of the torso, level, looking straight BACK.
#: Both origins sit on the LENS FACE (the housing's outer surface along its
#: thin axis), not the housing centre — the centre is inside the torso shell
#: and a camera rendered from there sees the shell interior.
CHEST_CAMERA_XYZ = (0.0891, 0.0002, 0.5653)
CHEST_CAMERA_PITCH = -math.atan2(0.2429, 0.97)   # negative Ry = up
BACK_FISHEYE_XYZ = (-0.1093, 0.0015, 0.4822)

# --------------------------------------------------------------------------
# Body boxes, dual_base frame, MEASURED from the CAD STEP (see header).
# Each: (name, (xlo,ylo,zlo), (xhi,yhi,zhi), comment)
# Boxes named *_exempt are modeled but skipped by the pyguard keep-out check
# (the arms coexist with them by construction).
# --------------------------------------------------------------------------
BODY_BOXES = [
    ("torso_core",
     (-0.045, -0.055, 0.0), (0.045, 0.055, 0.49),
     "legacy keep-out column (safety_zones.json torso_keepout_box, chest-bracket width) - kept for continuity with the C++/JS validators"),
    ("torso_frame",
     (-0.085, -0.082, 0.385), (0.085, 0.088, 0.56),
     "robot support frame 1120100258 between the shoulders - CAD bbox X+/-85 Y+/-85 Z-115..60"),
    ("torso_speaker",
     (0.068, -0.0565, 0.4355), (0.1003, 0.0625, 0.5545),
     "M260C smart speaker, front of chest - CAD bbox X+/-59.5 Y-100.3..-68 Z-64.5..54.5"),
    ("torso_belly",
     (-0.1297, -0.1171, 0.19), (0.1271, 0.1231, 0.45),
     "_omakase body shell, belly band - CAD shell z-slices -310..-50 (X+/-120 Y-127..130)"),
    ("torso_shoulder_shell_exempt",
     (-0.126, -0.147, 0.45), (0.123, 0.153, 0.635),
     "body-shell shoulder band (z -50..135, X up to +/-150): the arm Base barrels pass through this cover, so it is EXEMPT from the guard keep-out"),
]
# Static head keep-out for d1.urdf, at the PARKED neck pose (pan = tilt = 0):
# the union of the vendor dhead / uphead / headcamera meshes.  Like-for-like
# with the 2026-07-01 CAD box it replaces — x and y agree within 13 mm — but
# 70 mm shorter (top 0.8034 vs 0.873), consistent with the head revision
# implied by the vendor export name ("去双目" = stereo pair removed).
#
# It is deliberately NOT the swept volume over the PTU range.  That box would
# be (-0.1049, -0.1415, 0.5925)…(0.1431, 0.1415, 0.8105) — 44 mm wider per
# side in y and reaching 34 mm lower — and it rejects arm poses the guard
# accepts today (folded-EE, arm-crossing, elbow-to-chest test poses all trip
# it).  Widening the keep-out is a safety-behavior change that needs its own
# review against real teleop poses; until then the guard models the parked
# head, and d1_wholebody.urdf carries the articulated neck for planners.
HEAD_BOX = ("head_shell",
            (-0.0845, -0.0950, 0.6184), (0.1163, 0.0950, 0.8034),
            "vendor dhead+uphead+headcamera mesh union at the parked neck pose (pan=tilt=0)")
# Chassis-side boxes, authored in the CAD frame where the floor sat at
# dual_base z = -0.4887, re-expressed GROUND-RELATIVE (z += 0.4887) so they
# no longer depend on the lift extension: they hang off the floor, and the
# torso rides the lift above them.
CHASSIS_BOXES_CAD = [
    ("lift_pole",
     (-0.070, -0.067, -0.100), (0.070, 0.073, 0.40),
     "AMR lift pole - CAD X+/-70 Y+/-70 Z-600..-100 (upper half overlaps the belly shell)"),
    ("chassis_cover",
     (-0.2875, -0.237, -0.213), (0.252, 0.2435, 0.010),
     "chassis top cover - CAD X+/-240 Y-252..287.5 Z-713..-490"),
    ("chassis_body",
     (-0.3273, -0.2455, -0.4887), (0.3181, 0.2515, -0.1055),
     "drive base incl. wheels - CAD X+/-248.5 Y-318.1..327.3 Z-988.7..-605.5; ground plane at z=-0.4887"),
]
CAD_GROUND_IN_DUAL_BASE = -0.4887   # floor, in the frame CHASSIS_BOXES_CAD use
# Same boxes with the floor at z = 0.  Cross-checked against the vendor
# base_link mesh (ground z 0.0002 … 0.8465, x -0.294…0.303, y ±0.240): the
# CAD boxes stay outside it by ≤ 42 mm, i.e. conservative, as a keep-out
# volume should be.
CHASSIS_BOXES = [
    (name,
     (lo[0], lo[1], lo[2] - CAD_GROUND_IN_DUAL_BASE),
     (hi[0], hi[1], hi[2] - CAD_GROUND_IN_DUAL_BASE),
     comment)
    for name, lo, hi, comment in CHASSIS_BOXES_CAD
]


def _fmt(v):
    return ("%.6g" % v)


def _xyz(t):
    return " ".join(_fmt(v) for v in t)


def box_elem(name, lo, hi, comment, indent="    "):
    size = tuple(hi[i] - lo[i] for i in range(3))
    ctr = tuple((hi[i] + lo[i]) / 2.0 for i in range(3))
    return (f"{indent}<!-- {comment} -->\n"
            f"{indent}<collision name=\"{name}\">\n"
            f"{indent}  <origin xyz=\"{_xyz(ctr)}\" rpy=\"0 0 0\"/>\n"
            f"{indent}  <geometry><box size=\"{_xyz(size)}\"/></geometry>\n"
            f"{indent}</collision>\n")


def seg_collision(link, side, indent="    "):
    """Capsule primitive covering the link's joint-to-joint segment:
    cylinder (axis z rotated onto the segment) or sphere when degenerate."""
    seg = LINK_SEGMENT[link]
    r = CAP_RADII[link]
    L = math.sqrt(sum(c * c for c in seg))
    name = f"{link}_{side}_capsule"
    if L < 1e-9:
        return (f"{indent}<collision name=\"{name}\">\n"
                f"{indent}  <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
                f"{indent}  <geometry><sphere radius=\"{_fmt(r)}\"/></geometry>\n"
                f"{indent}</collision>\n")
    ux, uy, uz = (seg[0] / L, seg[1] / L, seg[2] / L)
    # rpy (roll about x then pitch about y, yaw 0) mapping local +z onto u:
    # R = Rz(0)*Ry(p)*Rx(rll); z' = (sin p * cos rll, -sin rll, cos p cos rll)
    rll = math.atan2(-uy, math.sqrt(ux * ux + uz * uz))
    p = math.atan2(ux, uz)
    mid = (seg[0] / 2.0, seg[1] / 2.0, seg[2] / 2.0)
    out = (f"{indent}<collision name=\"{name}\">\n"
           f"{indent}  <origin xyz=\"{_xyz(mid)}\" rpy=\"{_fmt(rll)} {_fmt(p)} 0\"/>\n"
           f"{indent}  <geometry><cylinder radius=\"{_fmt(r)}\" length=\"{_fmt(L)}\"/></geometry>\n"
           f"{indent}</collision>\n")
    # sphere caps so the primitive union is a true capsule
    for tag, pos in (("a", (0.0, 0.0, 0.0)), ("b", seg)):
        out += (f"{indent}<collision name=\"{name}_cap_{tag}\">\n"
                f"{indent}  <origin xyz=\"{_xyz(pos)}\" rpy=\"0 0 0\"/>\n"
                f"{indent}  <geometry><sphere radius=\"{_fmt(r)}\"/></geometry>\n"
                f"{indent}</collision>\n")
    return out


# ---------------------------------------------------------------- 4x4 rigid
def _mat(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0]]


def _mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _minv(a):
    """Inverse of a rigid transform (R^T, -R^T t)."""
    out = [[a[j][i] for j in range(3)] + [0.0] for i in range(3)]
    out.append([0.0, 0.0, 0.0, 1.0])
    for i in range(3):
        out[i][3] = -sum(a[k][i] * a[k][3] for k in range(3))
    return out


def _to_rpy(m):
    """rpy such that _mat(rpy=...) reproduces m's rotation."""
    sp = max(-1.0, min(1.0, -m[2][0]))
    p = math.asin(sp)
    if abs(math.cos(p)) < 1e-9:                      # gimbal lock
        return (math.atan2(m[0][1], m[1][1]), p, 0.0)
    return (math.atan2(m[2][1], m[2][2]), p, math.atan2(m[1][0], m[0][0]))


def box_visual(xyz, size, meshes, rgba=ARM_GREY, material="d1_primitive",
               indent="    "):
    """A ``<visual>`` box mirroring a collision primitive.

    For the few parts that have NO CAD mesh anywhere in this repo — the YUBI
    palm shell and its wrist camera, which the vendor xacro also models as
    boxes. Better a box that is visibly a box than a hole in the robot.
    """
    if not meshes:
        return ""
    return (f"{indent}<visual>\n"
            f"{indent}  <origin xyz=\"{xyz}\" rpy=\"0 0 0\"/>\n"
            f"{indent}  <geometry><box size=\"{size}\"/></geometry>\n"
            f"{indent}  <material name=\"{material}\">"
            f"<color rgba=\"{rgba}\"/></material>\n"
            f"{indent}</visual>\n")


def mesh_visual_elem(filename, m, rgba, material, scale=None, indent="    "):
    """``<visual>`` with a mesh, placed by the 4x4 ``m`` in its link's frame."""
    xyz = tuple(m[i][3] for i in range(3))
    s = (f"{indent}<visual>\n"
         f"{indent}  <origin xyz=\"{_xyz(xyz)}\" rpy=\"{_xyz(_to_rpy(m))}\"/>\n"
         f"{indent}  <geometry><mesh filename=\"{filename}\"")
    if scale:
        s += f" scale=\"{scale}\""
    return (s + "/></geometry>\n"
            f"{indent}  <material name=\"{material}\">"
            f"<color rgba=\"{rgba}\"/></material>\n"
            f"{indent}</visual>\n")


# ------------------------------------------------------- vendor body solve
_BODY_CACHE = {}


def vendor_body():
    """{link: (T_in_dual_base, [(mesh_file, T_mesh_local, scale)])}.

    FK over the vendor ``merged_robot`` tree at all-zero joints, re-expressed
    in OUR ``dual_base`` frame using the slider equality this file already
    relies on:  p_dual_base = p_base_link - slider_origin - DUAL_BASE_IN_SLIDER.
    Validated against the vendor arm mounting plates before anything uses it.
    """
    if _BODY_CACHE:
        return _BODY_CACHE
    root = ET.parse(BODY_URDF).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = tuple(float(v) for v in (o.get("xyz") or "0 0 0").split())
        rpy = tuple(float(v) for v in (o.get("rpy") or "0 0 0").split())
        joints[j.find("child").get("link")] = (j.find("parent").get("link"),
                                               _mat(xyz, rpy))
    slider_parent, slider_origin = joints["slider_Link"]
    assert slider_parent == "base_link", slider_parent
    # base_link -> dual_base
    to_db = _minv(_mmul(slider_origin, _mat(DUAL_BASE_IN_SLIDER)))

    def fk(link):
        m = _mat()
        while link in joints:
            parent, origin = joints[link]
            m = _mmul(origin, m)
            link = parent
        return _mmul(to_db, m)

    out = {}
    for link in root.findall("link"):
        name = link.get("name")
        meshes = []
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            o = vis.find("origin")
            xyz = tuple(float(v) for v in
                        ((o.get("xyz") if o is not None else None) or "0 0 0").split())
            rpy = tuple(float(v) for v in
                        ((o.get("rpy") if o is not None else None) or "0 0 0").split())
            stl = (mesh.get("filename") or "").rsplit("/", 1)[-1]
            meshes.append((stl, _mat(xyz, rpy), mesh.get("scale")))
        out[name] = (fk(name), meshes)

    # GATE: the vendor arm plates must land on our arm mounts, or the frame
    # mapping is wrong and every body mesh would be pasted on askew.
    for plate, (side, want) in ARM_PLATE_CHECK.items():
        got = tuple(out[plate][0][i][3] for i in range(3))
        err = max(abs(got[i] - want[i]) for i in range(3))
        if err > ARM_PLATE_TOLERANCE_M:
            raise SystemExit(
                f"body CAD alignment failed: vendor {plate} lands at {got} in "
                f"dual_base, expected our {side}-arm mount at {want} "
                f"({err * 1000:.1f} mm off). The frame mapping is wrong; do not "
                "ship these meshes.")
    _BODY_CACHE.update(out)
    return _BODY_CACHE


#: High-fidelity body visuals, one merged mesh per host link, split out of the
#: 2026-08-23 full-robot CAD (Drive/D1/CAD/d1_20260823.step) and expressed in
#: LINK-LOCAL coordinates, so they mount at identity.  They REPLACE the coarse
#: vendor visuals on these hosts: the vendor set models the AMR as a tall box
#: and the torso as a faceted vest, which is what made every sim capture look
#: wrong.  Fasteners and internal hardware were dropped (visual-only meshes)
#:  and each mesh decimated to a fixed budget.  Collision geometry is
#: untouched — the safety boxes still come from the vendor-measured AABBs.
#: The camera-mount solve also still reads the VENDOR meshes (files stay in
#: meshes/body/), so replacing a visual cannot move a camera frame.
#: Static/moving split of the lift column follows the CAD part names:
#: 升降柱外壳 (outer shells) are chassis-static, the vest rides the lift.
#: OBJ, not STL, deliberately: STL carries no vertex normals, so every
#: renderer flat-shades it and a decimated curved shell looks faceted — the
#: exact "low fidelity" complaint this replaces.  OBJ carries the normals.
BODY_HIFI = {
    "chassis_link": "chassis_link_hifi.obj",
    "torso_column": "torso_column_hifi.obj",
    "neck_pan_link": "neck_pan_link_hifi.obj",
    "head_link": "head_link_hifi.obj",
}
BODY_HIFI_DIR = "meshes/body_hifi"


def body_mesh_visuals(host, host_in_dual_base, indent="    "):
    """Visual elements for ``host``: the CAD-split high-fidelity mesh when one
    exists (link-local, identity mount), else the vendor mesh set."""
    hifi = BODY_HIFI.get(host)
    if hifi is not None:
        # Each visual references one clipped material region of the body mesh,
        # allowing standard URDF materials to reproduce the d1.avif palette.
        import json
        palette_path = os.path.join(DESC_DIR, BODY_HIFI_DIR, "color_regions.json")
        with open(palette_path) as palette_file:
            palette = json.load(palette_file)
        return "".join(
            mesh_visual_elem(f"{BODY_HIFI_DIR}/{host}_{color}.obj", _mat(),
                             palette["palette"][color], f"d1_{color}", None, indent)
            for color in palette["hosts"][host]["faces"]
        )
    names = dict(BODY_MESH_HOSTS).get(host, ())
    if not names:
        return ""
    body = vendor_body()
    inv_host = _minv(host_in_dual_base)
    out = ""
    for name in names:
        transform, meshes = body[name]
        for stl, local, scale in meshes:
            m = _mmul(_mmul(inv_host, transform), local)
            out += mesh_visual_elem(f"{BODY_MESH_DIR}/{stl}", m, BODY_GREY,
                                    "d1_body_shell", scale, indent)
    return out


# ----------------------------------------------------------- camera frames
#: Solves this generator can perform only with the external CAD present, keyed
#: by (host link, mesh file).  See :data:`HEAD_CAMERA_AABB`.
RECORDED_AABBS = {(HEAD_CAMERA_HOST, HEAD_CAMERA_MESH): HEAD_CAMERA_AABB}


def _external_mesh(desc_rel):
    """An external mesh under ``description/``, or ``None`` if absent.

    ``desc_rel`` is relative to this description tree, e.g.
    ``"meshes/body/headcamera_Link.STL"``.
    """
    from manipulation_kit import assets  # noqa: PLC0415  (runs as a script)

    return assets.resolve(f"description/d1/{desc_rel}")


def _recorded_aabb(host, mesh_file):
    try:
        return RECORDED_AABBS[(host, mesh_file)]
    except KeyError:
        raise SystemExit(
            f"{mesh_file} is not in this repository and no solve is recorded "
            f"for it on {host}. Fetch the CAD (`mkit-urdf fetch-assets`) or "
            "add an entry to RECORDED_AABBS.") from None


def _checked_aabb(host, mesh_file, solved):
    """The freshly solved AABB, after proving the recorded one still matches."""
    recorded = RECORDED_AABBS.get((host, mesh_file))
    if recorded is None:
        return solved
    if max(abs(a - b) for r, s in zip(recorded, solved)
           for a, b in zip(r, s)) > 1e-9:
        raise SystemExit(
            f"{mesh_file} no longer solves to the recorded AABB on {host}.\n"
            f"  recorded: {recorded}\n  from the mesh: {solved}\n"
            "The CAD changed. Update RECORDED_AABBS with the second line and "
            "expect the camera frames to move.")
    return solved


def _stl_points(path):
    """Every vertex of a binary or ASCII STL, in the mesh's own coordinates.

    Binary is discriminated by SIZE (84 + 50 * triangles), not by the "solid"
    magic: SolidWorks writes binary STLs whose 80-byte header happens to start
    with "solid", which is exactly how an ASCII sniff test gets this wrong.
    """
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        count = struct.unpack("<I", f.read(84)[80:84])[0]
        if size == 84 + 50 * count:
            out = []
            for _ in range(count):
                # 12 floats: normal, then the three vertices.
                v = struct.unpack("<12f", f.read(50)[:48])
                out.append(v[3:6])
                out.append(v[6:9])
                out.append(v[9:12])
            return out
    with open(path) as f:                      # ASCII fallback
        return [tuple(float(x) for x in parts[1:4])
                for parts in (line.split() for line in f)
                if parts and parts[0] == "vertex"]


def _host_in_dual_base(host):
    """Transform of a body-mesh host link in the ``dual_base`` frame.

    Only the links :data:`BODY_MESH_HOSTS` names, and only the whole-body
    variants' articulated-neck layout (the guard model has no camera frames —
    see the CAMERAS block).
    """
    pan = _mat(NECK_PAN[0], NECK_PAN[1])
    if host == "torso_column":
        return _mat()
    if host == "neck_pan_link":
        return pan
    if host == "head_link":
        return _mmul(pan, _mat(NECK_TILT[0], NECK_TILT[1]))
    raise SystemExit(f"no dual_base transform known for host link {host!r}")


def _mesh_aabb_in_link(host, mesh_file):
    """AABB of a vendor body mesh, expressed in its HOST LINK's frame.

    Solved through the same chain :func:`body_mesh_visuals` uses, so the box
    cannot disagree with where the mesh is actually drawn.  Every vertex is
    transformed (not the eight AABB corners), so the result stays exact if a
    vendor revision rotates the mesh.

    The mesh is external CAD (:mod:`manipulation_kit.assets`).  Absent, the
    recorded :data:`HEAD_CAMERA_AABB` answers; present, it is recomputed and
    cross-checked, so the record can never silently outlive its mesh.
    """
    body = vendor_body()
    inv_host = _minv(_host_in_dual_base(host))
    for link, names in BODY_MESH_HOSTS:
        if link != host:
            continue
        for name in names:
            transform, meshes = body[name]
            for stl, local, _scale in meshes:
                if stl != mesh_file:
                    continue
                m = _mmul(_mmul(inv_host, transform), local)
                path = _external_mesh(f"{BODY_MESH_DIR}/{stl}")
                if path is None:
                    return _recorded_aabb(host, mesh_file)
                pts = _stl_points(path)
                lo = [float("inf")] * 3
                hi = [float("-inf")] * 3
                for p in pts:
                    for i in range(3):
                        v = (m[i][0] * p[0] + m[i][1] * p[1]
                             + m[i][2] * p[2] + m[i][3])
                        lo[i] = min(lo[i], v)
                        hi[i] = max(hi[i], v)
                return _checked_aabb(host, mesh_file,
                                     (tuple(lo), tuple(hi)))
    raise SystemExit(f"{mesh_file} is not hosted on {host}: check "
                     "BODY_MESH_HOSTS")


def frame_link(name, indent="  "):
    """A pure FRAME link: no geometry, and the smallest mass that keeps a
    physics importer from inventing one.

    URDF says a link with no ``<inertial>`` is massless, and leaving it out
    reads as the honest thing to do — but it is not what happens. MEASURED:
    imported into Isaac Sim with ``merge_fixed_joints=False`` (which a consumer
    needs, or the frame is collapsed away and there is nothing to mount a
    camera on), PhysX gave each inertial-less frame a DEFAULT mass. Each hand
    came out at 2.5 kg instead of its measured 1.5, and the robot gained 4.8 kg
    — silently, because nothing in the URDF says 1.0 kg anywhere.

    So the mass is stated explicitly and small enough to be nothing: 1e-6 kg,
    four frames, 4 micrograms on a 150 kg robot. This is the value being pinned
    by test_camera_frames_weigh_effectively_nothing; do not "clean it up" by
    removing the inertial, that is the bug.
    """
    return (f'{indent}<link name="{name}">\n'
            f'{indent}  <!-- a frame, not a body: 1e-6 kg exists only so a\n'
            f'{indent}       physics importer does not substitute a default -->\n'
            f'{indent}  <inertial>\n'
            f'{indent}    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
            f'{indent}    <mass value="1e-06"/>\n'
            f'{indent}    <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09"'
            f' iyz="0" izz="1e-09"/>\n'
            f'{indent}  </inertial>\n'
            f'{indent}</link>\n')


def optical_frame(name, parent, indent="  "):
    """Fixed joint + frame link giving ``parent`` a ROS optical frame.

    Note for importers: a URDF importer that merges fixed joints (Isaac Lab's
    does by default) collapses a geometry-less link like this away, so a
    consumer that needs the prim must keep fixed joints — and see
    :func:`frame_link` for what that then costs in mass if the link is left
    inertial-less.
    """
    return (f'{indent}<joint name="{name}_joint" type="fixed">\n'
            f'{indent}  <origin xyz="0 0 0" rpy="{_xyz(CAMERA_OPTICAL_RPY)}"/>\n'
            f'{indent}  <parent link="{parent}"/>\n'
            f'{indent}  <child link="{name}"/>\n'
            f'{indent}</joint>\n'
            + frame_link(name, indent))


def head_camera():
    """`head_camera_link` (+ its optical frame) on the head tilt link.

    Placed at the CENTRE OF THE FRONT FACE of the vendor camera housing,
    solved from the mesh — see the CAMERAS block for why the front face is the
    +x one and for what is still uncalibrated.
    """
    lo, hi = _mesh_aabb_in_link(HEAD_CAMERA_HOST, HEAD_CAMERA_MESH)
    xyz = (hi[0], (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)
    size = " x ".join("%.1f" % ((hi[i] - lo[i]) * 1000) for i in range(3))
    return ("\n  <!-- Head camera. NOMINAL, NOT A CALIBRATED EXTRINSIC: it is\n"
            "       the centre of the FRONT FACE of the vendor\n"
            f"       {HEAD_CAMERA_MESH} housing ({size} mm in this\n"
            "       link's axes), solved from the mesh rather than transcribed.\n"
            "       So the lens offset WITHIN that multi-sensor bar (order 1 cm)\n"
            "       is not modelled, and the 180 deg roll about the lens axis is\n"
            "       UNVERIFIED. The lens looks along head_camera_link +x;\n"
            "       head_camera_optical_frame is the ROS optical convention\n"
            "       (+z out of the lens, +x image right, +y image down) so a\n"
            "       renderer mounted there needs no extra rotation. At the\n"
            "       parked neck pose it looks straight ahead pitched\n"
            f"       {math.degrees(HEAD_CAMERA_PITCH):.1f} deg BELOW horizontal — the mount tilt measured\n"
            "       off the D435 slab in the 2026-08-23 full-robot CAD, which\n"
            "       the axis-aligned housing mesh cannot carry. A close\n"
            "       tabletop still needs neck_tilt on top of this.\n"
            "       See the CAMERAS block in the generator and\n"
            "       description/d1/README.md. -->\n"
            '  <joint name="head_camera_mount" type="fixed">\n'
            f'    <origin xyz="{_xyz(xyz)}" rpy="0 0 {_fmt(HEAD_CAMERA_PITCH)}"/>\n'
            f'    <parent link="{HEAD_CAMERA_HOST}"/>\n'
            '    <child link="head_camera_link"/>\n'
            '  </joint>\n'
            + frame_link("head_camera_link")
            + optical_frame("head_camera_optical_frame", "head_camera_link"))


def inertial(mass=0.5):
    return ("    <inertial>\n"
            "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            f"      <mass value=\"{_fmt(mass)}\"/>\n"
            "      <inertia ixx=\"1e-3\" ixy=\"0\" ixz=\"0\" iyy=\"1e-3\" iyz=\"0\" izz=\"1e-3\"/>\n"
            "    </inertial>\n")


def arm_mesh_visual(link, side, meshes):
    """The vendor arm STL for this link, at IDENTITY in the link's own frame.

    No transform is needed and none is guessed: this file's J1..J7+TCP chain is
    byte-derived from the vendor D1 arm URDFs, and
    test_arm_chain_identical_across_all_models_and_sides asserts every origin
    and axis still matches them. Identical chains mean identical link frames,
    so the vendor mesh drops straight in.
    """
    if not meshes:
        return ""
    return mesh_visual_elem(f"{ARM_MESH_DIR[side]}/{link}_{side}.STL",
                            _mat(), ARM_GREY, "d1_arm_shell")


def arm(side, end_effector, meshes=False, cameras=False):
    s = f"\n  <!-- ===== Arm {side} "
    s += ("(SDK ArmSide::A, physical LEFT, +y)" if side == "R"
          else "(SDK ArmSide::B, physical RIGHT, -y)")
    s += " — D1 arm chain ===== -->\n"
    (mx, mr) = MOUNTS[side]
    s += (f"  <joint name=\"mount_{side}\" type=\"fixed\">\n"
          f"    <origin xyz=\"{_xyz(mx)}\" rpy=\"{_xyz(mr)}\"/>\n"
          f"    <parent link=\"torso_column\"/>\n"
          f"    <child link=\"Base_{side}\"/>\n"
          f"  </joint>\n")
    links = ["Base"] + [f"Link{i}" for i in range(1, 8)]
    for ln in links:
        s += (f"  <link name=\"{ln}_{side}\">\n" + inertial()
              + arm_mesh_visual(ln, side, meshes)
              + seg_collision(ln, side) + "  </link>\n")
    for i, (jn, rpy, xyz, lo, hi, eff) in enumerate(ARM_CHAIN):
        parent = f"{links[i]}_{side}"
        child = f"{links[i + 1]}_{side}"
        s += (f"  <joint name=\"{jn}_{side}\" type=\"revolute\">\n"
              f"    <origin xyz=\"{_xyz(xyz)}\" rpy=\"{_xyz(rpy)}\"/>\n"
              f"    <parent link=\"{parent}\"/>\n"
              f"    <child link=\"{child}\"/>\n"
              f"    <axis xyz=\"0 0 1\"/>\n"
              f"    <limit lower=\"{_fmt(lo)}\" upper=\"{_fmt(hi)}\" effort=\"{_fmt(eff)}\" velocity=\"3.1416\"/>\n"
              f"  </joint>\n")
    # TCP flange
    s += (f"  <link name=\"TCP_Link_{side}\">\n" + inertial(0.05)
          + arm_mesh_visual("TCP_Link", side, meshes)
          + seg_collision("TCP_Link", side) + "  </link>\n")
    s += (f"  <joint name=\"JointTCP_{side}\" type=\"fixed\">\n"
          f"    <origin xyz=\"{_xyz(TCP_XYZ)}\" rpy=\"{_xyz(TCP_RPY)}\"/>\n"
          f"    <parent link=\"Link7_{side}\"/>\n"
          f"    <child link=\"TCP_Link_{side}\"/>\n"
          f"  </joint>\n")
    return s + END_EFFECTORS[end_effector](side, meshes, cameras)


def yubi(side, meshes=False, cameras=False):
    (fx, fr) = YUBI_FLANGE[side]
    p = f"yubi_{side}"
    s = f"\n  <!-- YUBI hand on arm {side} (mount from d1-manip-sim d1_yubi.urdf) -->\n"
    s += (f"  <joint name=\"{p}_flange\" type=\"fixed\">\n"
          f"    <origin xyz=\"{_xyz(fx)}\" rpy=\"{_xyz(fr)}\"/>\n"
          f"    <parent link=\"TCP_Link_{side}\"/>\n"
          f"    <child link=\"{p}_hand_root\"/>\n"
          f"  </joint>\n")
    s += (f"  <link name=\"{p}_hand_root\">\n" + inertial(0.26)
          + box_visual("-0.022 0 -0.02015", "0.044 0.067 0.0735", meshes)
          + f"    <collision name=\"{p}_palm\">\n"
            f"      <origin xyz=\"-0.022 0 -0.02015\" rpy=\"0 0 0\"/>\n"
            f"      <geometry><box size=\"0.044 0.067 0.0735\"/></geometry>\n"
            f"    </collision>\n"
          + "  </link>\n")
    s += (f"  <joint name=\"{p}_camera_joint\" type=\"fixed\">\n"
          f"    <origin xyz=\"0 0 0.0426\" rpy=\"0 0 0\"/>\n"
          f"    <parent link=\"{p}_hand_root\"/>\n"
          f"    <child link=\"{p}_hand_cam_link\"/>\n"
          f"  </joint>\n")
    s += (f"  <link name=\"{p}_hand_cam_link\">\n" + inertial(0.03)
          + box_visual("-0.0175 0 0", "0.035 0.032 0.042", meshes)
          + f"    <collision name=\"{p}_cam\">\n"
            f"      <origin xyz=\"-0.0175 0 0\" rpy=\"0 0 0\"/>\n"
            f"      <geometry><box size=\"0.035 0.032 0.042\"/></geometry>\n"
            f"    </collision>\n"
          + "  </link>\n")
    if cameras:
        # The 35 x 32 x 42 mm housing box is centred at (-0.0175, 0, 0), i.e. it
        # extends BACKWARDS along -x and the link origin plane IS the lens face,
        # and the fingers reach +x (tips at x = +0.109). So the lens looks along
        # +x, down the approach axis, and the optical frame needs no offset.
        # NOMINAL, not a calibrated extrinsic — see the CAMERAS block.
        s += (f"  <!-- Wrist camera optical frame: lens along {p}_hand_cam_link\n"
              "       +x (the approach axis), ROS optical convention. The 180 deg\n"
              "       roll about the lens axis is UNVERIFIED. -->\n"
              + optical_frame(f"{p}_hand_cam_optical_frame",
                              f"{p}_hand_cam_link"))
    for fname, sign, (lo, hi) in (("right", -1.0, (0.0, 0.94)),
                                  ("left", 1.0, (-0.94, 0.0))):
        blo, bhi = FINGER_BOX[fname]
        s += (f"  <joint name=\"{p}_{fname}_finger_joint\" type=\"revolute\">\n"
              f"    <origin xyz=\"-0.016 {_fmt(sign * 0.015)} 0\" rpy=\"0 0 0\"/>\n"
              f"    <parent link=\"{p}_hand_root\"/>\n"
              f"    <child link=\"{p}_{fname}_finger_link\"/>\n"
              f"    <axis xyz=\"0 0 -1\"/>\n"
              f"    <limit lower=\"{_fmt(lo)}\" upper=\"{_fmt(hi)}\" effort=\"5\" velocity=\"2\"/>\n"
              f"  </joint>\n")
        hand = "right" if side == "R" else "left"
        s += (f"  <link name=\"{p}_{fname}_finger_link\">\n" + inertial(0.04)
              + (mesh_visual_elem(
                  f"{YUBI_MESH_DIR}/hand_{hand}_{fname}_finger_link.STL",
                  _mat(), ARM_GREY, "d1_yubi_shell") if meshes else "")
              + box_elem(f"{p}_{fname}_finger", blo, bhi,
                         "bounding box of yubi_description finger collision STL")
              + "  </link>\n")
    return s


def full_inertial(mass, com, inertia, note, indent="    "):
    """Inertial with a real COM and tensor (as opposed to :func:`inertial`,
    which emits the family's 1e-3 diagonal placeholder).

    Mass and COM print at 9 significant figures, not the file's usual 6: the
    per-side masses have to SUM to the measured 1.5 kg, and %.6g on
    1.416851 kg loses 2 mg per gripper.
    """
    ixx, ixy, ixz, iyy, iyz, izz = inertia
    return (f"{indent}<!-- {note} -->\n"
            f"{indent}<inertial>\n"
            f"{indent}  <origin xyz=\"{' '.join('%.9g' % v for v in com)}\""
            f" rpy=\"0 0 0\"/>\n"
            f"{indent}  <mass value=\"{'%.9g' % mass}\"/>\n"
            f"{indent}  <inertia ixx=\"{_fmt(ixx)}\" ixy=\"{_fmt(ixy)}\""
            f" ixz=\"{_fmt(ixz)}\" iyy=\"{_fmt(iyy)}\" iyz=\"{_fmt(iyz)}\""
            f" izz=\"{_fmt(izz)}\"/>\n"
            f"{indent}</inertial>\n")


def mesh_visual(name, kind, indent="    "):
    """``<visual>`` referencing a vendored gripper STL, path relative to the
    URDF file so it resolves outside ROS and from any working directory.
    ``kind`` picks the vendor material: "base" (housing) or "jaw"."""
    material, rgba = GRIPPER_COLOUR[kind]
    return (f"{indent}<visual>\n"
            f"{indent}  <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            f"{indent}  <geometry><mesh filename=\"{GRIPPER_MESH_DIR}/{name}\"/></geometry>\n"
            f"{indent}  <material name=\"{material}\">"
            f"<color rgba=\"{rgba}\"/></material>\n"
            f"{indent}</visual>\n")


def parallel_gripper(side, meshes=False, cameras=False):
    """D1 stock parallel gripper on arm ``side``.

    Collision is primitives like the rest of this family; VISUAL is the real
    CAD mesh, because this variant is the asset other repositories consume.

    Geometry provenance: the vendor CAD in dx-manipulator
    ``hands/d1/parallel_gripper/descriptions/`` — see the GRIPPER block above
    for the mount frame, the handedness argument, the mass correction and every
    transcribed number.

    ``cameras`` adds the wrist camera the arm-end plate carries: the plate
    link itself is unconditional (it is bolted hardware either way), the
    ``gripper_<S>_wrist_cam_link`` + optical frame ride on it when asked.
    See the GRIPPER WRIST-CAMERA PLATE block for provenance and caveats.
    """
    (fx, fr) = GRIPPER_FLANGE[side]
    p = f"gripper_{side}"
    up = "+y up on both arms; _R takes the half turn" if side == "R" else "+y already up"
    s = (f"\n  <!-- D1 stock parallel gripper on arm {side} — vendor CAD via\n"
         f"       dx-manipulator hands/d1/parallel_gripper. base_link IS the tool\n"
         f"       flange, so the mount is TCP_Link with no translation ({up}).\n"
         f"       Mass 1.5 kg MEASURED (Shu 2026-07-29); the CAD's own 0.3279 kg\n"
         f"       was a shell-only export. Per-link split is an estimate, the\n"
         f"       total and the assembly COM (68 mm) are not. -->\n")
    s += (f"  <joint name=\"{p}_flange\" type=\"fixed\">\n"
          f"    <origin xyz=\"{_xyz(fx)}\" rpy=\"{_xyz(fr)}\"/>\n"
          f"    <parent link=\"TCP_Link_{side}\"/>\n"
          f"    <child link=\"{p}_base_link\"/>\n"
          f"  </joint>\n")
    body = (f"  <link name=\"{p}_base_link\">\n"
            + full_inertial(GRIPPER_BODY_MASS, GRIPPER_BODY_COM,
                            GRIPPER_BODY_INERTIA,
                            "measured 1.5 kg assembly minus the two CAD jaws; COM"
                            " solved so the assembly COM is the validated 68 mm;"
                            " tensor = CAD tensor x mass ratio (ESTIMATE)")
            + (mesh_visual(GRIPPER_MESH["base"], "base") if meshes else "")
            + (box_visual(_xyz([(GRIPPER_ADAPTER_VISUAL[0][i]
                                + GRIPPER_ADAPTER_VISUAL[1][i]) / 2
                               for i in range(3)]),
                          _xyz([GRIPPER_ADAPTER_VISUAL[1][i]
                                - GRIPPER_ADAPTER_VISUAL[0][i]
                                for i in range(3)]),
                          meshes, GRIPPER_ADAPTER_COLOUR[1],
                          GRIPPER_ADAPTER_COLOUR[0])))
    for name, lo, hi, comment in GRIPPER_BODY_BOXES:
        body += box_elem(f"{p}_{name}", lo, hi, comment)
    s += body + "  </link>\n"
    (jx, jr) = GRIPPER_JAW_ORIGIN
    for jaw in ("r", "l"):
        lo, hi = GRIPPER_JAW_LIMITS[jaw]
        blo, bhi = GRIPPER_JAW_BOX[jaw]
        s += (f"  <joint name=\"{p}_tcp_{jaw}_joint\" type=\"prismatic\">\n"
              f"    <origin xyz=\"{_xyz(jx)}\" rpy=\"{_xyz(jr)}\"/>\n"
              f"    <parent link=\"{p}_base_link\"/>\n"
              f"    <child link=\"{p}_tcp_{jaw}_link\"/>\n"
              f"    <axis xyz=\"{_xyz(GRIPPER_JAW_AXIS)}\"/>\n"
              f"    <limit lower=\"{_fmt(lo)}\" upper=\"{_fmt(hi)}\""
              f" effort=\"{_fmt(GRIPPER_JAW_EFFORT)}\""
              f" velocity=\"{_fmt(GRIPPER_JAW_VELOCITY)}\"/>\n"
              f"  </joint>\n")
        s += (f"  <link name=\"{p}_tcp_{jaw}_link\">\n"
              + full_inertial(GRIPPER_JAW_MASS, GRIPPER_JAW_COM[jaw],
                              GRIPPER_JAW_INERTIA[jaw],
                              "CAD inertial, verbatim (41.6 g, 5.5% of the tool)")
              + (mesh_visual(GRIPPER_MESH[jaw], "jaw") if meshes else "")
              + box_elem(f"{p}_tcp_{jaw}_jaw", blo, bhi,
                         f"AABB of the CAD tcp_{jaw}_Link mesh; q=0 is OPEN")
              + "  </link>\n")
    return s + gripper_camera_plate(side, meshes, cameras)


def gripper_camera_plate(side, meshes=False, cameras=False):
    """The arm-end camera plate on the gripper base, and (``cameras``) the
    wrist camera it carries.

    The plate's CAD origin is the flange centre, so it mounts at IDENTITY on
    ``gripper_<S>_base_link`` — GRIPPER_FLANGE has already put the gripper's
    +y "up" on both arms, and the plate's camera arm rises along that +y.

    ``gripper_<S>_wrist_cam_link`` follows the family convention (lens along
    the link's +x): its +x is the plate's mount-face normal (the flange +z
    pitched WRIST_CAM_TILT toward the fingers) and its +y is "image down",
    pointing toward the fingers — the real wrist streams show the jaws at the
    bottom edge of the frame.  The rpy below is solved from those axes at
    generation time rather than transcribed.
    """
    p = f"gripper_{side}"
    s = (f"\n  <!-- Arm-end camera plate V2.0 on gripper {side} — the first REAL\n"
         f"       geometry in the 16.5 mm flange gap (its remaining 8.5 mm is the\n"
         f"       still-ASSUMED gripper-end plate).  105 g at aluminium book\n"
         f"       density, tensor exact from the mesh.  See the GRIPPER\n"
         f"       WRIST-CAMERA PLATE block in the generator. -->\n")
    s += (f"  <joint name=\"{p}_camera_plate_joint\" type=\"fixed\">\n"
          f"    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
          f"    <parent link=\"{p}_base_link\"/>\n"
          f"    <child link=\"{p}_camera_plate\"/>\n"
          f"  </joint>\n")
    s += (f"  <link name=\"{p}_camera_plate\">\n"
          + full_inertial(GRIPPER_PLATE_MASS, GRIPPER_PLATE_COM,
                          GRIPPER_PLATE_INERTIA,
                          "mesh volume 28.95 cm^3 x aluminium 2700 kg/m^3;"
                          " tensor exact from the mesh; density ASSUMED, part"
                          " not yet weighed")
          + (mesh_visual(GRIPPER_PLATE_MESH, "plate") if meshes else ""))
    for name, lo, hi, comment in GRIPPER_PLATE_BOXES:
        s += box_elem(f"{p}_{name}", lo, hi, comment)
    s += "  </link>\n"
    if not cameras:
        return s

    # wrist_cam_link axes in the plate/gripper-base frame: +x = mount-face
    # normal (flange +z pitched toward the fingers), +y = image down (toward
    # the fingers), +z completes right-handed (= gripper +x, so optical
    # x = image right lands opposite the jaw-travel +x).
    m = _mmul(_mat(rpy=(WRIST_CAM_TILT, 0.0, 0.0)),
              _mat(rpy=(0.0, -math.pi / 2.0, math.pi)))
    cam_rpy = _to_rpy(m)
    s += (f"\n  <!-- Wrist camera on the plate's mount face (the 4-hole pattern\n"
          f"       centre).  NOMINAL, NOT A CALIBRATED EXTRINSIC: the module on\n"
          f"       d1-2 has no nameplate, so its housing ({int(WRIST_CAM_BOX[0] * 1000)} x\n"
          f"       {int(WRIST_CAM_BOX[1] * 1000)} x {int(WRIST_CAM_BOX[2] * 1000)} mm forward of the face) and the lens offset\n"
          f"       within it are placeholders; the optical origin sits ON the\n"
          f"       face.  The lens looks along +x = the face normal, 15 deg off\n"
          f"       the approach axis toward the fingers; image down is toward\n"
          f"       the fingers (the real streams show the jaws at the bottom\n"
          f"       edge).  Cross-checked against the d1 teleop dataset by FK,\n"
          f"       2026-08-21. -->\n")
    s += (f"  <joint name=\"{p}_wrist_cam_mount\" type=\"fixed\">\n"
          f"    <origin xyz=\"{_xyz(WRIST_CAM_XYZ)}\" rpy=\"{_xyz(cam_rpy)}\"/>\n"
          f"    <parent link=\"{p}_camera_plate\"/>\n"
          f"    <child link=\"{p}_wrist_cam_link\"/>\n"
          f"  </joint>\n")
    s += (f"  <link name=\"{p}_wrist_cam_link\">\n"
          + full_inertial(WRIST_CAM_MASS,
                          (WRIST_CAM_BOX[0] / 2.0, 0.0, 0.0),
                          (WRIST_CAM_MASS * (WRIST_CAM_BOX[1] ** 2
                                             + WRIST_CAM_BOX[2] ** 2) / 12, 0.0,
                           0.0,
                           WRIST_CAM_MASS * (WRIST_CAM_BOX[0] ** 2
                                             + WRIST_CAM_BOX[2] ** 2) / 12, 0.0,
                           WRIST_CAM_MASS * (WRIST_CAM_BOX[0] ** 2
                                             + WRIST_CAM_BOX[1] ** 2) / 12),
                          "PLACEHOLDER 30 g box module, not weighed")
          + box_visual(_xyz((WRIST_CAM_BOX[0] / 2.0, 0.0, 0.0)),
                       _xyz(WRIST_CAM_BOX), meshes,
                       GRIPPER_COLOUR["camera"][1], GRIPPER_COLOUR["camera"][0])
          + box_elem(f"{p}_wrist_cam_housing",
                     (0.0, -WRIST_CAM_BOX[1] / 2.0, -WRIST_CAM_BOX[2] / 2.0),
                     (WRIST_CAM_BOX[0], WRIST_CAM_BOX[1] / 2.0,
                      WRIST_CAM_BOX[2] / 2.0),
                     "PLACEHOLDER module housing, forward of the mount face")
          + "  </link>\n")
    return s + optical_frame(f"{p}_wrist_cam_optical_frame",
                             f"{p}_wrist_cam_link")


#: End effector id -> the function that hangs it off ``TCP_Link_<side>``.
#: The id is dx-manipulator's ``"<maker>/<model>"``, which is how every other
#: consumer already addresses a hand (``get_hand`` / ``get_tool_config`` /
#: d1-isaaclab ``get_end_effector``). The stock gripper is one instance of the
#: abstraction, not a special case.
END_EFFECTORS = {
    "omakase/yubi": yubi,
    "d1/parallel_gripper": parallel_gripper,
}


# --------------------------------------------------------------------------
# Whole-body variant (d1_wholebody.urdf): mobile base + lift as JOINTS.
#
# Lift travel comes straight from the vendor body URDF (see VENDOR BODY):
# q_lift = 0 is the retracted rail with dual_base 0.52678 above the floor,
# q_lift = 0.300 the top.  Head top then sweeps 1.337 … 1.637 m, against a
# spec sheet of 整机高度 1293–1593 mm — 44 mm taller, which is the pan-swept
# corner of the box rather than the nominal head top.
#
# Base: planar x/y/yaw joints under a ground-level `world` root. The REAL
# chassis is a differential 2-wheel drive (nonholonomic — no lateral slide);
# that constraint is NOT expressible as a URDF joint, so consumers doing
# velocity-level IK must command the base through (v, ω) only — see
# wholebody/wb_ik.py, which parametrizes base velocity as
# [xdot, ydot, yawdot] = [v·cosθ, v·sinθ, ω].  Vendor track width for a
# differential model: wheels at y = ±0.15335, radius 0.0853.
#
# The chassis geometry hangs from base_footprint (does NOT ride the lift);
# torso/head/arms hang from dual_base (DOES ride the lift).
# --------------------------------------------------------------------------
WHEEL_HALF_TRACK, WHEEL_RADIUS = 0.15335, 0.0853


def wholebody_root():
    s = "\n  <!-- Whole-body root: ground-level world + planar base + lift -->\n"
    s += '  <link name="world"/>\n'
    prev = "world"
    for jn, jtype, axis, link, lim in (
            ("base_x", "prismatic", "1 0 0", "base_x_link", ' lower="-10" upper="10"'),
            ("base_y", "prismatic", "0 1 0", "base_y_link", ' lower="-10" upper="10"'),
            ("base_yaw", "continuous", "0 0 1", "base_footprint", "")):
        s += (f'  <joint name="{jn}" type="{jtype}">\n'
              '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
              f'    <parent link="{prev}"/>\n'
              f'    <child link="{link}"/>\n'
              f'    <axis xyz="{axis}"/>\n'
              f'    <limit{lim} effort="500" velocity="1.5"/>\n'
              "  </joint>\n")
        s += f'  <link name="{link}">\n' + inertial(1.0 if link != "base_footprint" else 60.0) + "  </link>\n"
        prev = link
    lo, hi = LIFT_RANGE
    s += ('  <joint name="lift" type="prismatic">\n'
          f'    <origin xyz="0 0 {_fmt(DUAL_BASE_GROUND_Z)}" rpy="0 0 0"/>\n'
          '    <parent link="base_footprint"/>\n'
          '    <child link="dual_base"/>\n'
          '    <axis xyz="0 0 1"/>\n'
          f'    <limit lower="{_fmt(lo)}" upper="{_fmt(hi)}"'
          f' effort="{_fmt(LIFT_EFFORT)}" velocity="{_fmt(LIFT_VELOCITY)}"/>\n'
          "  </joint>\n")
    s += '  <link name="dual_base">\n' + inertial(5.0) + "  </link>"
    return s


def neck(articulated, meshes=False):
    """Head subtree hanging off torso_column.

    articulated=False (d1.urdf, the guard model): ONE static `head_link`
    holding the swept AABB of the whole neck+head over the full PTU range.
    The guard's body boxes must be world-axis-aligned at every pose, which a
    rotating head is not — so it is bounded instead of articulated.

    articulated=True (d1_wholebody.urdf): the real vendor pan/tilt chain,
    `neck_pan` → `neck_tilt`, with the head geometry on the tilt link, plus the
    head camera frames (see the CAMERAS block for why they are here and not in
    the guard model).
    """
    if not articulated:
        name, lo, hi, comment = HEAD_BOX
        return ("\n  <!-- Head: static swept keep-out over the neck PTU range."
                "\n       Articulated pan/tilt variant: d1_wholebody.urdf. -->\n"
                '  <link name="head_link">\n' + inertial(2.0)
                + box_elem(name, lo, hi, comment) + "  </link>\n"
                '  <joint name="head_mount" type="fixed">\n'
                '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                '    <parent link="torso_column"/>\n'
                '    <child link="head_link"/>\n'
                '  </joint>')

    s = "\n  <!-- Neck PTU + head, vendor urdf2026072302 (dheadjoint/upheadjoint). -->\n"
    for jname, (xyz, rpy, axis, (lo, hi)), parent, child, box in (
            ("neck_pan", NECK_PAN, "torso_column", "neck_pan_link", NECK_PAN_BOX),
            ("neck_tilt", NECK_TILT, "neck_pan_link", "head_link", HEAD_TILT_BOX)):
        s += (f'  <joint name="{jname}" type="revolute">\n'
              f'    <origin xyz="{_xyz(xyz)}" rpy="{_xyz(rpy)}"/>\n'
              f'    <parent link="{parent}"/>\n'
              f'    <child link="{child}"/>\n'
              f'    <axis xyz="{_xyz(axis)}"/>\n'
              f'    <limit lower="{_fmt(lo)}" upper="{_fmt(hi)}"'
              f' effort="{_fmt(NECK_EFFORT)}" velocity="{_fmt(NECK_VELOCITY)}"/>\n'
              "  </joint>\n")
        host_in_db = (_mat(NECK_PAN[0], NECK_PAN[1]) if child == "neck_pan_link"
                      else _mmul(_mat(NECK_PAN[0], NECK_PAN[1]),
                                 _mat(NECK_TILT[0], NECK_TILT[1])))
        s += (f'  <link name="{child}">\n' + inertial(1.0 if child == "neck_pan_link" else 2.0)
              + (body_mesh_visuals(child, host_in_db) if meshes else "")
              + box_elem(f"{child}_shell", box[0], box[1],
                         "vendor mesh bbox in this link's frame")
              + "  </link>\n")
    return (s + head_camera() + torso_cameras()).rstrip("\n")


def torso_cameras():
    """Chest D435 + back fisheye on `torso_column` (wholebody variants only).

    Both cameras EXIST ON THE PHYSICAL ROBOT and are simply left off/unplugged
    for USB bandwidth, so the real stack never streams them — a simulator
    should. Poses measured off the 2026-08-23 full-robot CAD (D435 slab
    normal / fisheye housing), located through the head-D435 anchor; same
    nominal-not-calibrated caveats as the head camera.
    """
    out = ("\n  <!-- Torso cameras. PRESENT ON THE ROBOT, disabled for USB\n"
           "       bandwidth — enable them in simulation. CAD-measured\n"
           "       nominal poses, not calibrated extrinsics. -->\n")
    for name, xyz, rpy in (
            ("chest_camera", CHEST_CAMERA_XYZ, (0.0, CHEST_CAMERA_PITCH, 0.0)),
            ("back_fisheye", BACK_FISHEYE_XYZ, (0.0, 0.0, math.pi))):
        out += (f'  <joint name="{name}_mount" type="fixed">\n'
                f'    <origin xyz="{_xyz(xyz)}" rpy="{_xyz(rpy)}"/>\n'
                '    <parent link="torso_column"/>\n'
                f'    <child link="{name}_link"/>\n'
                '  </joint>\n'
                + frame_link(f"{name}_link")
                + optical_frame(f"{name}_optical_frame", f"{name}_link"))
    return out


def main(filename="d1.urdf", out_dir=None):
    wholebody, end_effector, meshes = OUTPUTS[filename]
    robot = os.path.splitext(filename)[0]
    u = ['<?xml version="1.0" encoding="utf-8"?>']
    u.append("<!-- AUTO-GENERATED by description/d1/tools/generate_d1_urdf.py — edit the")
    u.append("     generator, not this file.  Full-body D1 COLLISION model: primitives only")
    u.append("     (boxes/cylinders/spheres), no meshes.  See the generator header for the")
    u.append("     frame convention and the provenance of every number.")
    u.append(f"     END EFFECTOR: {end_effector}. -->")
    u.append(f'<robot name="{robot}">')
    if wholebody:
        u.append(wholebody_root())
    else:
        u.append('  <link name="dual_base"/>')

    # torso
    u.append("\n  <!-- Torso: measured boxes from the D1 CAD STEP (2026-07-01). -->")
    t = '  <link name="torso_column">\n' + inertial(20.0)
    if meshes:
        t += body_mesh_visuals("torso_column", _mat())
    for name, lo, hi, comment in BODY_BOXES:
        t += box_elem(name, lo, hi, comment)
    t += "  </link>"
    u.append(t)
    u.append('  <joint name="torso_mount" type="fixed">\n'
             '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
             '    <parent link="dual_base"/>\n'
             '    <child link="torso_column"/>\n'
             '  </joint>')

    # head — swept static box for the guard model, articulated PTU otherwise
    u.append(neck(articulated=wholebody, meshes=meshes))

    # chassis: floor-relative boxes, hung off the floor in both variants
    u.append("\n  <!-- Chassis: authored floor-relative (z = 0 is the floor) and"
             "\n       mounted at floor level, so it does NOT ride the lift. -->")
    c = '  <link name="chassis_link">\n' + inertial(40.0)
    if meshes:
        c += body_mesh_visuals("chassis_link",
                               _mat((0.0, 0.0, -DUAL_BASE_GROUND_Z)))
    for name, lo, hi, comment in CHASSIS_BOXES:
        c += box_elem(name, lo, hi, comment)
    c += "  </link>"
    u.append(c)
    chassis_parent = "base_footprint" if wholebody else "dual_base"
    chassis_z = 0.0 if wholebody else -DUAL_BASE_GROUND_Z
    u.append('  <joint name="chassis_mount" type="fixed">\n'
             f'    <origin xyz="0 0 {_fmt(chassis_z)}" rpy="0 0 0"/>\n'
             f'    <parent link="{chassis_parent}"/>\n'
             '    <child link="chassis_link"/>\n'
             '  </joint>')

    # Camera frames ride the whole-body variants only — the guard model is a
    # keep-out model and a camera frame is not a volume (see CAMERAS).
    u.append(arm("R", end_effector, meshes, cameras=wholebody))
    u.append(arm("L", end_effector, meshes, cameras=wholebody))
    u.append("</robot>")
    text = "\n".join(u) + "\n"

    # Parse what we just built before writing it. A URDF that no parser can
    # read has shipped from a generator in this codebase before (a `--` inside
    # an XML comment), and a generator that never reads its own output is how
    # that gets past review.
    try:
        parsed = ET.fromstring(text)
    except ET.ParseError as exc:
        raise SystemExit(f"{filename}: generated URDF does not parse: {exc}")
    if parsed.get("name") != robot:
        raise SystemExit(f"{filename}: robot name is {parsed.get('name')!r}")

    out = os.path.join(DESC_DIR if out_dir is None else out_dir, filename)
    with open(out, "w") as f:
        f.write(text)
    print(f"wrote {os.path.normpath(out)} ({len(text)} bytes, "
          f"end effector {end_effector})")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(
        description="Generate the description/d1/*.urdf collision models. "
                    "With no arguments it regenerates all of them.")
    ap.add_argument("--only", choices=sorted(OUTPUTS), action="append",
                    help="write just this file (repeatable)")
    # --out-dir DIR: write there instead of description/d1/ (used by the
    # up-to-date regression test to regenerate and diff without touching
    # the committed files).
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()
    for _name in (args.only or sorted(OUTPUTS)):
        main(_name, out_dir=args.out_dir)
