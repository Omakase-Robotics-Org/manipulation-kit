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

MEASURED TOOL GEOMETRY (d1-3, 2026-09-16, Shu, callipers)
---------------------------------------------------------
Everything along the tool axis below is now a CALLIPER MEASUREMENT taken on
the assembled robot, not the vendor CAD. Measured from the Marvin arm flange
face outward, the stack is::

    0 …   2 mm   camera mounting plate       (CAD said 8 mm)
    2 …   9 mm   spacer block                (was an ASSUMED 8 … 16.5 mm band)
    9 …  51 mm   gripper body
   51 …  71 mm   finger base plate
   71 … 129 mm   pads — the graspable depth, 58 mm
                 pad CENTRE 100 mm, pad TIP 129 mm
   max opening, pad face to pad face: 64 mm

The previous 108.5 mm jaw centre, 143.5 mm jaw tip, 136 mm registered TCP and
70 mm opening came from the vendor CAD export (and, for the 136 mm, from
``d1-sdk``'s hardcoded default carried over with it). The CAD is 8.5 mm long
at the jaw centre and 14.5 mm long at the tip; the measurement wins.

Residuals the sketch does NOT resolve, kept visible rather than smoothed
over: the vendored ``tcp_{r,l}_Link.STL`` jaw meshes are ~10 mm longer than
the measured 58 mm pads, and ``camera_plate.STL`` still models an 8 mm disc.
The constants and the primitive collision boxes follow the measurement; the
meshes are vendor CAD and are only replaced by a new CAD drop.

The one thing that could NOT be left as a residual is the jaw meshes ACROSS
the gap. The CAD was cut for a 70 mm opening, so each jaw mesh's inner pad
FACE sits at :data:`CAD_JAW_STROKE_M` = 35 mm in its own link frame — and a
consumer that renders or collides the mesh (Isaac builds its convex hulls
from it) then works to a 70 mm open / 6 mm closed gripper while the joints
say 64 / 0 and the DRIVER only reaches 51.96 mm. So the mesh is not re-cut,
it is MOVED: see :data:`JAW_MESH_ORIGIN_Z_M`.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from .... import assets
from . import description_path

#: Jaw travel of ONE finger, metres. ``tcp_r_joint`` spans ``[0, STROKE]``
#: and ``tcp_l_joint`` mimics it with multiplier -1 over ``[-STROKE, 0]``.
#: MEASURED: the pad faces are 64 mm apart wide open and meet closed, so one
#: finger travels half of that (d1-3, 2026-09-16, callipers). The vendor CAD
#: claimed 35 mm per finger / a 70 mm gap.
JAW_STROKE_M = 0.032

#: The half-opening the vendor CAD was cut for, and therefore where each jaw
#: mesh's inner pad FACE sits in its OWN link frame: ``tcp_r_Link.STL`` spans
#: local z 35 … 75 mm and ``tcp_l_Link.STL`` spans -75 … -35 mm. Kept for the
#: record, and because :data:`JAW_MESH_ORIGIN_Z_M` is derived from it.
CAD_JAW_STROKE_M = 0.035

#: Which way along a jaw link's local +z its pad face lies. Local +z IS the
#: jaw travel axis (both joints carry ``axis = 0 0 -1`` in that frame), so
#: both jaws close toward z = 0 and the face signs are opposite.
JAW_MESH_FACE_SIGN = {"r": +1.0, "l": -1.0}

#: The offset the URDF gives each jaw's ``<visual>`` AND ``<collision>`` mesh,
#: metres along the link's local +z. It is ``JAW_STROKE_M -
#: CAD_JAW_STROKE_M`` = **-3 mm** applied toward the centre, which lands the
#: pad face on the MEASURED half-opening: the faces sit at
#: ``JAW_MESH_FACE_SIGN[jaw] * JAW_STROKE_M`` at ``q = 0``, i.e. 64 mm apart
#: open, meeting closed, and :data:`DRIVEN_OPEN_GAP_M` apart at the driver's
#: stop. Only the gap direction moves; along the approach axis the mesh is
#: untouched and still overshoots :data:`PAD_TIP_Z_M` by 6 mm.
JAW_MESH_ORIGIN_Z_M = {
    jaw: sign * (JAW_STROKE_M - CAD_JAW_STROKE_M)
    for jaw, sign in JAW_MESH_FACE_SIGN.items()
}

#: Gap between the jaw faces at ``q = 0``. Both jaws close inward as ``|q|``
#: grows, so ``q = 0`` is OPEN and ``|q| = JAW_STROKE_M`` is CLOSED — the
#: opposite polarity to the CAN 2.0 position command, where 0.0 is closed.
#: MEASURED pad-to-pad maximum opening: 64 mm.
JAW_OPEN_GAP_M = 2 * JAW_STROKE_M

#: The gap the DRIVEN gripper actually reaches: d1-3 measures 51.96 mm at the
#: driver's commanded ceiling OPEN_RAD = 1.16 rad (2026-08-24), against the
#: 64 mm the mechanism reaches wide open (2026-09-16). The difference is
#: UNUSED TRAVEL, not a modelling error. Planning and sim must use THIS
#: opening, not JAW_OPEN_GAP_M: a 48 mm tape leaves 8 mm of clearance per
#: side on paper and 2 mm in reality. Raising OPEN_RAD would recover margin,
#: but 1.16 was field-tuned on d1-2 — a hardware decision, not one for this
#: file.
#:
#: UNRESOLVED, and left visible on purpose: the rad->mm map implied by these
#: two points is 44.8 mm/rad through zero, which would put the 64 mm stop at
#: 1.43 rad rather than the ~1.55 rad of mechanism travel the CAD implied. So
#: either the travel is shorter than the CAD said or the map is not linear
#: through zero. Both endpoint GAPS are measured; the map between them is
#: not — :data:`JAW_GAP_PER_MOTOR_RAD_M` below is the linear assumption the
#: firmware executor converts the daemon's jaw readings with.
DRIVEN_OPEN_GAP_M = 0.05196
#: ^ the NOMINAL driven opening, for kinematics, sim and dry-runs. It is not
#: what a particular robot's hand opens to: d1-firmwared publishes its own
#: ``open_rad`` ceiling (1.35 rad on d1-2 on 2026-09-22, about 60.5 mm), and
#: the firmware executor reports THAT, through :func:`gap_from_motor_rad`, as
#: ``HandState.open_gap_m``. Nothing reads an environment variable for it.

#: The daemon's driven-open ceiling this description's nominal gap was
#: measured at [motor rad] (d1-3, 2026-08-24).
NOMINAL_OPEN_RAD = 1.16

#: Pad-face gap per motor radian [m/rad] — the kinematic map between the
#: daemon's ``jaw_rad`` / ``open_rad`` and a gap, linear through zero
#: (``jaw_rad = 0`` is closed): 51.96 mm / 1.16 rad = 44.8 mm/rad. The kit
#: owns this GEOMETRY; the daemon owns the VALUE of ``open_rad``.
#:
#: Linear through zero is an assumption with one measured point behind it
#: (see above: extrapolated, it puts the 64 mm mechanical stop at 1.43 rad).
#: It is the map d1-2's 1.35 rad -> 60.5 mm was computed with.
JAW_GAP_PER_MOTOR_RAD_M = DRIVEN_OPEN_GAP_M / NOMINAL_OPEN_RAD


def gap_from_motor_rad(motor_rad: float) -> float:
    """The pad-face gap [m] at a jaw motor position [rad], clipped to the
    mechanism's measured 0 .. :data:`JAW_OPEN_GAP_M`."""
    return max(0.0, min(JAW_OPEN_GAP_M, float(motor_rad) * JAW_GAP_PER_MOTOR_RAD_M))


#: What a NON-GRASPING verb (``probe``, ``press``) asks the hand to be, as the
#: closedness it is driven to. The hand owns the map, not the verb, so a
#: different hand says what "closed" means for it. ``closed``: the pads meet
#: and the fingertips are one blunt probe; ``open``: driven fully open, the
#: two tips lead; ``pinched``: nearly shut, the tips a few millimetres apart.
#: (Added by redesign step 4 for the contact verbs; step 3 is to own and
#: reconcile this map — the values are the obvious ones, not measured.)
HAND_CLOSEDNESS = {"open": 0.0, "pinched": 0.85, "closed": 1.0}
HAND_POSES = tuple(HAND_CLOSEDNESS)


#: The jaw joint value at the driven-open stop: q = (JAW_OPEN_GAP_M -
#: DRIVEN_OPEN_GAP_M) / 2 per finger. Sim "fully open" is this, not 0.
DRIVEN_OPEN_Q = (JAW_OPEN_GAP_M - DRIVEN_OPEN_GAP_M) / 2.0

#: Distance from the mounting flange (``base_link`` origin) to the PAD ROOT,
#: PAD CENTRE and PAD TIP along the gripper's +Z approach axis, and the
#: graspable pad depth between the first two. MEASURED on d1-3 2026-09-16 by
#: Shu with callipers; the previous 108.5 mm centre / 143.5 mm tip came from
#: the vendor CAD export. The jaw links hang off PAD_CENTRE_Z_M and the
#: REGISTERED TCP is PAD_TIP_Z_M (see
#: :mod:`~manipulation_kit.hands.d1.parallel_gripper.toolconfig`).
PAD_ROOT_Z_M = 0.071
PAD_CENTRE_Z_M = 0.100
PAD_DEPTH_M = 0.058
PAD_TIP_Z_M = PAD_ROOT_Z_M + PAD_DEPTH_M      # 0.129

#: Clearance per side an object must leave inside the jaw opening, by WHERE
#: on the jaws it is taken [m]. At the PADS 4 mm — the clearance the kit has
#: planned pad grasps with since the driven opening was measured (43.96 mm
#: graspable of 51.96 mm). At the TIPS 2 mm — the redesign's number (DESIGN
#: C.2) for a fingertip pinch of something flat, which has no pad face to be
#: squared up against; NOT measured on hardware yet. These are properties of
#: this HAND, and the two grasp references (``primitives.grasp_geometry.PAD``
#: / ``TIP``) are built from them.
PAD_CLEARANCE_PER_SIDE_M = 0.004
TIP_CLEARANCE_PER_SIDE_M = 0.002

#: Backwards-compatible name for the pad tip — the number consumers place a
#: TCP against. MEASURED 129 mm (was the CAD's 143.5 mm).
JAW_TIP_Z_M = PAD_TIP_Z_M

#: The arm-end connection plate (V2.0, 2026-08-21) that carries the
#: wide-angle UVC wrist camera; its camera arm extends along ``base_link``
#: +Y. MEASURED at 2 mm on d1-3 2026-09-16 (callipers), not the 8 mm of the
#: CAD drop this file used to quote: the plate is the first 2 mm of the
#: flange stack and the spacer block behind it accounts for the next 7 mm.
#: ``descriptions/camera_plate.STL`` still models an 8 mm disc — that mesh is
#: vendor CAD and only a new drop replaces it. See ``descriptions/README.md``.
CAMERA_PLATE_THICKNESS_M = 0.002

#: Camera MOUNT-face frame in ``base_link``: origin at the centre of the
#: plate's 4-hole camera pattern, local +Z the face normal (15 deg from the
#: flange +Z toward -Y, i.e. toward the fingers), local +Y up the arm. The
#: ROS optical frame is this rotated pi about local Z (fingers at the image
#: bottom, as the real wrist streams show). UNCHANGED by the 2026-09-16
#: measurement: the sketch gives no lateral numbers and no camera numbers,
#: and the mount face is held by the plate's 33 mm riser, not by the disc
#: thickness that did change.
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

    The mesh files themselves are CAD this repository does not carry (see
    :mod:`manipulation_kit.assets`). ``absolute_meshes`` resolves them through
    an assets checkout when one is configured, and otherwise points at where
    they WOULD be: a composed URDF that names an absent file is still the
    right answer — the caller can see what is missing — whereas silently
    emitting a relative path that resolves against the caller's cwd is not.
    """
    path = Path(str(description_path(_urdf_name(camera))))
    tree = ET.parse(path)
    if absolute_meshes:
        for mesh in tree.getroot().iter("mesh"):
            filename = mesh.get("filename")
            if filename is None:
                continue
            mesh.set("filename", str(_resolve(path.parent, filename)))
    return tree


def _resolve(base: Path, filename: str) -> Path:
    """One mesh reference as an absolute path, via the assets layer."""
    here = (base / filename).resolve()
    try:
        found = assets.resolve(str(here.relative_to(assets.PACKAGE_ROOT)))
    except ValueError:
        return here
    return found if found is not None else here


def mesh_paths(camera: bool = False) -> list[Path]:
    """Every mesh the URDF references, as absolute paths, in document order.

    Resolved through :mod:`manipulation_kit.assets`, so a path here exists iff
    the CAD has been fetched or ``$MKIT_ASSETS_DIR`` is set.
    """
    path = Path(str(description_path(_urdf_name(camera))))
    return [_resolve(path.parent, m.get("filename", ""))
            for m in ET.parse(path).getroot().iter("mesh")]


def _urdf_name(camera: bool) -> str:
    return "gripper_with_camera.urdf" if camera else "gripper.urdf"
