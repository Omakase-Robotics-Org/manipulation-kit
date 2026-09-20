"""manipulation-kit — D1 manipulation computation, with no hardware in it.

Everything here is PURE COMPUTATION over the D1 humanoid's geometry: inverse
kinematics, the robot description and its generator, the motion guard, the
end-effector tool configs and the retargeting math. Nothing in this package
opens a socket, holds a robot lock or moves a joint.

Hardware access is ``d1-firmwared`` (``exp--d1-firmware``, REST on :4750). The
split is the point: a customer doing "second development" gets the firmware
pre-flashed and this package as the layer they read, extend and plan against.

Subpackages
-----------
``world``        The perception RESULT types — ObjectView / ContainerView /
                 SurfaceView / ArmView / GripperView / WorldView and the
                 FrameGraph that resolves or refuses a pose. The kit never
                 imports a camera; producers fill these in.
``primitives``   The verbs — Approach Grasp Lift Carry Place Release Nudge
                 Retreat GoHome, plus the Pour contract — each with
                 preconditions, a PURE plan() and a MEASURED verifier().
                 ``docs/PRIMITIVE_CONTRACT.md`` is the written contract.
``executor``     The Executor protocol a plan is run through, and two pure
                 test doubles (RecordingExecutor, KinematicExecutor).
``executors``    The ONE exception to "nothing here opens a socket":
                 ``executors.firmware`` runs a planned primitive on a real D1
                 through d1-firmwared, behind the optional ``[firmware]``
                 extra. Nothing else in the package imports it.
``arms``         DLS inverse kinematics with null-space, the joint clamps and
                 safety limits (single source of truth), target shaping /
                 clutch, filters, frames, the side conventions, the guard seam.
``guard``        ``MotionGuard`` — stdlib-only joint-limit clamp, torso
                 keep-out and self-collision check over the primitives-only
                 whole-body URDF. In-process, for planners and IK loops.
``hands``        End-effector identity by ``"<maker>/<model>"``: tool configs
                 (TCP / mass / COM / inertia), vendor CAD descriptions,
                 glove→hand retarget maps.
``description``  The D1 URDF family, its generator, and the provenance-tracked
                 exporter. ``mkit-urdf`` drives it.
``config``       The exported JSON a controller consumes (home/stow poses,
                 safety zones, collision thresholds, tool configs).

Not in the wheel
----------------
Research and tooling live beside the package, not inside it, so installing the
kit installs only what a consumer imports:

* ``contrib/wholebody/`` — whole-body IK research (base + lift + neck + both
  arms). Needs ``mujoco``, and P2 needs a dataset.
* ``examples/`` — runnable scripts: gesture generation and preview,
  click-to-move IK, and ``examples/agent/``: the offer gate, the schema
  exports, the decision trace and two runnable model loops. Agent-shaped code
  is deliberately NOT in the wheel (Shu, 2026-09-19) — the primitives are a
  robot capability, and driving them with a model is one way of using it.
* ``tools/vendoring/`` — re-import CAD from a vendor drop. Needs the private
  assets repository; see its README.
"""

__all__ = ["arms", "guard", "hands", "description", "world", "primitives",
           "executor", "executors"]
