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
  click-to-move IK.
* ``tools/vendoring/`` — re-import CAD from a vendor drop. Needs the private
  assets repository; see its README.
"""

__all__ = ["arms", "guard", "hands", "description"]
