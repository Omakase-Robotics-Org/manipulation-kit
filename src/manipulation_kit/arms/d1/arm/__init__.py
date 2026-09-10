"""the D1 arm (as mounted on D1) — the vendor binding.

Kinematics live in :mod:`~manipulation_kit.arms.d1.arm.kinematics` and
are reached through the arm-agnostic seam
(:func:`manipulation_kit.arms.get_arm_kinematics`,
:func:`manipulation_kit.arms.get_clutch_tuning`) rather than imported directly.

They are NOT re-exported here on purpose: importing them pulls in ``mujoco``,
and a consumer that only wants ``clutch_tuning()`` or the side conventions must
not be made to pay for that. Import the submodule when you want the model.

``D1ArmChannelBus`` used to be re-exported here — ctypes over the vendor
`the vendor arm SDK `.so``, the route every hand on the arm flange reached the wire
through. It did not come to manipulation-kit: the vendor SDK, the arm channel
passthrough and every mode sequence now belong to ``d1-firmwared``
(``exp--d1-firmware``, REST :4750). This package computes; the daemon moves.
"""

__all__: list = []
