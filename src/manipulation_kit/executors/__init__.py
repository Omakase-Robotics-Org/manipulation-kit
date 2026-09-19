"""manipulation_kit.executors — the ONE place in this package that has a wire.

Everything else in ``manipulation-kit`` computes: "Nothing in this repository
opens a socket, holds a robot lock, or moves a joint" is the repository's first
sentence and it is load-bearing. This subpackage is the deliberate exception,
decided by Shu on 2026-09-19 (「firmware transport を使うことになる」): a
primitive that can plan a motion and cannot run one leaves every consumer to
re-derive the same lease handling, the same mode entry and the same rate clamp,
and that is how safety code ends up in four copies that disagree.

The exception is fenced, not waived:

* nothing here is imported by anything else in the package — the only way in is
  to name it, ``from manipulation_kit.executors.firmware import FirmwareExecutor``;
* it needs the optional ``[firmware]`` extra (``d1fw-client``). Without the
  extra the import fails with one line saying so, and the base kit keeps its
  no-socket install;
* it is tested against a fake client, never a robot, so the suite still runs
  on a laptop and a green CI still means nothing was on the network.
"""
