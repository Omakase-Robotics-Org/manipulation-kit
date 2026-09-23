# docs/

**ARCHIVE, not instructions.** Institutional knowledge, carried **verbatim**
from the repositories this kit was assembled out of. Some of it describes
workflows and vendor APIs that no longer exist here: the commands in
[`GESTURES.md`](GESTURES.md), for instance, are d1-sdk's and **do not run in
this repository**.

For how to drive a robot TODAY, read the Quickstart in the top-level
[`README.md`](../README.md) and
[`PRIMITIVE_CONTRACT.md`](PRIMITIVE_CONTRACT.md). For the wire, read
`manipulation_kit.executors.firmware`. Everything else below is the record of
measurements, vendor conversations, field failures and decisions that the
numbers in `src/` depend on. These files were not rewritten in the move: they are the
record of measurements, vendor conversations, field failures and decisions that
the numbers in `src/` depend on, and paraphrasing them would quietly destroy
what makes them worth keeping.

| file | what it is | read it before |
|---|---|---|
| [`description-README.md`](description-README.md) | How the description directory is organised, the two sanctioned ways a consumer references it, and the invariants. From d1-sdk `description/README.md`. | touching anything under `manipulation_kit/description/`, or vendoring an asset into another repo |
| [`d1-description-README.md`](d1-description-README.md) | The D1 URDF family in detail: the three files, where every number came from, the 16.5 mm flange void, the gripper's open questions, the camera frames. From d1-sdk `description/d1/README.md`. | changing the generator, or trusting a dimension |
| [`d1-arm-notes.md`](d1-arm-notes.md) | Why the two arms are the SAME physical arm, why the wrist frames land 180° apart, and why the left hand mount compensates. | "fixing" any left/right asymmetry — this is the one that will burn you |
| [`arms-README.md`](arms-README.md) | The arm library's migration status: what moved out of dx-vr-teleop and omakase-core, and what had not yet. From dx-manipulator `arms/README.md`. | adding to `manipulation_kit.arms` |
| [`guard-README.md`](guard-README.md) | The motion guard's model, frames and configuration knobs. From d1-sdk `pyguard/README.md`. | changing a margin, or debugging a rejection |
| [`teach.md`](teach.md) | **CURRENT.** Teaching omakaseos gestures by hand over d1-firmwared with `mkit-teach` (record → export → check → play), the CSV contract as the player reads it, and the brake-release safety contract. JP operator flow + EN dev notes. | teaching or checking a gesture |
| [`GESTURES.md`](GESTURES.md) | **ARCHIVED.** The gesture CSV format and the record → preview → play workflow, from d1-sdk `devices/omakase_arm/GESTURES.md`. Its `omakase-arm …` commands are d1-sdk's and do not exist in this repository; the CSV format is still what [`../examples/`](../examples) reads and writes. | reading a gesture CSV — not for running one |
| [`hardware-notes-pyarmstate.md`](hardware-notes-pyarmstate.md) | Arm modes, faults and recovery, as documented while it lived in d1-sdk. Kept as PROSE only — the code is `d1-firmwared`'s. | asking the daemon for a mode change and wondering what it means |
| [`PRIMITIVE_CONTRACT.md`](PRIMITIVE_CONTRACT.md) | The three-part contract every verb in `manipulation_kit.primitives` keeps: cheap preconditions, a pure pre-checked `plan()`, a measured `verifier()`. Also why orientation is derived rather than emitted, and how a learned verb (`Pour`) fits the same contract. | adding a verb, or consuming one |
| [`HISTORY.md`](HISTORY.md) | The migration log: what this repository was assembled out of, the incidents that produced its rules, what deliberately did not come, and which consumers still import the old modules. Moved out of the root README so that README can address someone who just wants to use the kit. | wondering *why*, or finishing the migration |
| [`reference/`](reference/) | Frozen copies of retired sources that live tests still pin against. Not part of the kit. | see its own README |

Some of these still describe paths as they were in d1-sdk (`description/d1/…`,
`devices/omakase_arm/…`). That is on purpose: they are historical documents, and
the mapping to this repo is in the root [`README.md`](../README.md) table.
