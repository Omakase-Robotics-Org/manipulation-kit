# Examples

Runnable scripts built on the installed package. They are **outside the wheel**
— nothing in `manipulation_kit` imports them — so they are free to want a
display, a video encoder or a vendor binary without putting any of that in the
dependency list.

Install the kit first (`pip install -e .` from the repository root); each script
resolves the URDF and the HOME pose through that import, so it works from a
checkout and from an installed wheel alike.

| script | what it does | extra needs |
|---|---|---|
| `make_joint_test_gesture.py` | generate the joint-wiggle assembly-test gesture CSV, every keyframe validated by `MotionGuard` | — |
| `preview_gesture.py` | render a gesture CSV to MP4 so you can watch it before a robot does | `mujoco`, `imageio[ffmpeg]` |
| `ik_click_move.py` | click a point in a MuJoCo view, solve IK to it, export the session as a gesture; `--targets FILE` runs the same thing headless | `mujoco`, a display (except `--targets`) |
| `make_cartesian_test_gesture.py` | the Cartesian half of the assembly test | **does not run here**: it shells out to the d1-sdk C++ `fk_batch`/`ik_batch`, which are not in this repository. Kept for the waypoint layout and the seeding strategy; see its header |

The gesture CSV format and the record → preview → play workflow are documented
in [`../docs/GESTURES.md`](../docs/GESTURES.md).
