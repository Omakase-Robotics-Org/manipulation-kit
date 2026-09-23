"""``mkit-teach`` — teach an omakaseos gesture by hand, over d1-firmwared.

    mkit-teach record    take.json  --url http://127.0.0.1:4750   # brakes off, hand-guide
    mkit-teach keyframes take.json  take.keys.json                # reduce
    mkit-teach export    take.json  wave_motion.csv --name wave   # check + CSV
    mkit-teach check     wave_motion.csv --ascii                  # pre-flight
    mkit-teach play      wave_motion.csv --url http://127.0.0.1:4750
    mkit-teach register  wave --yaml <omakase-core>/robot_stack/robots/omakase/d1/gesture.yaml

The lifecycle the old /d1_teach panel showed — idle -> recording
(brakes released; --compliance for gesture_record's mode) -> recorded -> previewing -> saved / discarded — is these
commands in order; a take you do not export is simply discarded.
Operator guide: docs/teach.md.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
from pathlib import Path
from typing import List, Optional

from . import registry
from .check import ascii_preview, check_gesture
from .export import UnsafeGesture, export
from .gesture_csv import Gesture, Keyframe, load_csv, load_home, save_csv
from .process import (EPSILON_DEG, HOME_SPEED_DEG_S, MAX_JOINT_ACC_DEG_S2,
                      MAX_JOINT_VEL_DEG_S, MIN_KEYFRAME_S, SAG_MAX_S,
                      SAG_VEL_DEG_S, SMOOTH_WINDOW, KeyframeOptions,
                      keyframes_from_poses, keyframes_from_samples)
from .record import (BRAKE_CONTRACT, BRAKE_WINDOW_S, COMPLIANCE, DEFAULT_RATE_HZ,
                     Recording, record)

KEYFRAMES_SCHEMA = "manipulation_kit.teach.keyframes/1"
ARMS = {"both": ("left", "right"), "left": ("left",), "right": ("right",)}


def _say(state: str, message: str) -> None:
    print(f"[{state}] {message}", flush=True)


def _executor(args, *, lease_class: str):
    """The executor both robot commands drive. ``recover_on_entry``: teaching
    starts by holding the arms where they ARE, so an arm found idle (or
    faulted) — e.g. left there by hand — is recovered at its measured pose,
    announced, instead of refused (docs/teach.md, "Order of operations")."""
    from ..executors.firmware import FirmwareExecutor  # noqa: PLC0415
    return FirmwareExecutor(base_url=args.url, lease_class=lease_class,
                            vel_ratio=args.vel_ratio, acc_ratio=args.vel_ratio,
                            recover_on_entry=True,
                            announce=lambda line: _say("starting", line))


# -- record ------------------------------------------------------------------ #
def _guide(args) -> str:
    """Brake release (hand guiding) unless told otherwise."""
    if args.compliance and args.no_brake:
        raise SystemExit("mkit-teach record: --compliance and --no-brake are "
                         "mutually exclusive")
    return "compliance" if args.compliance else "idle" if args.no_brake else "brake"


def confirm_holding(args, arms, *, ask=input) -> bool:
    """The brake contract, printed and acknowledged ONCE for the session and
    all taught arms (one typed HOLDING, not one per arm)."""
    print(BRAKE_CONTRACT.format(window=args.brake_window_s, arms="/".join(arms)))
    if args.yes:
        return True
    try:
        answer = ask(f"Type HOLDING when a person is holding the "
                     f"{'/'.join(arms)} arm(s): ").strip()
    except EOFError:
        answer = ""
    if answer != "HOLDING":
        print("not confirmed; nothing was moved.", file=sys.stderr)
        return False
    return True


def cmd_record(args) -> int:
    guide = _guide(args)
    if guide == "brake" and not confirm_holding(args, ARMS[args.arms]):
        return 2
    home = load_home(args.home)
    stop = threading.Event()
    next_keyframe = None
    if args.mode == "stream":
        if args.stop_file:
            stop_path = Path(args.stop_file)
        else:
            stop_path = None
        if sys.stdin.isatty() and not args.duration_s:
            def wait_enter():
                try:
                    input()
                except EOFError:
                    return
                stop.set()
            threading.Thread(target=wait_enter, daemon=True).start()
            print("Press Enter to stop (Ctrl-C also stops and still saves).")

        def should_stop() -> bool:
            return stop.is_set() or bool(stop_path and stop_path.exists())
    else:
        def next_keyframe() -> bool:
            line = input("Enter = capture this pose, q + Enter = done: ").strip()
            return line.lower() not in ("q", "quit", "done")
        should_stop = None
    with _executor(args, lease_class=args.lease_class) as robot:
        try:
            rec = record(robot, home=home, guide=guide, arms=ARMS[args.arms],
                         mode=args.mode, rate_hz=args.rate_hz,
                         duration_s=args.duration_s, stationary_s=args.stationary_s,
                         stop=should_stop, next_keyframe=next_keyframe,
                         home_start=not args.no_home_start,
                         brake_window_s=args.brake_window_s,
                         adj_limit_mm=args.adj_limit_mm, countdown_s=args.countdown,
                         allow_bare_flange=args.allow_bare_flange, on_state=_say)
        except KeyboardInterrupt:
            print("interrupted; brakes engaged first, then the position hold "
                  "(any arm that could not be recovered is named above)",
                  file=sys.stderr)
            return 130
    rec.save(Path(args.out))
    print(f"saved {args.out}: {len(rec.samples)} samples. Next: "
          f"mkit-teach export {args.out} <name>_motion.csv --name <name>")
    return 0


# -- keyframes / export ------------------------------------------------------ #
def _options(args) -> KeyframeOptions:
    return KeyframeOptions(
        smooth_window=0 if args.no_smooth else args.smooth_window,
        epsilon_deg=args.epsilon_deg, method=args.method,
        min_spacing_s=args.min_spacing_s, lock_wrist=not args.free_wrist,
        pin_home=not args.no_home, max_idle_s=args.max_idle_s,
        speed_limit=not args.no_speed_limit,
        max_joint_vel_deg_s=args.max_joint_vel, max_joint_acc_deg_s2=args.max_joint_acc,
        min_keyframe_s=args.min_keyframe_s, home_speed_deg_s=args.home_speed,
        sag_max_s=0.0 if args.no_sag_trim else args.sag_max_s,
        sag_vel_deg_s=args.sag_vel)


def _gesture_from(path: Path, args):
    """A recording -> keyframes (with ``args``'s options); a keyframes JSON or
    a CSV -> as stored."""
    home = load_home(args.home)
    if path.suffix == ".csv":
        return load_csv(path), home
    doc = json.loads(path.read_text(encoding="utf-8"))
    if doc.get("schema") == KEYFRAMES_SCHEMA:
        return Gesture([Keyframe(r[0], r[1:]) for r in doc["keyframes"]]), home
    rec = Recording.from_json(doc)
    o = _options(args)
    if rec.mode == "keyframe":
        return keyframes_from_poses(rec.samples, home, args.segment_s, o), home
    return keyframes_from_samples(rec.times, rec.samples, home, o), home


def cmd_keyframes(args) -> int:
    gesture, home = _gesture_from(Path(args.recording), args)
    doc = {"schema": KEYFRAMES_SCHEMA, "home": home,
           "keyframes": [[round(k.duration, 4)] + [round(v, 4) for v in k.positions]
                         for k in gesture.keyframes]}
    Path(args.out).write_text(json.dumps(doc, indent=1), encoding="utf-8")
    print(f"{len(gesture.keyframes)} keyframes, {gesture.total_duration_s:.2f} s "
          f"-> {args.out}")
    return 0


def cmd_export(args) -> int:
    gesture, home = _gesture_from(Path(args.source), args)
    try:
        out, report = export(gesture, home, name=args.name, sentiment=args.sentiment,
                             usage=args.usage, force=args.force)
    except UnsafeGesture as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(report.summary())
    if not report.ok:
        print("FORCE-SAVED UNSAFE: `play` will refuse this file without "
              "--no-safety", file=sys.stderr)
    save_csv(Path(args.out), out)
    print(f"[saved] {args.out}: {len(out.keyframes)} keyframes, "
          f"{out.total_duration_s:.2f} s")
    if args.name:
        print("gesture.yaml entry (omakaseos robot_stack/robots/omakase/d1/):")
        print(registry.entry_yaml(args.name, sentiment=args.sentiment,
                                  usage=args.usage), end="")
        if args.register:
            how = registry.register(Path(args.register), args.name,
                                    sentiment=args.sentiment, usage=args.usage)
            print(f"{how} d1_{args.name} in {args.register}; copy the CSV to "
                  f"csv/{registry.csv_filename(args.name)} beside it")
    return 0


# -- check / play / register ------------------------------------------------- #
def cmd_check(args) -> int:
    gesture = load_csv(Path(args.csv))
    home = load_home(args.home)
    report = check_gesture(gesture, home, step_s=args.step_s)
    # Exit status: the HARD checks only. Guard clearance findings print as
    # WARNING lines (advisory for a taught gesture, docs/teach.md "Guard").
    if gesture.unsafe:
        print("this file was force-saved UNSAFE: " + "; ".join(gesture.unsafe))
    print(report.summary())
    if args.ascii:
        print(ascii_preview(gesture, home))
    return 0 if report.ok and not gesture.unsafe else 1


def cmd_play(args) -> int:
    from .play import play, preflight  # noqa: PLC0415
    gesture = load_csv(Path(args.csv))
    home = load_home(args.home)
    if args.dry_run:
        pre = preflight(gesture, home, no_safety=args.no_safety)
        print(pre.detail if not pre.check else pre.check.summary())
        return 0 if pre.ok else 1
    with _executor(args, lease_class=args.lease_class) as robot:
        report = play(robot, gesture, home, no_safety=args.no_safety,
                      guard=args.guard,
                      announce=lambda line: print(line, flush=True))
    print(report.detail)
    for note in report.notes:
        print(f"  note: {note}")
    return 0 if report.ok else 1


def cmd_register(args) -> int:
    print(registry.entry_yaml(args.name, sentiment=args.sentiment, usage=args.usage),
          end="")
    if args.yaml:
        how = registry.register(Path(args.yaml), args.name,
                                sentiment=args.sentiment, usage=args.usage)
        print(f"{how} d1_{args.name} in {args.yaml}")
    return 0


def _common(p, *, robot: bool) -> None:
    p.add_argument("--home", default=None,
                   help="home_pose.json (default: the kit's config/home_pose.json)")
    if robot:
        p.add_argument("--url", default=os.environ.get("D1FW_URL", "http://127.0.0.1:4750"),
                       help="d1-firmwared origin (default $D1FW_URL or 127.0.0.1:4750)")
        p.add_argument("--vel-ratio", type=float, default=0.15,
                       help="position-mode velocity/acceleration ratio, a FRACTION")


def _keyframe_args(p) -> None:
    g = p.add_argument_group("keyframe reduction (gesture_record defaults)")
    g.add_argument("--smooth-window", type=int, default=SMOOTH_WINDOW,
                   help="median+mean window in samples, odd (default 5)")
    g.add_argument("--no-smooth", action="store_true")
    g.add_argument("--epsilon-deg", type=float, default=EPSILON_DEG)
    g.add_argument("--method", choices=("collinear", "dp"), default="collinear")
    g.add_argument("--min-spacing-s", type=float, default=0.0)
    g.add_argument("--free-wrist", action="store_true",
                   help="keep J5-J7 as dragged (default: locked at HOME)")
    g.add_argument("--no-home", action="store_true",
                   help="do not pin HOME first/last (the omakaseos player "
                        "still replaces those rows with HOME)")
    g.add_argument("--max-idle-s", type=float, default=0.0,
                   help="trim idle pauses to this dwell (0 = keep; the panel "
                        "offered 0.25 / 0.5 / 1)")
    g.add_argument("--no-speed-limit", action="store_true")
    g.add_argument("--max-joint-vel", type=float, default=MAX_JOINT_VEL_DEG_S)
    g.add_argument("--max-joint-acc", type=float, default=MAX_JOINT_ACC_DEG_S2)
    g.add_argument("--min-keyframe-s", type=float, default=MIN_KEYFRAME_S)
    g.add_argument("--home-speed", type=float, default=HOME_SPEED_DEG_S,
                   help="joint speed [deg/s] of the HOME-in blend and the "
                        "appended return to HOME (default 20)")
    g.add_argument("--sag-max-s", type=float, default=SAG_MAX_S,
                   help="cut the brake-release sag within this many seconds "
                        "of the start (default 0.5)")
    g.add_argument("--sag-vel", type=float, default=SAG_VEL_DEG_S,
                   help="joint speed [deg/s] above which a start sample is sag")
    g.add_argument("--no-sag-trim", action="store_true")
    g.add_argument("--segment-s", type=float, default=1.5,
                   help="seconds per move for a keyframe-mode take")


def _meta_args(p) -> None:
    p.add_argument("--name", default=None, help="gesture name (a-z, 0-9, _, -)")
    p.add_argument("--sentiment", choices=registry.SENTIMENTS, default="neutral")
    p.add_argument("--usage", nargs="+", choices=registry.USAGES, default=["filler"])


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="mkit-teach", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("record", help="hand-guide the arms and sample them")
    _common(p, robot=True)
    p.add_argument("out", help="recording JSON to write")
    guide = p.add_argument_group(
        "how the arm goes soft (default: RELEASE THE HOLDING BRAKES — hand "
        "guiding; the arm drops unless someone holds it)")
    guide.add_argument("--compliance", action="store_true",
                       help="force_compliance with gesture_record's parameters "
                            "instead (servos on, the arm yields)")
    guide.add_argument("--no-brake", action="store_true",
                       help="servos off (idle), brakes untouched")
    p.add_argument("--yes", action="store_true",
                   help="skip the typed HOLDING confirmation (scripts only)")
    p.add_argument("--countdown", type=int, default=3,
                   help="seconds counted down before the arms go soft (default 3)")
    p.add_argument("--arms", choices=tuple(ARMS), default="both")
    p.add_argument("--mode", choices=("stream", "keyframe"), default="stream",
                   help="stream = sample at --rate-hz; keyframe = Enter per pose")
    p.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    p.add_argument("--duration-s", type=float, default=None)
    p.add_argument("--stationary-s", type=float, default=0.0,
                   help="auto-stop after this long without motion (0 = off)")
    p.add_argument("--stop-file", default=None)
    p.add_argument("--no-home-start", action="store_true",
                   help="do not drive to HOME first (the arms must already be "
                        "within 2 deg of HOME, or the start is refused)")
    p.add_argument("--brake-window-s", type=float, default=BRAKE_WINDOW_S)
    p.add_argument("--adj-limit-mm", type=float,
                   default=COMPLIANCE["adjustment_limit_mm"])
    p.add_argument("--allow-bare-flange", action="store_true")
    p.add_argument("--lease-class", choices=("operator", "policy"), default="operator")
    p.set_defaults(func=cmd_record)

    p = sub.add_parser("keyframes", help="reduce a recording to keyframes")
    _common(p, robot=False)
    p.add_argument("recording")
    p.add_argument("out")
    _keyframe_args(p)
    p.set_defaults(func=cmd_keyframes)

    p = sub.add_parser("export", help="check and write the omakaseos gesture CSV")
    _common(p, robot=False)
    p.add_argument("source", help="recording JSON, keyframes JSON or CSV")
    p.add_argument("out", help="<name>_motion.csv")
    _keyframe_args(p)
    _meta_args(p)
    p.add_argument("--force", action="store_true",
                   help="write even if unsafe; flagged UNSAFE, play needs --no-safety")
    p.add_argument("--register", default=None, metavar="GESTURE_YAML")
    p.set_defaults(func=cmd_export)

    p = sub.add_parser("check", help="limits + rates + timing (hard) and guard "
                                     "clearance (advisory) on the played spline")
    _common(p, robot=False)
    p.add_argument("csv")
    p.add_argument("--ascii", action="store_true", help="print joint strips")
    p.add_argument("--step-s", type=float, default=0.01)
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("play", help="play a CSV through FirmwareExecutor")
    _common(p, robot=True)
    p.add_argument("csv")
    p.add_argument("--no-safety", action="store_true",
                   help="skip the kit's pre-flight (the daemon still guards)")
    p.add_argument("--dry-run", action="store_true", help="pre-flight only")
    p.add_argument("--lease-class", choices=("operator", "policy"), default="policy")
    p.add_argument("--guard", choices=("speed_only", "full"), default="speed_only",
                   help="daemon trajectory guard: speed_only (default) skips its "
                        "clearance checks for this gesture, keeping limits, the "
                        "speed cap and timing; full has it refuse clearance "
                        "violations too")
    p.set_defaults(func=cmd_play)

    p = sub.add_parser("register", help="print / splice the gesture.yaml entry")
    p.add_argument("name")
    _meta_args(p)
    p.add_argument("--yaml", default=None)
    p.set_defaults(func=cmd_register)
    return ap


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if getattr(args, "cmd", None) == "register":
        args.name = registry.check_name(args.name)
    return int(args.func(args) or 0)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
