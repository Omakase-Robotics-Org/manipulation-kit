"""``mkit-teach`` — teach an omakaseos gesture by hand, over d1-firmwared.

    mkit-teach record    take.json  --url http://127.0.0.1:4750   # brakes off, hand-guide
    mkit-teach keyframes take.json  take.keys.json                # reduce
    mkit-teach export    take.json  --name wave                   # check + <take dir>/wave_motion.csv
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
from .process import (DEFAULT_SPEED, EPSILON_DEG,
                      MIN_KEYFRAME_S, SAG_MAX_S, SAG_VEL_DEG_S, SMOOTH_WINDOW,
                      STRETCH_KEY, KeyframeOptions, SpeedPolicy,
                      reduce_poses, reduce_samples)
from .record import (BRAKE_CONTRACT, BRAKE_WINDOW_S, COMPLIANCE, DEFAULT_RATE_HZ,
                     RecordAborted, Recording, record)

KEYFRAMES_SCHEMA = "manipulation_kit.teach.keyframes/1"
ARMS = {"both": ("left", "right"), "left": ("left",), "right": ("right",)}


def _say(state: str, message: str) -> None:
    print(f"[{state}] {message}", flush=True)


# -- what to run next ---------------------------------------------------------- #
def recover_steps(url: str, wires) -> List[str]:
    """How to put an arm back in a position hold by hand."""
    lines = ["recover the arm(s) at rest: d1-firmwared console Arms -> Recover, or"]
    lines += [f"curl -X POST {url}/v1/arm/{w}/recover -H 'Content-Type: "
              f"application/json' -d '{{}}'" for w in wires]
    return lines


def retry_record(take: Optional[Path], name: Optional[str],
                 arms: Optional[str]) -> str:
    """Record the take again. NEVER with ``--yes``: that skips the typed
    HOLDING confirmation, which a person at the robot must give every time."""
    if name and take is not None and Path(take) == teach_dir() / f"{name}.json":
        cmd = f"mkit-teach record --name {name}"
    elif take is not None:
        cmd = f"mkit-teach record {take}"
    else:
        cmd = "mkit-teach record"
    return cmd + (f" --arms {arms}" if arms else "") + "   # record it again"


def next_steps(step: str, *, ok: bool = True, take: Optional[Path] = None,
               csv: Optional[Path] = None, name: Optional[str] = None,
               url: str = "http://127.0.0.1:4750", unrecovered=(),
               dry_run: bool = False, arms: Optional[str] = None) -> List[str]:
    """The command(s) an operator should run after ``step``, absolute paths
    filled in — the fix when ``ok`` is False. Every subcommand ends by
    printing these (Shu 2026-09-23: "tell me the next command in the log")."""
    name_arg = name or "<name>"
    lines: List[str] = []
    if unrecovered:
        lines += recover_steps(url, unrecovered)
    if step == "record":
        if ok:
            lines.append(f"mkit-teach export {take}"
                         + ("" if name else " --name <name>")
                         + f"   # writes {Path(take).parent / (name_arg + '_motion.csv')}")
        else:
            lines.append(retry_record(take, name, arms))
    elif step == "keyframes":
        lines.append(f"mkit-teach export {take}" + ("" if name else " --name <name>"))
    elif step == "export":
        if ok:
            lines += [f"mkit-teach check {csv} --ascii",
                      f"mkit-teach play {csv} --dry-run"]
        else:
            where = Path(take).parent if take is not None else Path.cwd()
            lines += [f"mkit-teach record {where / (name_arg + '.json')}   # re-teach it",
                      f"mkit-teach export {take} --name {name_arg} --force"
                      f"   # or write it flagged UNSAFE (play then needs --no-safety)"]
    elif step == "check":
        lines.append(f"mkit-teach play {csv} --dry-run" if ok else
                     f"mkit-teach export <take.json> --name {name_arg}   "
                     f"# re-export (or re-teach) the take this CSV came from")
    elif step == "play":
        if ok and dry_run:
            lines.append(f"mkit-teach play {csv}")
        elif ok:
            lines += [f"mkit-teach register {name_arg} --yaml "
                      f"<omakase-core>/robot_stack/robots/omakase/d1/gesture.yaml"
                      f"   # then copy {csv} to csv/ beside it",
                      "mkit-teach record   # the next take (asks for its name)"]
        else:
            lines.append(f"mkit-teach check {csv} --ascii   # see what was refused"
                         if not unrecovered else f"mkit-teach play {csv}   # then again")
    elif step == "register":
        lines.append("mkit-teach record   # the next take (asks for its name)")
    else:
        raise ValueError(f"unknown step {step!r}")
    return lines


def print_next(lines: List[str], *, ok: bool = True) -> None:
    print("Next:" if ok else "To fix:", flush=True)
    for line in lines:
        print(f"  {line}", flush=True)


def _gesture_name(gesture: Gesture, csv: Path) -> str:
    name = gesture.meta.get("name")
    if name:
        return name
    stem = Path(csv).stem
    return stem[:-len("_motion")] if stem.endswith("_motion") else stem


def _executor(args, *, lease_class: str, vel_ratio: Optional[float] = None):
    """The executor both robot commands drive. ``recover_on_entry``: teaching
    starts by holding the arms where they ARE, so an arm found idle (or
    faulted) — e.g. left there by hand — is recovered at its measured pose,
    announced, instead of refused (docs/teach.md, "Order of operations")."""
    from ..executors.firmware import FirmwareExecutor  # noqa: PLC0415
    return FirmwareExecutor(base_url=args.url, lease_class=lease_class,
                            vel_ratio=vel_ratio if vel_ratio is not None else args.vel_ratio,
                            acc_ratio=vel_ratio if vel_ratio is not None else args.vel_ratio,
                            recover_on_entry=True,
                            announce=lambda line: _say("starting", line))


# -- record ------------------------------------------------------------------ #
def _guide(args) -> str:
    """Brake release (hand guiding) unless told otherwise."""
    if args.compliance and args.no_brake:
        raise SystemExit("mkit-teach record: --compliance and --no-brake are "
                         "mutually exclusive")
    return "compliance" if args.compliance else "idle" if args.no_brake else "brake"


def confirm_holding(args, arms, *, ask=None) -> bool:
    """The brake contract, printed and acknowledged ONCE for the session and
    all taught arms (one typed HOLDING, not one per arm)."""
    ask = _asker(ask)
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


def teach_dir() -> Path:
    """Where named takes and their CSVs live: ``$MKIT_TEACH_DIR`` or ``~/teach``."""
    return Path(os.environ.get("MKIT_TEACH_DIR") or "~/teach").expanduser().resolve()


def _asker(ask):
    """The prompt function: the one given, else ``input`` looked up NOW (so a
    replaced ``builtins.input`` — a scripted stdin — is honoured)."""
    return ask if ask is not None else (lambda prompt: input(prompt))


def _interactive(args=None) -> bool:
    return not getattr(args, "yes", False) and sys.stdin.isatty()


def _ask_name(ask, prompt: str = "Gesture name (a-z, 0-9, _, -): ") -> str:
    while True:
        try:
            name = ask(prompt).strip()
        except EOFError:
            raise SystemExit("mkit-teach record: no gesture name given") from None
        try:
            return registry.check_name(name)
        except ValueError as exc:
            print(exc, file=sys.stderr)


def resolve_take(args, *, ask=None, interactive: Optional[bool] = None):
    """``(take path, gesture name or None)`` for ``record``.

    A positional path is used as given (the name is ``--name``, else its
    stem when that is a valid name). Otherwise the name is ``--name`` or —
    interactively — asked for, and the take is ``<teach dir>/<name>.json``;
    an existing take there is only overwritten when the operator says so
    (or ``--yes``), else a new name is asked for.
    """
    ask = _asker(ask)
    interactive = _interactive(args) if interactive is None else interactive
    if args.out is not None:
        path = Path(args.out).expanduser().resolve()
        name = args.name or path.stem
        try:
            return path, registry.check_name(name)
        except ValueError:
            if args.name:
                raise SystemExit(f"mkit-teach record: bad --name {args.name!r}") from None
            return path, None
    if args.name:
        name = registry.check_name(args.name)
    elif interactive:
        name = _ask_name(ask)
    else:
        raise SystemExit("mkit-teach record: give --name NAME (or a take path); "
                         "the name is asked for only on a terminal")
    while True:
        path = teach_dir() / f"{name}.json"
        if not path.exists() or getattr(args, "yes", False):
            return path, name
        if not interactive:
            raise SystemExit(f"mkit-teach record: {path} exists; choose another "
                             f"--name, or pass --yes to overwrite it")
        answer = ask(f"{path} exists: [o]verwrite, or [n]ew name? ").strip().lower()
        if answer in ("o", "overwrite"):
            return path, name
        name = _ask_name(ask)


def resolve_arms(args, *, ask=None, interactive: Optional[bool] = None) -> str:
    """``--arms``, else — on a terminal — asked; else ``both``."""
    ask = _asker(ask)
    if args.arms is not None:
        return args.arms
    interactive = _interactive(args) if interactive is None else interactive
    if not interactive:
        return "both"
    short = {"b": "both", "l": "left", "r": "right", "": "both"}
    while True:
        try:
            answer = ask("Arms to teach [both/left/right] (default both): ").strip().lower()
        except EOFError:
            return "both"
        answer = short.get(answer, answer)
        if answer in ARMS:
            return answer
        print(f"answer both, left or right (b/l/r), not {answer!r}", file=sys.stderr)


def cmd_record(args) -> int:
    guide = _guide(args)
    take, name = resolve_take(args)
    arms = resolve_arms(args)
    if guide == "brake" and not confirm_holding(args, ARMS[arms]):
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
            print("Press Enter to stop (Ctrl-C while recording also stops and keeps the take).")

        def should_stop() -> bool:
            return stop.is_set() or bool(stop_path and stop_path.exists())
    else:
        def next_keyframe() -> bool:
            line = input("Enter = capture this pose, q + Enter = done: ").strip()
            return line.lower() not in ("q", "quit", "done")
        should_stop = None
    from ..executors.firmware import FirmwareUnavailable  # noqa: PLC0415
    out = take
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        with _executor(args, lease_class=args.lease_class) as robot:
            rec = record(robot, home=home, guide=guide, arms=ARMS[arms],
                         mode=args.mode, rate_hz=args.rate_hz,
                         duration_s=args.duration_s, stationary_s=args.stationary_s,
                         stop=should_stop, next_keyframe=next_keyframe,
                         home_start=not args.no_home_start,
                         brake_window_s=args.brake_window_s,
                         adj_limit_mm=args.adj_limit_mm, countdown_s=args.countdown,
                         allow_bare_flange=args.allow_bare_flange, name=name,
                         on_state=_say)
    except RecordAborted as exc:
        print(f"take discarded, nothing saved: {exc}", file=sys.stderr)
        return _record_ended(args, out, name, arms, exc.recording, code=1)
    except KeyboardInterrupt as exc:
        print("interrupted before recording started; nothing saved",
              file=sys.stderr)
        return _record_ended(args, out, name, arms,
                             getattr(exc, "teach_recording", None), code=130)
    except (FirmwareUnavailable, RuntimeError) as exc:
        print(f"record failed: {exc}", file=sys.stderr)
        return _record_ended(args, out, name, arms,
                             getattr(exc, "teach_recording", None), code=1,
                             failed=True)
    rec.save(out)
    print(f"[recorded] {out}: {len(rec.samples)} samples, "
          f"{rec.times[-1] - rec.times[0]:.2f} s")
    unrecovered = _unrecovered_wires(rec)
    if unrecovered:
        print_next(recover_steps(args.url, unrecovered), ok=False)
    print_next(next_steps("record", take=out, name=name, url=args.url))
    return 0


def _record_ended(args, take, name, arms, rec, *, code: int,
                  failed: bool = False) -> int:
    """No take was saved. ``To fix:`` only for what really needs fixing — an
    arm left without a position hold, or a failure — then how to record
    again."""
    unrecovered = _unrecovered_wires(rec) if rec is not None else []
    if unrecovered:
        print_next(recover_steps(args.url, unrecovered), ok=False)
    if failed:
        print_next(next_steps("record", ok=False, take=take, name=name, arms=arms),
                   ok=False)
    else:
        print_next([retry_record(take, name, arms)])
    return code


def _unrecovered_wires(rec: Recording) -> List[str]:
    """The arms the teardown left without a position hold."""
    return [w for w in ("a", "b")
            if any(p.startswith(f"arm {w} was not put back")
                   for p in rec.meta.get("exit_problems", []))]


def default_csv(source: Path, name: str) -> Path:
    """Where ``export`` writes when no ``out`` is given: beside the take, as
    the library names it (``<name>_motion.csv``), absolute — so the path the
    hint prints is the path ``play`` needs, whatever the cwd."""
    return Path(source).expanduser().resolve().parent / registry.csv_filename(name)


# -- keyframes / export ------------------------------------------------------ #
def _options(args) -> KeyframeOptions:
    return KeyframeOptions(
        smooth_window=0 if args.no_smooth else args.smooth_window,
        epsilon_deg=args.epsilon_deg, method=args.method,
        min_spacing_s=args.min_spacing_s, pin_wrist=args.pin_wrist,
        pin_home=not args.no_home, max_idle_s=args.max_idle_s,
        speed_limit=not args.no_speed_limit, speed=_speed(args),
        min_keyframe_s=args.min_keyframe_s, home_speed_deg_s=args.home_speed,
        sag_max_s=0.0 if args.no_sag_trim else args.sag_max_s,
        sag_vel_deg_s=args.sag_vel)


def _speed(args, gesture: Optional[Gesture] = None) -> SpeedPolicy:
    """THE speed ceiling for this command: the gesture's own (its CSV
    header), else the default, with the caps given on the command line."""
    base = SpeedPolicy.of(gesture) if gesture is not None else DEFAULT_SPEED
    return base.override(args.max_joint_vel, args.max_joint_acc)


def _gesture_from(path: Path, args):
    """``(gesture, home, reduction)``: a recording -> keyframes with
    ``args``'s options and its :class:`Reduction` report; a keyframes JSON or
    a CSV -> as stored, and no report."""
    home = load_home(args.home)
    if path.suffix == ".csv":
        return load_csv(path), home, None
    doc = json.loads(path.read_text(encoding="utf-8"))
    if doc.get("schema") == KEYFRAMES_SCHEMA:
        return Gesture([Keyframe(r[0], r[1:]) for r in doc["keyframes"]],
                       meta=dict(doc.get("meta") or {})), home, None
    rec = Recording.from_json(doc)
    o = _options(args)
    if rec.mode == "keyframe":
        reduction = reduce_poses(rec.samples, home, args.segment_s, o)
    else:
        reduction = reduce_samples(rec.times, rec.samples, home, o)
    gesture = reduction.gesture
    if rec.meta.get("name"):
        gesture.meta.setdefault("name", rec.meta["name"])
    return gesture, home, reduction


def _say_reduction(reduction) -> None:
    """Where the time went and which joints kept their motion."""
    if reduction is None:
        return
    for line in reduction.lines():
        print(line)


def cmd_keyframes(args) -> int:
    gesture, home, reduction = _gesture_from(Path(args.recording), args)
    doc = {"schema": KEYFRAMES_SCHEMA, "home": home, "meta": gesture.meta,
           "keyframes": [[round(k.duration, 4)] + [round(v, 4) for v in k.positions]
                         for k in gesture.keyframes]}
    out = Path(args.out).expanduser().resolve()
    out.write_text(json.dumps(doc, indent=1), encoding="utf-8")
    _say_reduction(reduction)
    _say_speed(gesture)
    print(f"{len(gesture.keyframes)} keyframes, {gesture.played_s:.2f} s -> {out}")
    print_next(next_steps("keyframes", take=out))
    return 0


def _say_speed(gesture: Gesture) -> None:
    """Say so when the speed ceiling slowed the taught timing — only then."""
    stretch = gesture.meta.get(STRETCH_KEY)
    if stretch:
        print(f"speed: {stretch}. Pass --max-joint-vel / --max-joint-acc to keep "
              f"more of the recorded speed (this ceiling is the only gesture "
              f"speed policy: the daemon caps 350 deg/s per step, the omakaseos "
              f"player none; docs/teach.md 'Speed').")


def ask_labels(args, *, ask=None, interactive: Optional[bool] = None):
    """``(sentiment, usage)``: as given; when omitted, asked for on a terminal,
    else the defaults ``neutral`` / ``filler``."""
    ask = _asker(ask)
    interactive = sys.stdin.isatty() if interactive is None else interactive
    sentiment, usage = args.sentiment, args.usage
    if sentiment is None:
        sentiment = "neutral"
        if interactive:
            while True:
                answer = ask(f"Sentiment {registry.SENTIMENTS} [neutral]: ").strip()
                if not answer or answer in registry.SENTIMENTS:
                    sentiment = answer or "neutral"
                    break
    if usage is None:
        usage = ["filler"]
        if interactive:
            while True:
                answer = ask(f"Usage, space-separated {registry.USAGES} [filler]: ").split()
                if all(u in registry.USAGES for u in answer):
                    usage = answer or ["filler"]
                    break
    return sentiment, usage


def cmd_export(args) -> int:
    source = Path(args.source).expanduser().resolve()
    gesture, home, reduction = _gesture_from(source, args)
    name = args.name or gesture.meta.get("name")
    if name:
        name = registry.check_name(name)
    if args.out is None and not name:
        raise SystemExit("mkit-teach export: give --name (the CSV is then written "
                         "beside the take as <name>_motion.csv) or an explicit out path")
    target = (Path(args.out).expanduser().resolve() if args.out is not None
              else default_csv(source, name))
    sentiment, usage = ask_labels(args)
    _say_reduction(reduction)
    _say_speed(gesture)
    try:
        out, report = export(gesture, home, name=name, sentiment=sentiment,
                             usage=usage, force=args.force,
                             speed=_speed(args, gesture))
    except UnsafeGesture as exc:
        print(str(exc), file=sys.stderr)
        print_next(next_steps("export", ok=False, take=source, name=name), ok=False)
        return 1
    print(report.summary())
    if not report.ok:
        print("FORCE-SAVED UNSAFE: `play` will refuse this file without "
              "--no-safety", file=sys.stderr)
    save_csv(target, out)
    # the same number `check` prints: the daemon spline's end time
    print(f"[saved] {target}: {len(out.keyframes)} keyframes, "
          f"{out.played_s:.2f} s on the daemon's spline")
    if name:
        print("gesture.yaml entry (omakaseos robot_stack/robots/omakase/d1/):")
        print(registry.entry_yaml(name, sentiment=sentiment, usage=usage), end="")
        if args.register:
            how = registry.register(Path(args.register), name,
                                    sentiment=sentiment, usage=usage)
            print(f"{how} d1_{name} in {args.register}; copy the CSV to "
                  f"csv/{registry.csv_filename(name)} beside it")
    print_next(next_steps("export", csv=target, name=name))
    return 0


# -- check / play / register ------------------------------------------------- #
def cmd_check(args) -> int:
    gesture = load_csv(Path(args.csv))
    home = load_home(args.home)
    report = check_gesture(gesture, home, step_s=args.step_s,
                           speed=_speed(args, gesture))
    # Exit status: the HARD checks only. Guard clearance findings print as
    # WARNING lines (advisory for a taught gesture, docs/teach.md "Guard").
    if gesture.unsafe:
        print("this file was force-saved UNSAFE: " + "; ".join(gesture.unsafe))
    print(report.summary())
    if args.ascii:
        print(ascii_preview(gesture, home))
    ok = report.ok and not gesture.unsafe
    csv = Path(args.csv).expanduser().resolve()
    print_next(next_steps("check", ok=ok, csv=csv, name=_gesture_name(gesture, csv)),
               ok=ok)
    return 0 if ok else 1


def cmd_play(args) -> int:
    from .play import APPROACH_RATIO, play, preflight  # noqa: PLC0415
    gesture = load_csv(Path(args.csv))
    home = load_home(args.home)
    speed = _speed(args, gesture)
    csv = Path(args.csv).expanduser().resolve()
    name = _gesture_name(gesture, csv)
    if args.dry_run:
        pre = preflight(gesture, home, no_safety=args.no_safety, speed=speed)
        print(pre.detail if not pre.check else pre.check.summary())
        print_next(next_steps("play", ok=pre.ok, csv=csv, name=name, dry_run=True),
                   ok=pre.ok)
        return 0 if pre.ok else 1
    from ..executor import controller_fault  # noqa: PLC0415
    from ..executors.firmware import FirmwareUnavailable  # noqa: PLC0415
    try:
        approach = args.vel_ratio if args.vel_ratio is not None else APPROACH_RATIO
        with _executor(args, lease_class=args.lease_class, vel_ratio=approach) as robot:
            report = play(robot, gesture, home, no_safety=args.no_safety,
                          speed=speed, vel_ratio=args.vel_ratio,
                          announce=lambda line: print(line, flush=True))
            faulted = controller_fault(robot.state()) is not None
    except FirmwareUnavailable as exc:
        print(f"play failed: {exc}", file=sys.stderr)
        print_next(next_steps("play", ok=False, csv=csv, name=name, url=args.url,
                              unrecovered=("a", "b")), ok=False)
        return 1
    print(report.detail)
    for note in report.notes:
        print(f"  note: {note}")
    print_next(next_steps("play", ok=report.ok, csv=csv, name=name, url=args.url,
                          unrecovered=("a", "b") if faulted else ()),
               ok=report.ok)
    return 0 if report.ok else 1


def cmd_register(args) -> int:
    sentiment, usage = ask_labels(args)
    print(registry.entry_yaml(args.name, sentiment=sentiment, usage=usage), end="")
    if args.yaml:
        how = registry.register(Path(args.yaml), args.name,
                                sentiment=sentiment, usage=usage)
        print(f"{how} d1_{args.name} in {args.yaml}")
    print_next(next_steps("register", name=args.name))
    return 0


def _common(p, *, robot: bool, vel_ratio: Optional[float] = 0.15,
            vel_help: str = "position-mode velocity/acceleration ratio, a FRACTION"
            ) -> None:
    p.add_argument("--home", default=None,
                   help="home_pose.json (default: the kit's config/home_pose.json)")
    if robot:
        p.add_argument("--url", default=os.environ.get("D1FW_URL", "http://127.0.0.1:4750"),
                       help="d1-firmwared origin (default $D1FW_URL or 127.0.0.1:4750)")
        p.add_argument("--vel-ratio", type=float, default=vel_ratio, help=vel_help)


def _speed_args(p) -> None:
    """The speed ceiling (process.SpeedPolicy). Unset = the gesture's own
    (its CSV header), or the default for a recording."""
    p.add_argument("--max-joint-vel", type=float, default=None,
                   help=f"joint speed ceiling [deg/s] (default: the CSV's own, "
                        f"else {DEFAULT_SPEED.max_joint_vel_deg_s:g})")
    p.add_argument("--max-joint-acc", type=float, default=None,
                   help=f"joint acceleration ceiling [deg/s^2] (default: the "
                        f"CSV's own, else {DEFAULT_SPEED.max_joint_acc_deg_s2:g})")


def _keyframe_args(p) -> None:
    g = p.add_argument_group("keyframe reduction (gesture_record defaults)")
    g.add_argument("--smooth-window", type=int, default=SMOOTH_WINDOW,
                   help="median+mean window in samples, odd (default 5)")
    g.add_argument("--no-smooth", action="store_true")
    g.add_argument("--epsilon-deg", type=float, default=EPSILON_DEG)
    g.add_argument("--method", choices=("collinear", "dp"), default="collinear")
    g.add_argument("--min-spacing-s", type=float, default=0.0)
    g.add_argument("--pin-wrist", action="store_true",
                   help="pin J5-J7 to HOME (gesture_record's anti-sag rule). "
                        "Default: the wrist is kept as taught; only a wrist "
                        "joint that moved less than 2 deg (sag/noise) is pinned")
    g.add_argument("--no-home", action="store_true",
                   help="do not pin HOME first/last (the omakaseos player "
                        "still replaces those rows with HOME)")
    g.add_argument("--max-idle-s", type=float, default=0.0,
                   help="trim idle pauses to this dwell (0 = keep; the panel "
                        "offered 0.25 / 0.5 / 1)")
    g.add_argument("--no-speed-limit", action="store_true",
                   help="keep the taught timing (no stretch); the check still "
                        "refuses what exceeds the ceiling")
    _speed_args(g)
    g.add_argument("--min-keyframe-s", type=float, default=MIN_KEYFRAME_S)
    g.add_argument("--home-speed", type=float, default=None,
                   help="joint speed [deg/s] of the HOME-in blend and the "
                        "appended return to HOME (default: the take's own peak "
                        "joint speed, clamped to 20..90; keyframe mode 20)")
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
    p.add_argument("--sentiment", choices=registry.SENTIMENTS, default=None,
                   help="default neutral (asked for on a terminal when omitted)")
    p.add_argument("--usage", nargs="+", choices=registry.USAGES, default=None,
                   help="default filler (asked for on a terminal when omitted)")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="mkit-teach", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("record", help="hand-guide the arms and sample them")
    _common(p, robot=True)
    p.add_argument("out", nargs="?", default=None,
                   help="recording JSON to write (default: <teach dir>/<name>.json, "
                        "teach dir = $MKIT_TEACH_DIR or ~/teach; the name is "
                        "asked for, or --name)")
    p.add_argument("--name", default=None,
                   help="gesture name (a-z, 0-9, _, -); asked for when omitted")
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
    p.add_argument("--arms", choices=tuple(ARMS), default=None,
                   help="arm(s) to teach (default: asked on a terminal, else both)")
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
    p.add_argument("out", nargs="?", default=None,
                   help="CSV to write (default: <source dir>/<name>_motion.csv, "
                        "needs --name)")
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
    _speed_args(p)
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("play", help="play a CSV through FirmwareExecutor")
    _common(p, robot=True, vel_ratio=None,
            vel_help="position-mode ratio, a FRACTION (default: derived from the "
                     "gesture's own peak speed, 0.3..1.0, so the controller "
                     "tracks it as taught; the HOME approach runs at 0.3)")
    p.add_argument("csv")
    p.add_argument("--no-safety", action="store_true",
                   help="skip the kit's pre-flight (the daemon still guards)")
    p.add_argument("--dry-run", action="store_true", help="pre-flight only")
    _speed_args(p)
    p.add_argument("--lease-class", choices=("operator", "policy"), default="policy")
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
