"""Offline lab for the servo's v2 drawing: the JAW OPENING (the two pads at
the current gap, carried to the object's depth) and the object's REAL-SHAPE
outline on real wrist photos, and the typed questions about the object
relative to the fingers (``jev_questions`` ``jaws`` / ``jaws_words``).

Ground truth comes from the photos: a manifest names each photo, the arm's
joints when it was taken, the jaw gap, the object, its declared size and
shape and its MEASURED centre pixel. The lab unprojects that pixel onto the
object's top plane through the robot's measured wrist camera, then draws the
jaw opening at the object's depth shifted by known offsets along the jaw axis
and across it (:data:`OFFSETS_M`), so each case has a true image direction
(``xy`` cases). ``turn`` cases paint a synthetic elongated bar over the
object at known angles to the jaw travel (:data:`TURN_ANGLES_DEG`), outline
it, and keep the opening centred on it: the true turn is the one that puts
the jaw travel across the bar. Every case is asked in every view of
:data:`manipulation_kit.agent.judge.VIEWS`, every question once, and the
answers are RECORDED (``--out``); the tables are recomputed from the record
with no request (``--report RECORD``)::

    python examples/agent/jev_jaws_lab.py --manifest jaws.json \\
        --robot-profile tests/data/d1-2.camera_calibration.json \\
        --judge-url http://127.0.0.1:8766 --out jaws.jsonl
    python examples/agent/jev_jaws_lab.py --report jaws.jsonl

manifest: ``[{"name", "photo", "object", "centre": [u, v], "side",
"joints_deg": [7], "gap_m", "size": [l, w, h], "shape", "top_z"}]``
(``top_z``: the base height of the plane the centre pixel is on).
``--segment-url`` draws the segmenter's outline instead of the declared
shape. Nothing moves; the only sockets are the judge's (and the
segmenter's).
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
import warnings
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from manipulation_kit.agent import jaws as J  # noqa: E402
from manipulation_kit.agent.judge import (ALL_VIEWS, draw_letters,  # noqa: E402
                                          read, unview, view_image)
from manipulation_kit.agent.servo import DEFAULT_MARGIN, decide  # noqa: E402
from jev_questions import questions, state_for, to_reading  # noqa: E402
from jev_questions_lab import grade  # noqa: E402

#: the jaw opening's offset from the object (along the jaw axis, across it)
OFFSETS_M = (-0.04, -0.02, 0.0, 0.02, 0.04)
#: the synthetic bar's long axis against the jaw travel, in the image
TURN_ANGLES_DEG = (0.0, 22.5, 45.0, 67.5, 90.0, 112.5, 135.0, 157.5)
#: the synthetic bar (m): long, across, and its colour
BAR_M = (0.10, 0.024)
BAR_RGB = (40, 40, 150)

#: arms: formulation @ views
ARMS: Dict[str, Tuple[str, Tuple[str, ...]]] = {
    "J": ("jaws", ("upright",)), "Jflip": ("jaws", ("flip_v",)),
    "Jrot": ("jaws", ("rot180",)), "J4": ("jaws", ALL_VIEWS),
    "W": ("jaws_words", ("upright",)), "Wflip": ("jaws_words", ("flip_v",)),
    "Wrot": ("jaws_words", ("rot180",)), "W4": ("jaws_words", ALL_VIEWS)}


def camera_for(item: Dict[str, Any], kin, wrist) -> Any:
    from manipulation_kit.perception import WristCamera  # noqa: PLC0415
    side = item.get("side", "right")
    kin.set_joints(side, np.radians(item["joints_deg"]))
    return WristCamera.from_kin(kin, side, **wrist[side])


def truth_point(camera, item) -> np.ndarray:
    """The object's centre (base): the centre pixel on its top plane, then
    half its height down."""
    u, v = item["centre"]
    top = camera.locate(float(u), float(v), plane_z=float(item["top_z"]))
    return np.asarray(top.p, dtype=float) - [0.0, 0.0, float(item["size"][2]) / 2.0]


def _bar(camera, centre, jaw_x, jaw_y, angle_deg) -> List[Tuple[float, float]]:
    a = math.radians(angle_deg)
    along = math.cos(a) * jaw_x + math.sin(a) * jaw_y
    across = -math.sin(a) * jaw_x + math.cos(a) * jaw_y
    half_l, half_w = BAR_M[0] / 2.0, BAR_M[1] / 2.0
    corners = [centre + sl * half_l * along + sw * half_w * across
               for sl, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return [tuple(camera.project(p)) for p in J._edge_loop(corners)]


def cases(manifest, kin, wrist, work: Path, segmenter=None):
    """Every case: (row header, drawn photo). The photo carries the jaw
    opening and the outline (and, for ``turn``, the painted bar)."""
    from manipulation_kit.world import FrameGraph, ObjectView  # noqa: PLC0415
    from PIL import Image, ImageDraw  # noqa: PLC0415
    work.mkdir(parents=True, exist_ok=True)
    for item in manifest:
        camera = camera_for(item, kin, wrist)
        p = truth_point(camera, item)
        depth = J.approach_depth(camera, p)
        obj = ObjectView(item["object"], p=p, size=item["size"])
        seen = camera.project_point(p)
        if segmenter is not None:
            outline = segmenter(Path(item["photo"]), type("L", (), {
                "object": item["object"]})())
        else:
            outline = J.declared_outline(camera, obj, FrameGraph(),
                                         item.get("shape", "box"))
        m = camera.flange_r.as_matrix()
        # where the object is across the approach axis: the opening is
        # centred on it, then offset
        lateral = (float(np.dot(p - camera.flange_p, m[:, 0])),
                   float(np.dot(p - camera.flange_p, m[:, 1])))
        for dx in OFFSETS_M:
            for dy in OFFSETS_M:
                jaws = J.jaw_opening(camera, gap_m=item["gap_m"],
                                     depth_m=depth, gap_source="measured",
                                     shift_m=(lateral[0] + dx, lateral[1] + dy))
                tag = f"{item['name']}_xy_{dx * 1000:+.0f}_{dy * 1000:+.0f}"
                drawn = J.draw(Path(item["photo"]), work / f"{tag}.png",
                               jaws=jaws, outline=outline)
                yield ({"photo": item["name"], "kind": "xy", "dx": dx,
                        "dy": dy, "du": round(seen.u - jaws.centre[0], 1),
                        "dv": round(seen.v - jaws.centre[1], 1),
                        "box": list(jaws.bbox()), "object": item["object"],
                        "fits": item.get("fits", True)}, drawn)
        jaws = J.jaw_opening(camera, gap_m=item["gap_m"], depth_m=depth,
                             gap_source="measured", shift_m=lateral)
        centre = np.asarray(jaws.point)
        for angle in TURN_ANGLES_DEG:
            bar = _bar(camera, centre, m[:, 0], m[:, 1], angle)
            tag = f"{item['name']}_turn_{angle:05.1f}"
            with Image.open(item["photo"]) as source:
                image = source.convert("RGB")
            ImageDraw.Draw(image).polygon(bar, fill=BAR_RGB)
            painted = work / f"{tag}_bar.png"
            image.save(painted)
            shape = J.outline_from_points(bar, "synthetic")
            drawn = J.draw(painted, work / f"{tag}.png", jaws=jaws,
                           outline=shape)
            truth = J.wrap_turn(shape.long_axis_deg + 90.0 - jaws.jaw_angle_deg)
            yield ({"photo": item["name"], "kind": "turn", "angle": angle,
                    "truth_turn": round(truth, 1), "box": list(jaws.bbox()),
                    "object": "bar", "fits": True}, drawn)


def collect(manifest, judge, out: Path, work: Path, kin, wrist,
            formulations=("jaws", "jaws_words"), segmenter=None,
            views: Sequence[str] = ALL_VIEWS, say=print):
    rows = []
    with out.open("w") as record:
        for head, drawn in cases(manifest, kin, wrist, work, segmenter):
            images = {}
            for f in formulations:
                images[f] = drawn
                if f == "jaws":
                    images[f], _where = draw_letters(
                        drawn, head["box"], drawn.with_name(f"{drawn.stem}_L.png"))
            for view in views:
                answers = {}
                for f in formulations:
                    shown = view_image(images[f], view, images[f].with_name(
                        f"{images[f].stem}_{view}.png"))
                    for q in questions(head["object"], f):
                        if head["kind"] == "turn" and q.id not in (
                                "jaw_turn", "letters", "where"):
                            continue
                        key = f"{f}:{q.id}"
                        if key in answers:
                            continue
                        started = time.time()
                        probs, _ms = judge._ask(shown, state_for(
                            head["object"], f), [q])
                        answers[key] = {"probs": probs[0], "ms": round(
                            (time.time() - started) * 1000.0, 1)}
                row = {**head, "view": view, "answers": answers}
                record.write(json.dumps(row) + "\n")
                record.flush()
                rows.append(row)
            say(f"{head['photo']} {head['kind']} "
                f"{head.get('dx', head.get('angle'))} {head.get('dy', '')}: "
                f"asked")
    return rows


def arm_cases(rows, formulation, views):
    """(photo, kind, dx, dy, angle) -> answers averaged over ``views``."""
    out: Dict[Tuple, Dict[str, Any]] = {}
    for row in rows:
        if row["view"] not in views:
            continue
        key = (row["photo"], row["kind"], row.get("dx"), row.get("dy"),
               row.get("angle"))
        case = out.setdefault(key, {"row": row, "sum": {}, "ms": 0.0})
        for q in questions("x", formulation):
            answer = row["answers"].get(f"{formulation}:{q.id}")
            if answer is None:
                continue
            probs = unview(q, answer["probs"], row["view"])
            acc = case["sum"].setdefault(q.id, [0.0] * len(q.options))
            for i, p in enumerate(probs):
                acc[i] += p / len(views)
            case["ms"] += answer["ms"]
    for case in out.values():
        items = {q.id: q for q in questions("x", formulation)}
        case["answers"] = {k: read(items[k], v) for k, v in case["sum"].items()}
    return out


def _asked(rows, f, views) -> bool:
    return any(r["view"] == v and any(k.startswith(f + ":") for k in r["answers"])
               for r in rows for v in views) and all(
        any(r["view"] == v for r in rows) for v in views)


def xy_table(rows, say=print) -> Dict[str, Any]:
    table = {}
    for name, (f, views) in ARMS.items():
        if not _asked(rows, f, views):
            continue
        cases_ = {k: c for k, c in arm_cases(rows, f, views).items()
                  if k[1] == "xy"}
        if not cases_:
            continue
        graded = {"ok": 0, "wrong": 0, "premature_on": 0, "abstain": 0}
        depth_ok = occl_ok = fit_ok = n = 0
        dist_by: Dict[float, List[float]] = {}
        for (_p, _k, dx, dy, _a), case in cases_.items():
            reading = to_reading(case["answers"])
            verdict, _why = decide(reading.direction, 1, window=1,
                                   margin=DEFAULT_MARGIN)
            du, dv = case["row"]["du"], case["row"]["dv"]
            if dx == dy == 0.0:
                du = dv = 0.0
            graded[grade(verdict, du, dv)] += 1
            n += 1
            if reading.depth:
                depth_ok += max(reading.depth, key=reading.depth.get) == "ahead"
            if reading.occluded is not None:
                occl_ok += reading.occluded < 0.5
            if reading.fit is not None:
                fit_ok += (reading.fit >= 0.5) == bool(case["row"]["fits"])
            if reading.distance is not None:
                dist_by.setdefault(round(math.hypot(dx, dy) * 1000),
                                   []).append(reading.distance)
        table[name] = {
            "cases": n, "right": graded["ok"], "wrong_step": graded["wrong"],
            "premature": graded["premature_on"], "abstain": graded["abstain"],
            "ms_per_look": round(statistics.median(c["ms"] for c in cases_.values())),
            "depth=ahead": f"{depth_ok}/{n}", "not_occluded": f"{occl_ok}/{n}",
            "fit_right": f"{fit_ok}/{n}",
            "distance_by_mm": {k: round(statistics.median(v), 2)
                               for k, v in sorted(dist_by.items())}}
    _print(table, say)
    return table


def turn_table(rows, say=print) -> Dict[str, Any]:
    table = {}
    for name, (f, views) in ARMS.items():
        if not _asked(rows, f, views):
            continue
        cases_ = {k: c for k, c in arm_cases(rows, f, views).items()
                  if k[1] == "turn" and "jaw_turn" in c["answers"]}
        if not cases_ or not name.startswith("J"):
            continue
        within = exact = exact_n = 0
        errors = []
        for key, case in cases_.items():
            truth = case["row"]["truth_turn"]
            reading = to_reading(case["answers"])
            err = abs(J.wrap_turn(reading.turn_deg - truth))
            errors.append(err)
            within += err <= 22.5
            options = [-90.0, -45.0, 0.0, 45.0, 90.0]
            nearest = min(options, key=lambda o: abs(J.wrap_turn(o - truth)))
            if abs(J.wrap_turn(nearest - truth)) <= 5.0:   # a 45-deg case
                exact_n += 1
                probs = case["answers"]["jaw_turn"]["probs"]
                chosen = options[int(np.argmax(probs))]
                exact += abs(J.wrap_turn(chosen - nearest)) < 1e-6
        table[name] = {"cases": len(cases_), "within_22.5deg": within,
                       "argmax_exact": f"{exact}/{exact_n}",
                       "median_err_deg": round(statistics.median(errors), 1),
                       "ms_per_look": round(statistics.median(
                           c["ms"] for c in cases_.values()))}
    _print(table, say)
    return table


def _print(table, say):
    if not table:
        return
    keys = list(next(iter(table.values())))
    say("| arm | " + " | ".join(keys) + " |")
    say("|" + "---|" * (len(keys) + 1))
    for name, row in table.items():
        f, views = ARMS[name]
        say(f"| {name} {f}@{'+'.join(views)} | "
            + " | ".join(str(row[k]) for k in keys) + " |")


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--robot-profile", type=Path)
    parser.add_argument("--judge-url", default="http://127.0.0.1:8766")
    parser.add_argument("--segment-url", default=None)
    parser.add_argument("--out", type=Path, default=Path("jaws.jsonl"))
    parser.add_argument("--report", type=Path, nargs="+", default=None)
    parser.add_argument("--formulations", default="jaws,jaws_words")
    parser.add_argument("--views", default=",".join(ALL_VIEWS))
    args = parser.parse_args(argv)
    if args.report is not None:
        rows = [json.loads(line) for path in args.report
                for line in path.read_text().splitlines() if line.strip()]
    else:
        if args.manifest is None or args.robot_profile is None:
            parser.error("--manifest and --robot-profile, or --report RECORD")
        from jev_judge import RemoteJudge  # noqa: PLC0415
        from scene import demo_scene  # noqa: PLC0415
        from manipulation_kit.agent.robot import wrist_camera_from_scene  # noqa: PLC0415
        from manipulation_kit.description.robot_profile import (  # noqa: PLC0415
            RobotProfile, with_profile)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            profile = RobotProfile.resolve(args.robot_profile,
                                           allow_failed_gate=True)
        wrist = wrist_camera_from_scene(with_profile(None, profile),
                                        measured_only=True)
        segmenter = None
        if args.segment_url:
            from segmenter import RemoteSegmenter  # noqa: PLC0415
            segmenter = RemoteSegmenter(args.segment_url)
        _world, kin = demo_scene()
        rows = collect(json.loads(args.manifest.read_text()),
                       RemoteJudge(args.judge_url), args.out,
                       args.out.parent / (args.out.stem + "_marked"), kin,
                       wrist, segmenter=segmenter,
                       formulations=tuple(args.formulations.split(",")),
                       views=tuple(args.views.split(",")))
    say = print
    say("XY (one photo per decision, the servo's decide):")
    xy_table(rows, say)
    say("\nTURN (synthetic bars at known angles to the jaw travel):")
    turn_table(rows, say)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
