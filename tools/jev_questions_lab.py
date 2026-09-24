"""Offline question-design lab for the wrist servo's judge: which SHAPE of
question (and which views of the photo) answers "where is the object
relative to the drawn box" best, measured on real wrist photos with a
known answer.

Ground truth comes from the photos themselves: a manifest names each photo,
the object in it and the object's MEASURED centre pixel; the lab draws the
servo's box (``manipulation_kit.agent.servo.mark``) at that centre shifted by
known pixel offsets, so each case has a true direction and distance. Every
case is asked in every view of :data:`manipulation_kit.agent.judge.VIEWS`,
every question of every formulation, once — and the answers are RECORDED
(``--out``), so every table below is recomputed from the record with no
request (``--report RECORD``)::

    python tools/jev_questions_lab.py --manifest photos.json \\
        --judge-url http://127.0.0.1:8766 --out lab.jsonl
    python tools/jev_questions_lab.py --report lab.jsonl

manifest: ``[{"name", "photo", "object", "centre": [u, v], "box": [w, h]}]``
(``box``: the drawn box's size in pixels — the object's projected outline
grown by the servo's tolerance). Nothing moves; the only socket is the judge
server's.

Arms (formulation @ views): ``A`` choice@upright (the servo's question so
far), ``Aflip`` / ``Arot`` choice@flip_v / @rot180, ``A4`` choice@4 views,
``B`` score@upright, ``C`` grasp@upright (B plus the grasp questions),
``Drot`` score@rot180, ``D2`` score@upright+rot180, ``D4`` score@4 views.
The ``servo_*`` columns are what the servo's own rule
(``manipulation_kit.agent.servo.decide`` on ``judge.to_choices``) does with
ONE photo; the inner loop accumulates several.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples" / "agent"))

from manipulation_kit.agent.judge import (ALL_VIEWS, STATE,  # noqa: E402
                                          questions, read, to_choices,
                                          unview, view_image)
from manipulation_kit.agent.servo import DEFAULT_MARGIN, decide  # noqa: E402

#: pixel offsets of the object centre from the box centre, per axis
OFFSETS_PX = (-80, -40, 0, 40, 80)

ARMS: Dict[str, Tuple[str, Tuple[str, ...]]] = {
    "A": ("choice", ("upright",)), "Aflip": ("choice", ("flip_v",)),
    "Arot": ("choice", ("rot180",)), "A4": ("choice", ALL_VIEWS),
    "B": ("score", ("upright",)), "C": ("grasp", ("upright",)),
    "Drot": ("score", ("rot180",)), "D2": ("score", ("upright", "rot180")),
    "D4": ("score", ALL_VIEWS)}


def ordinal(offset_px: float, half_box_px: float) -> int:
    """The score a perfect judge gives an axis offset: 0 on the centre, +-1
    within half a box, +-2 at half a box or more."""
    if offset_px == 0:
        return 0
    size = 2 if abs(offset_px) >= half_box_px else 1
    return size if offset_px > 0 else -size


def collect(manifest: Sequence[Dict[str, Any]], judge, out: Path,
            work: Path, say=print) -> List[Dict[str, Any]]:
    """Ask every case, every view, every question; one JSON line per
    (case, view) with per-question probabilities and wall milliseconds."""
    from manipulation_kit.agent.servo import mark  # noqa: PLC0415
    rows = []
    work.mkdir(parents=True, exist_ok=True)
    with out.open("w") as record:
        for item in manifest:
            u0, v0 = (float(x) for x in item["centre"])
            w, h = (float(x) for x in item["box"])
            items = [q for f in ("choice", "grasp")
                     for q in questions(item["object"], f)]
            state = STATE.format(object=item["object"].replace("_", " "))
            for du in OFFSETS_PX:
                for dv in OFFSETS_PX:
                    # the box is drawn so the object sits (du, dv) from it
                    u, v = u0 - du, v0 - dv
                    tag = f"{item['name']}_{du:+d}_{dv:+d}"
                    marked = mark(Path(item["photo"]), u, v, work / f"{tag}.png",
                                  box_px=(u - w / 2, v - h / 2,
                                          u + w / 2, v + h / 2))
                    for view in ALL_VIEWS:
                        shown = view_image(marked, view,
                                           work / f"{tag}_{view}.png")
                        answers = {}
                        for q in items:
                            started = time.time()
                            probs, _ms = judge._ask(shown, state, [q])
                            answers[q.id] = {
                                "probs": probs[0],
                                "ms": round((time.time() - started) * 1000.0, 1)}
                        row = {"photo": item["name"], "du": du, "dv": dv,
                               "half_box": [w / 2, h / 2], "view": view,
                               "answers": answers}
                        record.write(json.dumps(row) + "\n")
                        record.flush()
                        rows.append(row)
                    say(f"{tag}: asked {len(ALL_VIEWS)} views x {len(items)} "
                        f"questions")
    return rows


def arm_answers(rows: Sequence[Dict[str, Any]], formulation: str,
                views: Sequence[str]) -> Dict[Tuple, Dict[str, Any]]:
    """(photo, du, dv) -> the arm's answers, views mapped back and averaged,
    plus the arm's summed wall milliseconds."""
    items = questions("x", formulation)
    cases: Dict[Tuple, Dict[str, Any]] = {}
    for row in rows:
        if row["view"] not in views:
            continue
        key = (row["photo"], row["du"], row["dv"])
        case = cases.setdefault(key, {"sum": {q.id: [0.0] * len(q.options)
                                              for q in items},
                                      "ms": 0.0, "half": row["half_box"]})
        for q in items:
            answer = row["answers"][q.id]
            for i, p in enumerate(unview(q, answer["probs"], row["view"])):
                case["sum"][q.id][i] += p / len(views)
            case["ms"] += answer["ms"]
    out = {}
    for key, case in cases.items():
        answers = {q.id: read(q, case["sum"][q.id]) for q in items}
        out[key] = {"answers": answers, "dist": to_choices(answers),
                    "ms": case["ms"], "half": case["half"]}
    return out


def _auc(pos: Sequence[float], neg: Sequence[float]) -> float:
    if not pos or not neg:
        return float("nan")
    wins = sum((p > n) + 0.5 * (p == n) for p in pos for n in neg)
    return wins / (len(pos) * len(neg))


def grade(verdict: str, du: float, dv: float) -> str:
    """What the servo would have done with one photo, against the truth:
    ``ok`` (on when on; a step with the right sign on an axis that is off),
    ``wrong`` (a step the wrong way, or along an axis that is on),
    ``premature_on`` (stopped while off) or ``abstain`` (another photo)."""
    if verdict not in ("on", "left", "right", "above", "below"):
        return "abstain"
    if du == dv == 0:
        return "ok" if verdict == "on" else "wrong"
    if verdict == "on":
        return "premature_on"
    axis, sign = {"left": ("u", -1), "right": ("u", 1), "above": ("v", -1),
                  "below": ("v", 1)}[verdict]
    off = du if axis == "u" else dv
    return "ok" if off and (off > 0) == (sign > 0) else "wrong"


def score_arm(cases: Dict[Tuple, Dict[str, Any]]) -> Dict[str, Any]:
    """The arm's numbers: axis sign accuracy and AUC from the raw answers
    (``choice``: P(right) - P(left); ``score``: the expected score),
    ordinal exactness, "on" AUC and hit/miss gap, and the servo's decision
    on ONE photo (:func:`~manipulation_kit.agent.servo.decide` on
    :func:`~manipulation_kit.agent.judge.to_choices`, window 1)."""
    axis_ok, axis_n, exact, exact_n = 0, 0, 0, 0
    axis_pos: List[float] = []
    axis_neg: List[float] = []
    on_pos: List[float] = []
    on_neg: List[float] = []
    graded: Dict[str, int] = {"ok": 0, "wrong": 0, "premature_on": 0,
                              "abstain": 0}
    tops = []
    for (_photo, du, dv), case in cases.items():
        dist, answers = case["dist"], case["answers"]
        hu, hv = case["half"]
        if "horizontal" in answers:
            signal = {"u": answers["horizontal"]["value"],
                      "v": answers["vertical"]["value"]}
            on_score = answers["inside"]["p"]
        else:
            signal = {"u": dist["right"] - dist["left"],
                      "v": dist["below"] - dist["above"]}
            on_score = dist["on"]
        for axis, off, half in (("u", du, hu), ("v", dv, hv)):
            if off:
                axis_n += 1
                axis_ok += (signal[axis] > 0) == (off > 0)
                (axis_pos if off > 0 else axis_neg).append(signal[axis])
            if "horizontal" in answers:
                exact_n += 1
                exact += round(signal[axis]) == ordinal(off, half)
        (on_pos if du == dv == 0 else on_neg).append(on_score)
        verdict, _why = decide(dist, 1, window=1, margin=DEFAULT_MARGIN)
        graded[grade(verdict, du, dv)] += 1
        tops.append(max(dist.values()))
    med = (lambda xs: statistics.median(xs) if xs else float("nan"))
    return {
        "cases": len(cases), "axis_sign": f"{axis_ok}/{axis_n}",
        "axis_auc": round(_auc(axis_pos, axis_neg), 3),
        "ordinal_exact": f"{exact}/{exact_n}" if exact_n else "-",
        "on_auc": round(_auc(on_pos, on_neg), 3),
        "on_gap": (round(min(on_pos) - max(on_neg), 3)
                   if on_pos and on_neg else None),
        "servo_ok": graded["ok"], "servo_wrong": graded["wrong"],
        "servo_premature_on": graded["premature_on"],
        "servo_abstain": graded["abstain"],
        "confidence_min_med_max": [round(min(tops), 2), round(med(tops), 2),
                                   round(max(tops), 2)],
        "ms_per_look_median": round(med([c["ms"] for c in cases.values()])),
    }


def axis_spread(rows: Sequence[Dict[str, Any]], views: Sequence[str],
                say=print) -> None:
    """The gap before the threshold: each score question's expected value
    by the true ordinal (median [min, max]), and P(inside) on / off."""
    cases = arm_answers(rows, "score", views)
    for q, axis in (("horizontal", 1), ("vertical", 2)):
        by: Dict[int, List[float]] = {}
        for key, case in cases.items():
            half = case["half"][axis - 1]
            by.setdefault(ordinal(key[axis], half), []).append(
                case["answers"][q]["value"])
        say(f"  {'+'.join(views)} {q}: " + "  ".join(
            f"{k:+d}: {statistics.median(v):+.2f} [{min(v):+.2f},"
            f"{max(v):+.2f}]" for k, v in sorted(by.items())))
    on = [c["answers"]["inside"]["p"] for k, c in cases.items()
          if k[1] == k[2] == 0]
    off = [c["answers"]["inside"]["p"] for k, c in cases.items()
           if k[1] or k[2]]
    say(f"  {'+'.join(views)} inside: on {sorted(round(x, 2) for x in on)}, "
        f"off median {statistics.median(off):.2f} max {max(off):.2f}")


def report(rows: Sequence[Dict[str, Any]], say=print) -> Dict[str, Any]:
    asked = {row["view"] for row in rows}
    table = {name: score_arm(arm_answers(rows, f, views))
             for name, (f, views) in ARMS.items() if set(views) <= asked}
    keys = list(next(iter(table.values())))
    say("| arm | " + " | ".join(keys) + " |")
    say("|" + "---|" * (len(keys) + 1))
    for name, row in table.items():
        say(f"| {name} {ARMS[name][0]}@{'+'.join(ARMS[name][1])} | "
            + " | ".join(str(row[k]) for k in keys) + " |")
    for views in (("upright",), ("rot180",), ALL_VIEWS):
        if set(views) <= asked:
            axis_spread(rows, views, say)
    grasp = arm_answers(rows, "grasp", ("upright",))
    on = [c["answers"]["grasp_here"]["p"] for (_p, du, dv), c in grasp.items()
          if du == dv == 0]
    off = [c["answers"]["grasp_here"]["p"] for (_p, du, dv), c in grasp.items()
           if du or dv]
    turn = [c["answers"]["jaw_turn"]["value"] for c in grasp.values()]
    say(f"C grasp_here: AUC {_auc(on, off):.3f}, on median "
        f"{statistics.median(on):.2f}, off max {max(off):.2f}; jaw_turn "
        f"(a round roll: truth 0 or any) median {statistics.median(turn):+.0f} "
        f"deg, range {min(turn):+.0f}..{max(turn):+.0f}")
    return table


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--judge-url", default="http://127.0.0.1:8766")
    parser.add_argument("--out", type=Path, default=Path("lab.jsonl"))
    parser.add_argument("--work", type=Path, default=None,
                        help="where the marked photos go (default: beside --out)")
    parser.add_argument("--report", type=Path, default=None, metavar="RECORD",
                        help="recompute the tables from a record; no request")
    args = parser.parse_args(argv)
    if args.report is not None:
        rows = [json.loads(line) for line in args.report.read_text().splitlines()
                if line.strip()]
    else:
        if args.manifest is None:
            parser.error("--manifest, or --report RECORD")
        from jev_judge import RemoteJudge  # noqa: PLC0415
        manifest = json.loads(args.manifest.read_text())
        rows = collect(manifest, RemoteJudge(args.judge_url), args.out,
                       args.work or args.out.parent / "lab_marked")
    report(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
