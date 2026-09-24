"""``manipulation_kit.agent.judge``: what the servo's judge is asked, in
which shape, about which view — and the recorded d1-2 answers replayed
through the servo's accumulation rule."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from manipulation_kit.agent.judge import (ALL_VIEWS, SCORE_VOTE, AskingJudge,
                                          questions, read, score_verdict,
                                          to_choices, unview)
from manipulation_kit.agent.servo import (CHOICES, DEFAULT_MARGIN, ServoLook,
                                          accumulate, decide)

FIXTURE = (Path(__file__).resolve().parents[1] / "data"
           / "servo_judge_d1_2_20260924.json")


def _answers(per_view, formulation, views):
    """A recorded photo's raw per-view answers, mapped back and averaged —
    what :class:`AskingJudge` computes from a live transport."""
    items = questions("tape", formulation)
    summed = {q.id: [0.0] * len(q.options) for q in items}
    for view in views:
        for q in items:
            for i, p in enumerate(unview(q, per_view[view][q.id], view)):
                summed[q.id][i] += p / len(views)
    return {q.id: read(q, summed[q.id]) for q in items}


def test_the_formulations_ask_what_they_say():
    choice, = questions("tape", "choice")
    assert choice.kind == "choice" and choice.ids == CHOICES
    score = questions("tape", "score")
    assert [(q.id, q.kind) for q in score] == [
        ("inside", "noul"), ("horizontal", "score"), ("vertical", "score")]
    assert all(q.ids == (-2, -1, 0, 1, 2) for q in score[1:])
    grasp = questions("tape", "grasp")
    assert [q.id for q in grasp] == [q.id for q in score] + ["grasp_here",
                                                             "jaw_turn"]
    with pytest.raises(ValueError):
        questions("tape", "sixty_ways")


def test_a_mirrored_view_maps_back_to_the_upright_photo():
    choice, = questions("tape", "choice")
    # asked about the photo flipped top-bottom, "below" means upright "above"
    shown = [0.1, 0.0, 0.0, 0.0, 0.9, 0.0]           # below 0.9
    back = dict(zip(CHOICES, unview(choice, shown, "flip_v")))
    assert back["above"] == 0.9 and back["below"] == 0.0
    back = dict(zip(CHOICES, unview(choice, [0, 0.8, 0.2, 0, 0, 0], "rot180")))
    assert back["right"] == 0.8 and back["left"] == 0.2
    assert unview(choice, shown, "upright") == shown
    horizontal = questions("tape", "score")[1]
    assert unview(horizontal, [1, 0, 0, 0, 0], "flip_h") == [0, 0, 0, 0, 1]
    assert unview(horizontal, [1, 0, 0, 0, 0], "flip_v") == [1, 0, 0, 0, 0]
    turn = questions("tape", "grasp")[-1]
    assert unview(turn, [1, 0, 0, 0, 0], "flip_v") == [0, 0, 0, 0, 1]
    assert unview(turn, [1, 0, 0, 0, 0], "rot180") == [1, 0, 0, 0, 0]


def test_a_score_look_votes_by_its_own_thresholds():
    def answers(h, v, inside):
        return {"horizontal": {"value": h}, "vertical": {"value": v},
                "inside": {"p": inside}}
    assert score_verdict(answers(-0.9, 0.2, 0.9)) == "left"
    assert score_verdict(answers(0.1, 1.1, 0.2)) == "below"
    assert score_verdict(answers(0.1, -0.2, 0.7)) == "on"
    assert score_verdict(answers(0.3, -0.4, 0.5)) == ""
    vote = to_choices(answers(0.1, -1.2, 0.3))
    assert vote["above"] == pytest.approx(SCORE_VOTE)
    assert sum(vote.values()) == pytest.approx(1.0)
    assert decide(vote, 1, window=3, margin=DEFAULT_MARGIN)[0] == "above"
    none = to_choices(answers(0.1, 0.1, 0.1))
    assert decide(none, 1, window=3, margin=DEFAULT_MARGIN)[0] == "more"


def test_the_d1_2_photos_upright_step_the_wrong_way_and_the_fix_does_not():
    """The three marked photos of the d1-2 live run (the roll 63 px ABOVE
    the box), their recorded answers replayed through the servo's rule.
    Upright, the six-way answers — each under 0.37 — accumulate to "below":
    a step away from the roll. Over the four views the servo does not step;
    the score form on the rotated photo votes "above" on every photo."""
    fixture = json.loads(FIXTURE.read_text())
    photos = list(fixture["photos"].values())
    assert fixture["truth"]["choice"] == "above" and len(photos) == 3

    def accumulated(formulation, views):
        dists = [to_choices(_answers(p, formulation, views)) for p in photos]
        return decide(accumulate(dists), len(dists), window=3,
                      margin=DEFAULT_MARGIN)[0]

    assert accumulated("choice", ("upright",)) == "below"      # the bias
    assert accumulated("choice", ALL_VIEWS) not in ("below", "left", "right")
    assert accumulated("choice", ("flip_v",)) == "above"
    for photo in photos:
        answers = _answers(photo, "score", ("rot180",))
        assert score_verdict(answers) == "above"
        assert answers["vertical"]["value"] < -0.7
    assert accumulated("score", ("rot180",)) == "above"


class Transport(AskingJudge):
    """A fake transport: 'below' in the photo AS SHOWN, whatever it is."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.shown = []

    def _ask(self, image, state, items):
        self.shown.append(Path(image).name)
        out = []
        for q in items:
            if q.kind == "choice":
                out.append([1.0 if c == "below" else 0.0 for c in q.ids])
            elif q.id == "vertical":
                out.append([0.0, 0.0, 0.0, 0.0, 1.0])
            else:
                out.append([0.5] * len(q.options))
        return out, 1.0


def _look(tmp_path):
    from PIL import Image
    photo = tmp_path / "m.png"
    Image.new("RGB", (64, 48), (128, 128, 128)).save(photo)
    return ServoLook(side="right", object="tape", camera=None, u=0.0, v=0.0,
                     depth_m=0.2, image=photo, photo=photo)


def test_a_judge_that_always_says_below_cancels_over_the_views(tmp_path):
    """The bias the four views are for: an answer that does not follow the
    image averages to a tie between above and below, never a step."""
    judge = Transport()
    dist = judge(_look(tmp_path))
    assert dist["above"] == pytest.approx(0.5) and dist["below"] == pytest.approx(0.5)
    assert decide(dist, 1, window=3, margin=DEFAULT_MARGIN)[0] == "more"
    assert judge.shown == ["m.png", "m_flip_v.png", "m_flip_h.png",
                           "m_rot180.png"]
    assert judge.last["views"] == list(ALL_VIEWS)
    upright = Transport(views=("upright",))(_look(tmp_path))
    assert max(upright, key=upright.get) == "below"
    with pytest.raises(ValueError):
        Transport(views=("sideways",))


# --------------------------------------------------------------------------- #
# the server's batch endpoint, and the lab that measures formulations
# --------------------------------------------------------------------------- #

class FakeJev:
    loaded = True

    def __init__(self):
        self.calls = []

    def predict(self, *, state, question, options, media, modality):
        self.calls.append(question)
        return {"probabilities": {o: (0.7 if i == 0 else 0.3 / (len(options) - 1))
                                  for i, o in enumerate(options)}}


@pytest.fixture
def server():
    import threading
    import jev_judge_server
    fake = FakeJev()
    httpd = jev_judge_server.make_server(fake, "127.0.0.1", 0)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{httpd.server_address[1]}", fake
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_the_remote_judge_batches_every_question_of_a_view(agent_examples,
                                                           tmp_path, server):
    from jev_judge import RemoteJudge
    url, fake = server
    judge = RemoteJudge(url, formulation="score", views=("upright", "rot180"))
    dist = judge(_look(tmp_path))
    assert judge.batch is True
    assert len(fake.calls) == 2 * 3            # 3 questions x 2 views
    assert set(dist) == set(CHOICES)
    answers = judge.last["answers"]
    # option 0 is "yes" / "far left" / "far above" in the photo as shown:
    # upright and rotated disagree about the axes, so they average to level
    assert answers["inside"]["p"] == pytest.approx(0.7)
    assert answers["horizontal"]["value"] == pytest.approx(0.0, abs=1e-9)


def test_the_lab_grades_what_the_servo_would_do(agent_examples, tmp_path):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
    import jev_questions_lab as lab
    assert lab.ordinal(0, 47) == 0 and lab.ordinal(-40, 47) == -1
    assert lab.ordinal(80, 47) == 2
    assert lab.grade("on", 0, 0) == "ok" and lab.grade("left", 0, 0) == "wrong"
    assert lab.grade("on", 40, 0) == "premature_on"
    assert lab.grade("right", 40, -80) == "ok"      # the minor axis, right sign
    assert lab.grade("above", 40, 0) == "wrong"     # an axis that is on
    assert lab.grade("more", 40, 0) == "abstain"
    # a record whose every answer is the truth scores perfectly
    rows = []
    items = questions("tape", "choice") + questions("tape", "grasp")
    for du in lab.OFFSETS_PX:
        for dv in lab.OFFSETS_PX:
            truth = ("on" if du == dv == 0 else
                     ("right" if du > 0 else "left") if abs(du) >= abs(dv)
                     else ("below" if dv > 0 else "above"))
            answers = {}
            for q in items:
                if q.kind == "choice":
                    probs = [1.0 if c == truth else 0.0 for c in q.ids]
                elif q.id in ("horizontal", "vertical"):
                    want = lab.ordinal(du if q.id == "horizontal" else dv, 47)
                    probs = [1.0 if v == want else 0.0 for v in q.ids]
                elif q.id == "inside":
                    probs = [1.0, 0.0] if du == dv == 0 else [0.0, 1.0]
                else:
                    probs = [1.0 / len(q.options)] * len(q.options)
                answers[q.id] = {"probs": probs, "ms": 1.0}
            rows.append({"photo": "p", "du": du, "dv": dv, "view": "upright",
                         "half_box": [47, 47], "answers": answers})
    said = []
    table = lab.report(rows, say=said.append)
    for arm in ("A", "B"):
        assert table[arm]["servo_wrong"] == 0
        assert table[arm]["servo_ok"] == 25, (arm, table[arm])
    assert table["B"]["ordinal_exact"] == "50/50"
    assert said[0].startswith("| arm | cases |")
