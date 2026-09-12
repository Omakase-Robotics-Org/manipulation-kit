"""Enforce that config/safety_zones.json is a faithful DERIVED export of the
authored safety geometry in docs/reference/safety_zones.h.

safety_zones.h is the single authored source of the D1 dual-arm safety
geometry: it is the header the playback tools (gesture_play / loadCsv)
compile and run to refuse an unsafe trajectory (example/gesture_play.cpp
calls omakase_arm::safety::checkSmoothPath on it). config/safety_zones.json
carries the same numbers so that external validators -- a Python validator
and a JavaScript validator in the omakase_core repository, and any future
consumer -- can READ one derived artifact instead of authoring their own
copy of the numbers.

This test TEXT-PARSES the header (no C++ compiler required, same "runs on
any host" property as the header itself and as pyguard) and asserts, field
by field, that every value the header authors is present in the JSON with
the same value, and that the JSON introduces no safety number the header
does not author. If either side is edited without the other, this test
fails -- which is what makes the JSON a derived export rather than a
hand-maintained fourth copy.

The header is authoritative: if a number disagrees, the header wins (it is
where these numbers were authored) and the JSON must be corrected.

NOTE ON THE HEADER'S STATUS IN THIS REPO. It is a frozen reference copy — the
C++ that compiled it stayed in d1-sdk, and d1-firmwared's Rust guard replaces
it. The test is kept because the JSON is still read by real validators and this
is the only check that it has not drifted from the numbers somebody actually
derived. See docs/reference/README.md.
"""
import json
import os
import re

import manipulation_kit

HERE = os.path.dirname(os.path.abspath(__file__))
# manipulation-kit resolves its assets INSIDE the package (no $D1_SDK_DIR, no
# sibling checkout), so the tests do too — importing the anchor rather than
# counting ".." keeps them working from a wheel as well as a checkout.
KIT = os.path.dirname(os.path.abspath(manipulation_kit.__file__))
CONFIG = os.path.join(KIT, "config")
#: The authored C++ header, kept as a FROZEN REFERENCE COPY under
#: docs/reference/. The compiled C++ guard stayed in d1-sdk and is being retired
#: for d1-firmwared's Rust port, so this file is no longer "what runs" — but it
#: is still the only place these numbers were ever AUTHORED, and this test is
#: the only thing standing between config/safety_zones.json and quiet drift.
#: When the daemon publishes its own authored source, repoint HEADER at it and
#: delete the frozen copy. Until then, do not edit either side alone.
HEADER = os.path.join(HERE, "..", "..", "docs", "reference", "safety_zones.h")

# Numbers written as C++ vs JSON literals ("0.50" vs 0.5) compare only after
# float conversion; this is an exactness check on VALUES, so the tolerance is
# tiny -- it absorbs textual formatting, not real drift.
TOL = 1e-9

_NUM = r"[-+]?[0-9]*\.?[0-9]+"


def _floats(text):
    return [float(x) for x in re.findall(_NUM, text)]


# --------------------------------------------------------------------------
# Header parsers -- each pulls one authored construct out of safety_zones.h by
# locating its unique anchor and reading the brace/paren group after it.
# --------------------------------------------------------------------------
def _read_header():
    with open(HEADER) as f:
        return f.read()


def _strip_line_comments(text):
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def parse_arm_chain(src):
    """safety_zones.h::armChain() -> list of 7 (rx,ry,rz, tx,ty,tz)."""
    m = re.search(r"armChain\(\).*?chain\s*=\s*\{\{(.*?)\}\};", src, re.S)
    assert m, "armChain() initializer not found in header"
    body = _strip_line_comments(m.group(1))
    rows = re.findall(r"\{([^{}]*)\}", body)
    chain = []
    for row in rows:
        vals = _floats(row)
        assert len(vals) == 6, f"chain row has {len(vals)} values, want 6: {row!r}"
        chain.append(tuple(vals))
    assert len(chain) == 7, f"chain has {len(chain)} rows, want 7"
    return chain


def parse_capsule_radii(src):
    """safety_zones.h::capsuleRadii() -> list of 8 doubles (Base..Link7)."""
    m = re.search(r"capsuleRadii\(\).*?r\s*=\s*\{([^{}]*)\};", src, re.S)
    assert m, "capsuleRadii() initializer not found in header"
    vals = _floats(_strip_line_comments(m.group(1)))
    assert len(vals) == 8, f"radii has {len(vals)} values, want 8"
    return vals


def parse_mount(src, name):
    """safety_zones.h::mountA()/mountB() -> (rx,ry,rz, tx,ty,tz)."""
    m = re.search(name + r"\(\)\s*\{\s*return\s+fromRpyXyz\(([^)]*)\)", src)
    assert m, f"{name}() fromRpyXyz(...) not found in header"
    vals = _floats(m.group(1))
    assert len(vals) == 6, f"{name} has {len(vals)} args, want 6"
    return tuple(vals)


def parse_zones(src):
    """safety_zones.h::Zones defaults -> dict with torso box, margins, limits."""
    # torso Box{{lo},{hi}}
    tm = re.search(r"Box\s+torso\s*\{\{([^{}]*)\},\s*\{([^{}]*)\}\}", src)
    assert tm, "Zones::torso initializer not found in header"
    lo = _floats(tm.group(1))
    hi = _floats(tm.group(2))
    assert len(lo) == 3 and len(hi) == 3, "torso box needs 3+3 values"

    am = re.search(r"min_arm_arm\s*=\s*(" + _NUM + ")", src)
    bm = re.search(r"min_body_clearance\s*=\s*(" + _NUM + ")", src)
    assert am and bm, "min_arm_arm / min_body_clearance not found in header"

    # limits = {{ {lo,hi}, ... }} -- 7 pairs
    lm = re.search(r"limits\s*=\s*\{\{(.*?)\}\};", src, re.S)
    assert lm, "Zones::limits initializer not found in header"
    pairs = re.findall(r"\{\s*(" + _NUM + r")\s*,\s*(" + _NUM + r")\s*\}",
                       _strip_line_comments(lm.group(1)))
    limits = [(float(a), float(b)) for a, b in pairs]
    assert len(limits) == 7, f"limits has {len(limits)} pairs, want 7"

    return {
        "torso_lo": lo,
        "torso_hi": hi,
        "min_arm_arm": float(am.group(1)),
        "min_body_clearance": float(bm.group(1)),
        "limits": limits,
    }


# --------------------------------------------------------------------------
# The JSON export
# --------------------------------------------------------------------------
def _read_json():
    with open(os.path.join(CONFIG, "safety_zones.json")) as f:
        return json.load(f)


def _eq(a, b):
    return abs(a - b) <= TOL


# --------------------------------------------------------------------------
# Field-by-field equality tests. Every safety number the header authors is
# checked here; a header edit without a JSON edit (or vice versa) fails.
# --------------------------------------------------------------------------
def test_joint_limits_match_header():
    src = _read_header()
    z = parse_zones(src)
    j = _read_json()["joint_limits_deg"]
    for i, (lo, hi) in enumerate(z["limits"], start=1):
        jlo, jhi = j[f"J{i}"]
        assert _eq(jlo, lo), f"J{i} lower: json {jlo} != header {lo}"
        assert _eq(jhi, hi), f"J{i} upper: json {jhi} != header {hi}"


def test_capsule_radii_match_header():
    radii = parse_capsule_radii(_read_header())
    j = _read_json()["capsule_radii_m"]
    names = ["Base", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6",
             "Link7"]
    for name, r in zip(names, radii):
        assert _eq(j[name], r), f"{name}: json {j[name]} != header {r}"


def test_torso_box_and_margins_match_header():
    z = parse_zones(_read_header())
    j = _read_json()
    for k, (jv, hv) in enumerate(zip(j["torso_keepout_box"]["min"],
                                     z["torso_lo"])):
        assert _eq(jv, hv), f"torso min[{k}]: json {jv} != header {hv}"
    for k, (jv, hv) in enumerate(zip(j["torso_keepout_box"]["max"],
                                     z["torso_hi"])):
        assert _eq(jv, hv), f"torso max[{k}]: json {jv} != header {hv}"
    assert _eq(j["min_arm_arm_distance_m"], z["min_arm_arm"]), \
        f"min_arm_arm: json {j['min_arm_arm_distance_m']} != header {z['min_arm_arm']}"
    assert _eq(j["min_body_clearance_m"], z["min_body_clearance"]), \
        f"min_body_clearance: json {j['min_body_clearance_m']} != header {z['min_body_clearance']}"


def test_arm_chain_matches_header():
    chain = parse_arm_chain(_read_header())
    j = _read_json()["arm_chain"]
    assert len(j) == 7, f"json arm_chain has {len(j)} rows, want 7"
    for i, (row, (rx, ry, rz, tx, ty, tz)) in enumerate(zip(j, chain),
                                                        start=1):
        jr = row["rpy"] + row["xyz"]
        hr = [rx, ry, rz, tx, ty, tz]
        for k, (jv, hv) in enumerate(zip(jr, hr)):
            assert _eq(jv, hv), \
                f"chain Joint{i} field {k}: json {jv} != header {hv}"


def test_arm_mounts_match_header():
    src = _read_header()
    j = _read_json()["arm_mounts"]
    for side, fn in (("A", "mountA"), ("B", "mountB")):
        rx, ry, rz, tx, ty, tz = parse_mount(src, fn)
        jr = j[side]["rpy"] + j[side]["xyz"]
        hr = [rx, ry, rz, tx, ty, tz]
        for k, (jv, hv) in enumerate(zip(jr, hr)):
            assert _eq(jv, hv), \
                f"mount {side} field {k}: json {jv} != header {hv}"


def test_shoulder_scalars_are_derived_from_mount():
    """shoulder_half_width_m / shoulder_z_m are scalar conveniences: they must
    equal the y/z of the arm-A mount translation (from the header), so they
    cannot drift into a third independent value."""
    src = _read_header()
    _, _, _, _, ty, tz = parse_mount(src, "mountA")
    j = _read_json()
    assert _eq(j["shoulder_half_width_m"], abs(ty)), \
        f"shoulder_half_width_m {j['shoulder_half_width_m']} != |mountA.y| {abs(ty)}"
    assert _eq(j["shoulder_z_m"], tz), \
        f"shoulder_z_m {j['shoulder_z_m']} != mountA.z {tz}"


def test_json_introduces_no_unbacked_safety_field():
    """The export must carry only fields the header authors (plus documented
    scalar conveniences, units, version and _comment metadata). A new safety
    number appearing in the JSON with no header counterpart is drift in the
    other direction and must be caught here too."""
    allowed = {
        "_comment", "_chain_comment", "_mounts_comment", "version", "units",
        "shoulder_half_width_m", "shoulder_z_m",
        "torso_keepout_box", "min_arm_arm_distance_m", "min_body_clearance_m",
        "capsule_radii_m", "joint_limits_deg", "arm_chain", "arm_mounts",
    }
    extra = set(_read_json()) - allowed
    assert not extra, f"JSON has fields with no header backing: {sorted(extra)}"
