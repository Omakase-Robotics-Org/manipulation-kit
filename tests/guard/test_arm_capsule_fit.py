"""The guard's arm capsules are the vendor arm meshes, and stay so.

Two halves, because CI has no CAD (it is withheld, see LICENSE-STATUS.md):

* **Always** — the committed ``ARM_CAPSULES`` are exactly the fit record
  ``arm_capsule_fit.json``, the record's per-link residual (how far the
  worst mesh vertex sits outside the capsules: the housing rims and tube
  ends the model does not cover) stays within the accepted bound, and
  ``d1.urdf`` carries exactly those capsules. Editing a radius by hand fails
  here.
* **With the CAD** (``MKIT_ASSETS_DIR``) — the meshes are the ones the record
  was fitted to (sha256), no vertex sticks out further than the link's
  recorded residual + 1 mm, and re-running the fit reproduces the record.
"""
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from manipulation_kit import assets
from manipulation_kit.description.d1.tools import fit_arm_capsules as fit
from manipulation_kit.description.d1.tools import generate_d1_urdf as gen

D1 = os.path.join(os.path.dirname(gen.__file__), "..")
RECORD = json.load(open(os.path.join(D1, "arm_capsule_fit.json")))


def _norm(caps):
    return [[p, [round(x, 6) for x in a], [round(x, 6) for x in b], round(r, 6)]
            for p, a, b, r in caps]


def test_generator_capsules_are_the_fit_record():
    assert set(gen.ARM_CAPSULES) == set(RECORD["links"])
    for link, entry in RECORD["links"].items():
        assert _norm(gen.ARM_CAPSULES[link]) == _norm(entry["capsules"]), link


#: The residual Shu accepted on 2026-09-23 (Link4's elbow-housing rim is the
#: worst); a refit that leaves more of the arm outside must be looked at.
ACCEPTED_RESIDUAL_MM = 25.0


def test_the_recorded_residuals_stay_within_what_was_accepted():
    for link, entry in RECORD["links"].items():
        for side, meta in entry["meshes"].items():
            assert meta["max_protrusion_mm"] <= ACCEPTED_RESIDUAL_MM, (link, side, meta)
            assert meta["vertices"] > 1000, (link, side, meta)


def test_d1_urdf_carries_exactly_the_fitted_capsules():
    root = ET.parse(os.path.join(D1, "d1.urdf")).getroot()
    for side in ("R", "L"):
        for link, caps in gen.ARM_CAPSULES.items():
            el = root.find(f"link[@name='{link}_{side}']")
            prims = {c.get("name"): c for c in el.findall("collision")
                     if "_cap_" not in c.get("name")}
            assert sorted(prims) == sorted(f"{link}_{side}_capsule_{p}" for p, *_ in caps)
            for part, a, b, r in caps:
                g = prims[f"{link}_{side}_capsule_{part}"].find("geometry")
                shape = g.find("cylinder") if g.find("cylinder") is not None else g.find("sphere")
                assert math.isclose(float(shape.get("radius")), r, abs_tol=1e-9)
                if shape.tag == "cylinder":
                    assert math.isclose(float(shape.get("length")),
                                        float(np.linalg.norm(np.subtract(b, a))), abs_tol=1e-6)


def _clouds(link):
    out = {}
    for side in ("R", "L"):
        path = fit.mesh_path(link, side)
        if path is None:
            pytest.skip("vendor arm meshes absent (set MKIT_ASSETS_DIR)")
        out[side] = (str(path), fit.mesh_vertices(str(path)))
    return out


@pytest.mark.skipif(assets.assets_root() is None and fit.mesh_path("Link2", "R") is None,
                    reason="vendor arm meshes absent (set MKIT_ASSETS_DIR)")
@pytest.mark.parametrize("link", sorted(RECORD["links"]))
def test_every_mesh_vertex_is_inside_its_capsules(link):
    for side, (path, P) in _clouds(link).items():
        meta = RECORD["links"][link]["meshes"][side]
        assert fit.sha256(path) == meta["sha256"], f"{link}_{side} mesh changed: refit"
        worst = float(fit.protrusion(P, gen.ARM_CAPSULES[link]).max()) * 1000
        allowed = meta["max_protrusion_mm"] + fit.TOLERANCE_M * 1000
        assert worst <= allowed, (
            f"{link}_{side}: a mesh vertex is {worst:.3f} mm outside the capsules, "
            f"recorded residual {meta['max_protrusion_mm']:.3f} mm + 1 mm")


@pytest.mark.skipif(assets.assets_root() is None and fit.mesh_path("Link2", "R") is None,
                    reason="vendor arm meshes absent (set MKIT_ASSETS_DIR)")
def test_refitting_reproduces_the_record():
    table, record = fit.fit_all()
    for link, caps in table.items():
        assert _norm(caps) == _norm(RECORD["links"][link]["capsules"]), link
