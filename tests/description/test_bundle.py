"""The description bundle: what it holds, that it is what the package builds,
that its digest is stable, and that a damaged copy is refused."""

from __future__ import annotations

import json

import pytest

from manipulation_kit import assets
from manipulation_kit.description import GUARD_URDF
from manipulation_kit.description import bundle as db
from manipulation_kit.guard import MotionGuard
from manipulation_kit.guard.urdf_model import UrdfModel


def test_the_built_bundle_is_the_locked_one():
    lock = db.read_lock()
    assert db.lock_problems(lock, db.digest()) == []
    assert lock["files"] == db.file_hashes(db.bundle_files())
    assert db.description_version() == lock["description_version"]


def test_building_twice_gives_the_same_bytes():
    assert db.bundle_files() == db.bundle_files()


def test_the_digest_is_over_paths_and_hashes_in_path_order():
    h = {"b": "2" * 64, "a": "1" * 64}
    assert db.digest_of(h) == db.digest_of(dict(sorted(h.items())))
    assert db.digest_of(h) != db.digest_of({"a": "2" * 64, "b": "1" * 64})
    assert db.digest_of(h).startswith("sha256:")


def test_the_bundle_holds_the_documented_files():
    paths = set(db.bundle_files())
    for p in ("kinematics/d1.urdf", "kinematics/d1_wholebody_gripper.urdf",
              "collision/collision.json", "guard/profile.d1.json",
              "poses/home_pose.json", "visual/visual.json",
              "end_effectors/tool_configs/d1_parallel_gripper.json"):
        assert p in paths


def test_verbatim_files_are_byte_identical_to_their_source():
    files = db.bundle_files()
    assert files["kinematics/d1.urdf"] == GUARD_URDF.read_bytes()
    cfg = db.ROOT.parent / "config"
    assert files["poses/home_pose.json"] == (cfg / "home_pose.json").read_bytes()
    for p in (cfg / "tool_configs").glob("*.json"):
        assert files[f"end_effectors/tool_configs/{p.name}"] == p.read_bytes()


def test_generated_json_is_canonical():
    for path, data in db.bundle_files().items():
        if path.endswith(".json") and not path.startswith(("poses/", "end_effectors/")):
            text = data.decode("utf-8")
            assert text == json.dumps(json.loads(text), indent=2, sort_keys=True) + "\n"


def test_the_guard_profile_is_the_guard_s_own_defaults():
    prof = json.loads(db.bundle_files()["guard/profile.d1.json"])
    g = MotionGuard()
    assert prof["body_margin_m"] == g.body_margin_m
    assert prof["arm_arm_margin_m"] == g.arm_arm_margin_m
    assert prof["self_margin_m"] == g.self_margin_m
    assert set(prof["body_exempt_links"]) == g.body_exempt_links
    assert set(prof["disabled_body_boxes"]) == g.disabled_body_boxes
    assert set(prof["chest_exempt_links"]) == g.chest_exempt_links
    for side, (lo, hi) in g.chest_keepout.items():
        assert prof["chest_keepout"][side] == {"lo": list(lo), "hi": list(hi)}


def test_the_collision_model_is_the_guard_urdf():
    col = json.loads(db.bundle_files()["collision/collision.json"])
    m = UrdfModel(str(GUARD_URDF))
    assert [link["name"] for link in col["links"]] == m.links
    for link in col["links"]:
        assert [c["name"] for c in link["collisions"]] == \
            [p.name for p in m.collisions[link["name"]]]
        for c, p in zip(link["collisions"], m.collisions[link["name"]]):
            assert c["kind"] == p.kind
            assert tuple(c["size"]) == tuple(p.size)
    by_name = {j["name"]: j for j in col["joints"]}
    for j in m.revolute_joints():
        assert by_name[j.name]["limit"] == [j.lower, j.upper]


def test_every_mesh_reference_is_package_relative_and_classed():
    vis = json.loads(db.bundle_files()["visual/visual.json"])
    assert vis["meshes"], "the whole-body URDF references meshes"
    for m in vis["meshes"]:
        assert not m["package_path"].startswith(("/", ".."))
        parent = m["package_path"].rsplit("/", 1)[0]
        want = ("withheld" if parent in assets.EXTERNAL_ASSET_DIRS else
                "optional" if parent == assets.OPTIONAL_VISUAL_DIR else "shipped")
        assert m["license"] == want


def test_a_written_bundle_verifies_and_loads(tmp_path):
    man = db.write_bundle(tmp_path / "b")
    assert man["digest"] == db.digest()
    assert man["schema"] == db.BUNDLE_SCHEMA
    b = db.load_bundle(tmp_path / "b")
    assert b.digest == man["digest"]
    assert b.description_version == db.description_version()
    assert b.read_json("guard/profile.d1.json")["profile"] == "d1"
    with pytest.raises(KeyError):
        b.path("not/listed.json")


@pytest.mark.parametrize("damage", ["modify", "remove", "extra", "digest"])
def test_a_damaged_bundle_is_refused(tmp_path, damage):
    root = tmp_path / "b"
    db.write_bundle(root)
    if damage == "modify":
        (root / "poses/home_pose.json").write_text("{}\n")
    elif damage == "remove":
        (root / "visual/visual.json").unlink()
    elif damage == "extra":
        (root / "kinematics/other.urdf").write_text("<robot/>")
    else:
        man = json.loads((root / db.MANIFEST_NAME).read_text())
        man["digest"] = "sha256:" + "0" * 64
        (root / db.MANIFEST_NAME).write_text(json.dumps(man))
    assert db.verify_bundle(root)
    with pytest.raises(db.BundleError):
        db.load_bundle(root)


def test_write_refuses_a_non_empty_directory(tmp_path):
    (tmp_path / "x").write_text("x")
    with pytest.raises(db.BundleError):
        db.write_bundle(tmp_path)
    db.write_bundle(tmp_path, force=True)


def test_a_changed_description_needs_a_new_version(tmp_path):
    lock = tmp_path / "lock.json"
    lock.write_text(json.dumps({"schema": db.LOCK_SCHEMA, "model": "d1",
                                "description_version": "1.0.0",
                                "digest": "sha256:" + "0" * 64,
                                "history": {"1.0.0": "sha256:" + "0" * 64}}))
    assert db.lock_problems(db.read_lock(lock), db.digest())
    with pytest.raises(db.BundleError, match="new version"):
        db.update_lock("1.0.0", lock)
    new = db.update_lock("1.1.0", lock)
    assert new["digest"] == db.digest()
    assert db.lock_problems(new, db.digest()) == []
    with pytest.raises(db.BundleError, match="already description_version"):
        db.update_lock("1.2.0", lock)


def test_cli(tmp_path, capsys):
    assert db.main(["digest"]) == 0
    assert db.digest() in capsys.readouterr().out
    assert db.main(["check"]) == 0
    assert db.main(["build", "--out", str(tmp_path / "b")]) == 0
    assert db.main(["verify", str(tmp_path / "b"),
                    "--expect-digest", db.digest()]) == 0
    assert db.main(["verify", str(tmp_path / "b"),
                    "--expect-digest", "sha256:" + "0" * 64]) == 1


def test_the_bundle_carries_the_hand_catalog_verbatim(tmp_path):
    from manipulation_kit.hands import catalog

    files = db.bundle_files()
    assert files[db.HAND_CATALOG] == catalog.CATALOG_FILE.read_bytes()
    db.write_bundle(tmp_path / "b")
    b = db.load_bundle(tmp_path / "b")
    doc = b.hand_catalog()
    assert doc == catalog.load()
    assert {m["model"] for m in doc["models"]} >= {"d1/parallel_gripper", "wuji/wujihand2"}


def test_a_bundle_without_the_hand_catalog_is_refused(tmp_path):
    root = tmp_path / "b"
    db.write_bundle(root)
    (root / db.HAND_CATALOG).unlink()
    man = json.loads((root / db.MANIFEST_NAME).read_text())
    man["files"] = [e for e in man["files"] if e["path"] != db.HAND_CATALOG]
    man["digest"] = db.digest_of({e["path"]: e["sha256"] for e in man["files"]})
    (root / db.MANIFEST_NAME).write_text(json.dumps(man))
    problems = db.verify_bundle(root)
    assert problems == [f"{db.HAND_CATALOG}: required but not in the manifest"]
