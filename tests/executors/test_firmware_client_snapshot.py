"""The committed d1-firmwared client is the one its metadata says it is.

``_client/`` holds machine output: an OpenAPI document, a client generated from
it, and a ``SNAPSHOT.json`` that claims which is which. Nothing at runtime can
detect a snapshot whose metadata drifted from its files — the comparison at
connect time hashes the *document on disk*, so a stale recorded hash would
simply be ignored and a stale client would look current. These checks are the
place that mismatch is caught.

They need no daemon, no network and no generator.
"""
from __future__ import annotations

import hashlib
import json
import sys

import pytest

from manipulation_kit.executors.firmware import _gen, ensure
from manipulation_kit.executors.firmware.client import ROUTES

needs_310 = pytest.mark.skipif(
    sys.version_info < ensure.MIN_PYTHON,
    reason=("the generated client is emitted with PEP 604 unions at module "
            "scope and imports on Python 3.10+; planning runs on 3.9"))


@pytest.fixture(scope="module")
def spec_document():
    return json.loads(ensure.bundled_spec_bytes())


def test_the_recorded_sha_is_the_sha_of_the_document_on_disk():
    recorded = ensure.snapshot()["spec_sha256"]
    computed = hashlib.sha256(ensure.BUNDLED_SPEC.read_bytes()).hexdigest()
    assert recorded == computed, (
        "SNAPSHOT.json records a hash the vendored document does not have. "
        "Either the document was hand-edited or a refresh was half-applied; "
        "run `mkit-firmware-client refresh --spec <document>`.")
    assert ensure.bundled_spec_sha256() == computed


def test_the_recorded_version_is_the_documents_own(spec_document):
    assert ensure.snapshot()["spec_version"] == spec_document["info"]["version"]


def test_the_recorded_generator_is_the_one_this_package_would_run():
    snapshot = ensure.snapshot()
    assert snapshot["generator"] == _gen.GENERATOR_SPEC, (
        "the pinned generator moved without the snapshot being regenerated, so "
        "a regeneration on a robot would not reproduce the committed tree")
    assert snapshot["package"] == _gen.PACKAGE_NAME


def test_the_routes_the_adapter_spells_by_hand_are_in_the_spec(spec_document):
    """The four verbs are written as literal paths; the spec has to carry them."""
    published = set(spec_document["paths"])
    missing = [route for route in ROUTES if route not in published]
    assert not missing, (
        f"{missing} are not routes this daemon publishes. The adapter in "
        f"client.py would 404 against it.")


def test_the_snapshot_says_where_a_private_document_came_from():
    """Provenance is not decoration: the firmware repository is private."""
    source = ensure.snapshot()["spec_source"]
    assert source.get("repository") and source.get("commit"), source
    assert source.get("obtained_via"), (
        "an outside reader cannot open the firmware repository; the snapshot "
        "has to name the public route this document came through")


@needs_310
def test_the_bundled_tree_imports_and_is_the_generated_client():
    tree = ensure.import_tree(ensure.resolve_client(policy="bundled"))
    assert tree.__name__ == ensure.BUNDLED_MODULE
    # The two names every generated operation is handed.
    assert hasattr(tree, "Client") and hasattr(tree, "AuthenticatedClient")
    from manipulation_kit.executors.firmware._client.d1fw_api.api.arm import (
        arm_state)
    assert callable(arm_state.sync_detailed)


def test_the_committed_tree_was_lowered_to_the_python_the_robots_run():
    """``compat.py`` ran: no 3.11-only construct is left in the tree."""
    offenders = []
    for path in sorted(ensure.BUNDLED_PACKAGE.rglob("*.py")):
        text = path.read_text(encoding="utf-8")
        if "StrEnum" in text:
            offenders.append(f"{path.name}: StrEnum")
        for line in text.splitlines():
            stripped = line.strip()
            if (stripped.startswith("from typing import ")
                    and "Self" in [n.strip() for n in
                                   stripped[len("from typing import "):].split(",")]):
                offenders.append(f"{path.name}: typing.Self")
    assert not offenders, offenders


def test_the_bundled_client_is_a_real_tree_not_an_empty_directory():
    files = list(ensure.BUNDLED_PACKAGE.rglob("*.py"))
    assert (ensure.BUNDLED_PACKAGE / "client.py").is_file()
    assert len(files) == ensure.snapshot()["files"], (
        "SNAPSHOT.json counts a different number of generated files than the "
        "tree holds — a partial commit")
