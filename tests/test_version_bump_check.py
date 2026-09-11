"""``tools/check_version_bump.py`` decides correctly what obliges a bump.

The gate itself runs in CI against a real diff; these are its pure parts, so a
change to the parsing cannot quietly turn the gate into a no-op that passes
everything.
"""
import importlib.util
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "check_version_bump.py"

spec = importlib.util.spec_from_file_location("check_version_bump", _SCRIPT)
cvb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cvb)


PYPROJECT = """\
[build-system]
requires = ["setuptools>=64"]

[project]
name = "manipulation-kit"
version = "0.1.0"
dependencies = [
    "numpy",
    "scipy>=1.7",
]

[project.optional-dependencies]
dev = ["pytest", "yourdfpy"]
"""


def test_version_is_the_project_version_not_a_dependency_pin():
    # [build-system] sits above [project] with no version of its own, and the
    # ">=1.7" inside a requirement string must not be mistaken for one.
    assert cvb.parse_version(PYPROJECT) == "0.1.0"


def test_version_absent_reads_as_none():
    assert cvb.parse_version("[project]\nname = 'x'\n") is None


def test_dependencies_collect_across_both_tables():
    assert cvb.parse_dependencies(PYPROJECT) == [
        "setuptools>=64", "numpy", "scipy>=1.7", "pytest", "yourdfpy"]


def test_src_change_without_a_bump_fails():
    required, why = cvb.needs_bump(
        ["src/manipulation_kit/arms/ik.py"], PYPROJECT, PYPROJECT)
    assert required and "src/" in why


def test_docs_and_tests_alone_do_not_oblige_a_bump():
    # A README paragraph changes nothing in a consumer's venv, so requiring a
    # new sha for it would churn five pins for nothing.
    required, _ = cvb.needs_bump(
        ["README.md", "tests/test_cli.py", ".github/workflows/ci.yml"],
        PYPROJECT, PYPROJECT)
    assert not required


def test_a_dependency_edit_obliges_a_bump():
    after = PYPROJECT.replace('"scipy>=1.7",', '"scipy>=1.11",')
    required, why = cvb.needs_bump(["pyproject.toml"], PYPROJECT, after)
    assert required and "dependency" in why


def test_a_comment_only_pyproject_edit_does_not():
    after = PYPROJECT.replace("[project]", "# why this floor\n[project]")
    required, _ = cvb.needs_bump(["pyproject.toml"], PYPROJECT, after)
    assert not required


@pytest.mark.parametrize("current,expected", [
    ("0.1.0", "0.1.1"), ("1.2.9", "1.2.10"), (None, "0.1.1")])
def test_the_error_message_suggests_the_next_version(current, expected):
    assert cvb._suggest(current) == expected
