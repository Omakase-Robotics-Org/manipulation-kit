"""Regression checks for continuous paint boundaries on coarse CAD triangles."""

import pytest
from build_visual_colors import area, partition, regions


def test_head_cap_border_cuts_triangle_and_interpolates_normals():
    triangle = [
        (0, -0.08, 0, 1, 0, 0),
        (0, -0.04, 0, 0, 1, 0),
        (0, -0.04, 0.1, 0, 0, 1),
    ]
    pieces = list(partition(triangle, regions("head_link")))
    navy = [p for c, p in pieces if c == "navy"]
    white = [p for c, p in pieces if c == "white"]
    assert navy and white
    assert all(v[1] <= -0.063 + 1e-12 for p in navy for v in p)
    assert all(v[1] >= -0.063 - 1e-12 for p in white for v in p)
    edge_n = {v for p in navy for v in p if abs(v[1] + 0.063) < 1e-12}
    edge_w = {v for p in white for v in p if abs(v[1] + 0.063) < 1e-12}
    assert len(edge_n) == 2 and edge_n == edge_w
    assert all(sum(v[3:6]) == pytest.approx(1) for v in edge_n)
    assert sum(area(p) for _, p in pieces) == pytest.approx(area(triangle))


def test_narrow_base_band_is_preserved_even_without_vertex_inside_band():
    triangle = [(0, 0, 0.4, 0, 1, 0), (0.1, 0, 0.5, 0, 1, 0), (-0.1, 0, 0.5, 0, 1, 0)]
    pieces = list(partition(triangle, regions("chassis_link")))
    navy = [p for c, p in pieces if c == "navy"]
    assert navy
    assert all(0.438 - 1e-12 <= v[2] <= 0.485 + 1e-12 for p in navy for v in p)
    assert sum(area(p) for _, p in pieces) == pytest.approx(area(triangle))


def test_torso_band_has_exact_horizontal_edge():
    triangle = [(0, 0, 0.20, 1, 0, 0), (0, 0.1, 0.3, 1, 0, 0), (0, -0.1, 0.3, 1, 0, 0)]
    pieces = list(partition(triangle, regions("torso_column")))
    for color, polygon in pieces:
        assert all(
            (v[2] <= 0.249 + 1e-12) if color == "navy" else (v[2] >= 0.249 - 1e-12)
            for v in polygon
        )
    assert sum(area(p) for _, p in pieces) == pytest.approx(area(triangle))


def test_face_accent_meshes_match_complete_cad_components():
    import json
    from build_visual_colors import BODY

    rows = (BODY / "head_link_hifi.obj").read_text().splitlines()
    positions = [tuple(map(float, r.split()[1:4])) for r in rows if r.startswith("v ")]
    faces = [r.split()[1:] for r in rows if r.startswith("f ")]
    feature = json.loads((BODY / "face_feature_colors.json").read_text())
    for color in ("red", "cyan"):
        expected = {
            positions[int(token.split("/")[0]) - 1]
            for index, c in feature["faces"].items()
            if c == color
            for token in faces[int(index)]
        }
        actual = {
            tuple(map(float, r.split()[1:4]))
            for r in (BODY / f"head_link_{color}.obj").read_text().splitlines()
            if r.startswith("v ")
        }
        assert actual == expected
    assert sorted(c["color"] for c in feature["components"].values()) == [
        "cyan",
        "cyan",
        "red",
    ]


def test_collar_wraps_rear_and_has_rounded_lower_edge():
    from build_visual_colors import collar_planes

    planes = collar_planes()

    def painted(x, y, z):
        return all(a * x + b * y + c * z + d >= -1e-10 for a, b, c, d in planes)

    # Front center reaches lower than its corners: a rounded U, not a flat cut.
    assert painted(0.1, 0.0016, 0.502)
    assert not painted(0.1, 0.050, 0.502)
    assert painted(0.1, 0.050, 0.525)
    # The neck wrap continues to the rear, but only down the upper back.
    assert painted(-0.1, 0.0016, 0.565)
    assert not painted(-0.1, 0.0016, 0.545)
    assert painted(0, 0.045, 0.555)
    assert not painted(0.1, 0.075, 0.56)
