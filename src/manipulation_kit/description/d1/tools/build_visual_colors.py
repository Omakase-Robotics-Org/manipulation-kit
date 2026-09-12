#!/usr/bin/env python3
"""Partition existing CAD triangles into URDF material regions from d1.avif.

Boundary triangles are split on the existing surface and normals interpolated.
The surface shape, kinematics and collision geometry are preserved.
Uses the supplied reference's white/navy palette; boundaries are approximate.
"""

from pathlib import Path
import hashlib
import json

BODY = Path(__file__).resolve().parents[1] / "meshes" / "body_hifi"
PALETTE = {
    "white": "0.90 0.91 0.92 1",
    "navy": "0.012 0.032 0.16 1",
    "dark": "0.015 0.020 0.026 1",
    "silver": "0.48 0.54 0.60 1",
    "cyan": "0.02 0.48 0.90 1",
    "red": "0.72 0.02 0.025 1",
    "optic": "0.008 0.024 0.021 1",
    "gray": "0.48 0.54 0.60 1",
}
HOSTS = ("chassis_link", "torso_column", "neck_pan_link", "head_link")


# A plane (a,b,c,d) keeps a*x+b*y+c*z+d >= 0.
def above(axis, value):
    return tuple(1.0 if i == axis else 0.0 for i in range(3)) + (-value,)


def below(axis, value):
    return tuple(-1.0 if i == axis else 0.0 for i in range(3)) + (value,)


def box(x=None, y=None, z=None):
    return [
        plane
        for axis, bounds in enumerate((x, y, z))
        if bounds
        for plane in (above(axis, bounds[0]), below(axis, bounds[1]))
    ]


def collar_planes():
    """Continuous neck wrap with a rounded front and a shallower rear drop.

    Lower half of an elliptical cylinder, tilted toward the front. Clipping
    against its tangent planes gives a smooth U-shaped edge on both sides.
    The 64 arc segments limit curve approximation error to below 0.02 mm.
    """
    import math

    width, depth, center_z, center_y, slope = 0.058, 0.045, 0.574, 0.0016, 0.30
    planes = [above(1, center_y - width), below(1, center_y + width)]
    for i in range(65):
        angle = math.pi + math.pi * i / 64
        cy, sz = math.cos(angle) / width, math.sin(angle) / depth
        planes.append((-sz * slope, -cy, -sz, 1 + cy * center_y + sz * center_z))
    return planes


def rear_panel_planes():
    """Photo-aligned rounded rear bib surrounding the emergency stop."""
    import math
    cy, cz, width, depth = .0016, .545, .051, .055
    planes = [above(1, cy-width), below(1, cy+width)]
    for i in range(49):
        a = math.pi + math.pi*i/48
        y, z = math.cos(a)/width, math.sin(a)/depth
        planes.append((0, -y, -z, 1+y*cy+z*cz))
    return planes


def regions(host):
    if host == "head_link":
        # Robot up = -local Y; robot left = local Z + .0285.
        return [
            ("dark", [above(0, 0.086)] + box(y=(-0.096, -0.071), z=(-0.0695, 0.0125))),
            ("navy", [below(1, -0.063)]),
            ("dark", [above(1, 0.045)]),
        ]
    if host == "torso_column":
        return [
            ("navy", box(z=(0.175, 0.249))),
            ("red", [below(0, -0.098)] + box(y=(-0.014, 0.017), z=(0.538, 0.566))),
            ("navy", [below(0, -0.055)] + rear_panel_planes()),
            ("dark", [above(0, 0.078)] + box(y=(-0.039, 0.039), z=(0.548, 0.569))),
            ("navy", collar_planes()),
        ]
    if host == "chassis_link":
        return [
            ("white", box(x=(-.046, .066), y=(-.071, .074), z=(.30, .73))),
            ("navy", box(z=(.438, .485))),
            ("navy", [below(0, -.055)] + box(z=(.438, .65))),
            ("silver", box(z=(.238, .274))),
            ("silver", [above(0, .258)] + box(z=(.173, .235))),
        ]
    return [("silver", [])]


def split_polygon(poly, plane):
    """Clip on the CAD surface, interpolating vertex normals and UVs.

    Return two complementary polygons; intersection vertices are shared exactly
    so adjacent materials cannot leave gaps or overlap.
    """
    distances = [sum(a * b for a, b in zip(v[:3], plane[:3])) + plane[3] for v in poly]
    if min(distances) >= -1e-12:
        return poly, []
    if max(distances) <= 1e-12:
        return [], poly
    inside, outside = [], []
    for i, v in enumerate(poly):
        w = poly[(i + 1) % len(poly)]
        d, e = distances[i], distances[(i + 1) % len(poly)]
        if d >= 0:
            inside.append(v)
        if d <= 0:
            outside.append(v)
        if (d < 0 < e) or (e < 0 < d):
            # Canonical edge direction makes adjacent-face intersections agree.
            lo, hi = (v, w) if v < w else (w, v)
            dl = sum(a * b for a, b in zip(lo[:3], plane[:3])) + plane[3]
            dh = sum(a * b for a, b in zip(hi[:3], plane[:3])) + plane[3]
            t = dl / (dl - dh)
            cut = tuple(a + t * (b - a) for a, b in zip(lo, hi))
            inside.append(cut)
            outside.append(cut)
    return inside, outside


def partition(poly, rules):
    pending = [poly]
    for color, planes in rules:
        remainder = []
        for polygon in pending:
            # Most triangles are wholly outside a region: avoid unnecessary cuts.
            if any(
                max(
                    sum(a * b for a, b in zip(v[:3], plane[:3])) + plane[3]
                    for v in polygon
                )
                < -1e-12
                for plane in planes
            ):
                remainder.append(polygon)
                continue
            inside = polygon
            for plane in planes:
                inside, outside = split_polygon(inside, plane)
                if outside:
                    remainder.append(outside)
                if not inside:
                    break
            if inside:
                yield color, inside
        pending = remainder
    for polygon in pending:
        yield "white", polygon


def area(poly):
    import math

    total = 0
    for i in range(1, len(poly) - 1):
        a = [poly[i][k] - poly[0][k] for k in range(3)]
        b = [poly[i + 1][k] - poly[0][k] for k in range(3)]
        cross = (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )
        total += math.sqrt(sum(v * v for v in cross)) / 2
    return total


def build(refined=False):
    previous = json.loads((BODY / "color_regions.json").read_text()) if refined else {}
    output = previous.get("hosts", {})
    for host in (("torso_column", "chassis_link", "head_link") if refined else HOSTS):
        source = BODY / (
            f"{host}_refined.obj" if refined else ("torso_column_smooth.obj" if host == "torso_column" else f"{host}_hifi.obj")
        )
        overrides = (
            json.loads((BODY / "face_feature_colors.json").read_text())["faces"]
            if host == "head_link" and not refined
            else {}
        )
        rows = source.read_text().splitlines()
        positions = [
            tuple(map(float, r.split()[1:4])) for r in rows if r.startswith("v ")
        ]
        normals = [
            tuple(map(float, r.split()[1:4])) for r in rows if r.startswith("vn ")
        ]
        texcoords = [
            tuple(map(float, r.split()[1:])) for r in rows if r.startswith("vt ")
        ]
        groups = {name: [] for name in PALETTE}
        rules = regions(host)
        source_area = result_area = 0.0
        source_faces = 0
        object_color = None
        for row in rows:
            if row.startswith("o "):
                name = row.split()[1]
                object_color = name.split("_")[1] if name.startswith("Paint_") else None
            if not row.startswith("f "):
                continue
            poly = []
            for token in row.split()[1:]:
                indices = token.split("/")
                normal = normals[int(indices[2]) - 1]
                uv = texcoords[int(indices[1]) - 1] if indices[1] else ()
                poly.append(positions[int(indices[0]) - 1] + normal + uv)
            original_area = area(poly)
            feature_color = object_color or overrides.get(str(source_faces))
            pieces = (
                [(feature_color, poly)]
                if feature_color
                else list(partition(poly, rules))
            )
            split_area = sum(area(piece) for _, piece in pieces)
            assert abs(split_area - original_area) <= 1e-10 + original_area * 1e-8
            source_area += original_area
            result_area += split_area
            source_faces += 1
            for color, piece in pieces:
                for i in range(1, len(piece) - 1):
                    tri = (piece[0], piece[i], piece[i + 1])
                    if area(tri) > 1e-18:
                        groups[color].append(tri)
        counts = {}
        for color, faces in groups.items():
            if not faces:
                continue
            vertices = {}
            for face in faces:
                for v in face:
                    if v not in vertices:
                        vertices[v] = len(vertices) + 1
            out = [
                "# CAD surface clipped at exact material boundaries; interpolated normals."
            ]
            for prefix, start, end in (("v", 0, 3), ("vn", 3, 6), ("vt", 6, None)):
                for v in vertices:
                    values = v[start:end]
                    if values:
                        out.append(
                            prefix + " " + " ".join(format(x, ".17g") for x in values)
                        )
            for face in faces:
                out.append(
                    "f "
                    + " ".join(
                        f"{vertices[v]}/{vertices[v] if len(v) > 6 else ''}/{vertices[v]}"
                        for v in face
                    )
                )
            (BODY / f"{host}_{color}.obj").write_text("\n".join(out) + "\n")
            counts[color] = len(faces)
        output[host] = {
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "source_faces": source_faces,
            "source_area_m2": source_area,
            "result_area_m2": result_area,
            "faces": counts,
        }
        print(host, output[host], flush=True)
    (BODY / "color_regions.json").write_text(
        json.dumps(
            {
                "reference": "Robot back and middle part / IMG_1384–1391 (2026-09-10)" if refined else "d1.avif",
                "boundary_method": "surface triangle clipping",
                "palette": PALETTE,
                "hosts": output,
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--body-dir", type=Path, default=BODY)
    parser.add_argument("--refined", action="store_true", help="Use photo-refined rear/lift surfaces; preserve head regions")
    args = parser.parse_args()
    BODY = args.body_dir
    build(args.refined)
