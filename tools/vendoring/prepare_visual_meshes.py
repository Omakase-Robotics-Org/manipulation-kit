#!/usr/bin/env python3
"""Align paint to CAD components and simplify decorative torso front panels.

Requires numpy, scipy and trimesh. Source CAD and collision meshes are untouched.
"""

from pathlib import Path
import json
import sys

import numpy as np
import trimesh

from _kit_paths import description_d1

#: Where the hi-fi visual OBJs land. They are not in this repository (see
#: ../../LICENSE-STATUS.md) — this tool is what puts them there.
BODY = description_d1() / "meshes" / "body_hifi"

#: ``build_visual_colors`` stays INSIDE the package (it is data about the
#: robot's paint, and a test beside it exercises it), so make it importable
#: from here.
sys.path.insert(0, str(description_d1() / "tools"))


def components(mesh):
    return trimesh.graph.connected_components(
        mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), min_len=1
    )


def main():
    # Retain original face order so the color generator can address CAD faces.
    head = trimesh.load(BODY / "head_link_hifi.obj", process=False, force="mesh")
    welded = trimesh.Trimesh(vertices=head.vertices, faces=head.faces, process=False)
    welded.merge_vertices()
    overrides = {}
    bounds = {}
    for ids in components(welded):
        points = welded.vertices[welded.faces[ids].ravel()]
        lo, hi = points.min(axis=0), points.max(axis=0)
        color = None
        if (
            len(ids) > 500
            and lo[0] > 0.103
            and -0.043 < lo[1] < -0.040
            and hi[1] > -0.002
        ):
            color = "cyan"
        if (
            len(ids) > 500
            and 0.066 < lo[0] < 0.069
            and hi[0] > 0.122
            and 0.020 < hi[1] < 0.023
        ):
            color = "red"
        if color:
            for i in ids:
                overrides[str(int(i))] = color
            bounds[str(int(ids.min()))] = {
                "color": color,
                "faces": len(ids),
                "bounds": [lo.tolist(), hi.tolist()],
            }
    assert sum(v["color"] == "cyan" for v in bounds.values()) == 2
    assert sum(v["color"] == "red" for v in bounds.values()) == 1
    (BODY / "face_feature_colors.json").write_text(
        json.dumps({"components": bounds, "faces": overrides}, indent=2) + "\n"
    )

    torso = trimesh.load(BODY / "torso_column_hifi.obj", process=True, force="mesh")
    output = []
    changes = []
    front_panels = []
    for ids in components(torso):
        part = torso.submesh([ids], append=True, repair=False)
        lo, hi = part.bounds
        # Front upper/lower body shells: use their smooth convex envelope to
        # remove recessed speaker perforations, ribs and embossed details.
        shell = len(ids) > 5000 and 0.014 < lo[0] < 0.016 and hi[0] > 0.13
        insert = lo[0] > 0.075 and hi[2] < 0.42 and lo[2] > 0.20
        if insert:
            changes.append({"removed_insert_faces": len(ids)})
            continue
        if shell:
            front_panels.append(part)
        else:
            output.append(part)
    # A single continuous front envelope removes the chest/belly panel seam.
    assert len(front_panels) == 2
    smooth = trimesh.util.concatenate(front_panels).convex_hull
    # Convex envelope preserves overall dimensions. Smooth vertex normals
    # remove shading facets without displacing the new shell surface.
    # Preserve the real chest camera's recessed mounting area by
    # cutting an opening in the replacement shell.
    from build_visual_colors import partition, box, above

    planes = [above(0, 0.04)] + box(y=(-0.047, 0.049), z=(0.542, 0.581))
    verts, faces = [], []
    for face in smooth.faces:
        poly = [
            tuple(smooth.vertices[i]) + tuple(smooth.vertex_normals[i]) for i in face
        ]
        for color, p in partition(poly, [("opening", planes)]):
            if color == "opening":
                continue
            for k in range(1, len(p) - 1):
                tri = (p[0], p[k], p[k + 1])
                start = len(verts)
                verts.extend(v[:3] for v in tri)
                faces.append([start, start + 1, start + 2])
    smooth = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    changes.append(
        {
            "joined_front_panels": len(front_panels),
            "source_faces": sum(len(p.faces) for p in front_panels),
            "replacement_faces": len(smooth.faces),
        }
    )
    output.append(smooth)
    result = trimesh.util.concatenate(output)
    result.export(BODY / "torso_column_smooth.obj", include_normals=True)
    (BODY / "visual_preparation.json").write_text(
        json.dumps({"head_features": bounds, "torso_changes": changes}, indent=2) + "\n"
    )
    print(json.dumps({"head_features": bounds, "torso_changes": changes}, indent=2))


if __name__ == "__main__":
    main()
