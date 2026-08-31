#!/usr/bin/env python3
"""Triangulate generated OBJ assets and write explicit flat face normals."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path


def normal(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-12:
        raise ValueError("degenerate triangle")
    return nx / length, ny / length, nz / length


def signed_area(points):
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )


def point_in_triangle(p, a, b, c, orientation):
    def edge(u, v, q):
        return (v[0] - u[0]) * (q[1] - u[1]) - (v[1] - u[1]) * (q[0] - u[0])

    eps = 1e-12
    return all(orientation * edge(u, v, p) >= -eps for u, v in ((a, b), (b, c), (c, a)))


def triangulate_polygon(face, vertices):
    if len(face) == 3:
        return [tuple(face)]

    polygon_normal = normal(vertices[face[0] - 1], vertices[face[1] - 1], vertices[face[2] - 1])
    drop_axis = max(range(3), key=lambda axis: abs(polygon_normal[axis]))
    axes = [axis for axis in range(3) if axis != drop_axis]
    projected = [(vertices[index - 1][axes[0]], vertices[index - 1][axes[1]]) for index in face]
    orientation = 1 if signed_area(projected) > 0 else -1
    remaining = list(range(len(face)))
    triangles = []

    while len(remaining) > 3:
        found_ear = False
        for pos, current in enumerate(remaining):
            previous = remaining[pos - 1]
            following = remaining[(pos + 1) % len(remaining)]
            a, b, c = projected[previous], projected[current], projected[following]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if orientation * cross <= 1e-12:
                continue
            if any(
                point_in_triangle(projected[other], a, b, c, orientation)
                for other in remaining
                if other not in (previous, current, following)
            ):
                continue
            triangles.append((face[previous], face[current], face[following]))
            del remaining[pos]
            found_ear = True
            break
        if not found_ear:
            raise ValueError(f"cannot triangulate face with {len(face)} vertices")

    triangles.append(tuple(face[index] for index in remaining))
    return triangles


def rebuild(path: Path, flip_faces: bool = False):
    source = path.read_text(encoding="utf-8").splitlines()
    vertices = []
    faces = []
    header = []

    for line in source:
        if line.startswith("v "):
            vertices.append(tuple(map(float, line.split()[1:4])))
        elif line.startswith("f "):
            face = [int(token.split("/")[0]) for token in line.split()[1:]]
            faces.append(list(reversed(face)) if flip_faces else face)
        elif not line.startswith("vn "):
            header.append(line)

    triangles = [triangle for face in faces for triangle in triangulate_polygon(face, vertices)]
    normals = [normal(*(vertices[index - 1] for index in triangle)) for triangle in triangles]

    output = header + [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices]
    output += [f"vn {x:.8f} {y:.8f} {z:.8f}" for x, y, z in normals]
    output += [
        "f " + " ".join(f"{vertex}//{normal_index}" for vertex in triangle)
        for normal_index, triangle in enumerate(triangles, 1)
    ]
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text("\n".join(output) + "\n", encoding="utf-8")
    os.replace(temporary, path)
    print(f"{path}: {len(faces)} polygons -> {len(triangles)} triangles, {len(normals)} normals")


if __name__ == "__main__":
    arguments = sys.argv[1:]
    flip = bool(arguments and arguments[0] == "--flip")
    for filename in arguments[1:] if flip else arguments:
        rebuild(Path(filename), flip_faces=flip)
