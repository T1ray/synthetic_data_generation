"""Generate the project's small, dependency-free human proxy OBJ.

Run from any directory with ``python generate_low_poly_person.py``. The mesh is
authored in meters, stands on Z=0, and faces toward positive Y.
"""

from __future__ import annotations

import math
from pathlib import Path


vertices: list[tuple[float, float, float]] = []
faces: list[tuple[int, int, int]] = []


def add_mesh(points: list[tuple[float, float, float]], triangles: list[tuple[int, int, int]]) -> None:
    first = len(vertices)
    vertices.extend(points)
    faces.extend(tuple(first + index + 1 for index in face) for face in triangles)


def add_tapered_box(z0: float, z1: float, width0: float, width1: float,
                    depth0: float, depth1: float, x: float = 0.0, y: float = 0.0) -> None:
    points = [
        (x-width0/2, y-depth0/2, z0), (x+width0/2, y-depth0/2, z0),
        (x+width0/2, y+depth0/2, z0), (x-width0/2, y+depth0/2, z0),
        (x-width1/2, y-depth1/2, z1), (x+width1/2, y-depth1/2, z1),
        (x+width1/2, y+depth1/2, z1), (x-width1/2, y+depth1/2, z1),
    ]
    triangles = [
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
    ]
    add_mesh(points, triangles)


def add_cylinder_between(start: tuple[float, float, float],
                         end: tuple[float, float, float], radius: float,
                         sides: int = 10) -> None:
    a, b = (list(start), list(end))
    delta = [b[i] - a[i] for i in range(3)]
    length = math.sqrt(sum(value * value for value in delta))
    axis = [value / length for value in delta]
    reference = [0.0, 1.0, 0.0]
    u = [axis[1]*reference[2]-axis[2]*reference[1],
         axis[2]*reference[0]-axis[0]*reference[2],
         axis[0]*reference[1]-axis[1]*reference[0]]
    u_length = math.sqrt(sum(value * value for value in u))
    u = [value / u_length for value in u]
    v = [axis[1]*u[2]-axis[2]*u[1],
         axis[2]*u[0]-axis[0]*u[2],
         axis[0]*u[1]-axis[1]*u[0]]
    points = []
    for center in (a, b):
        for side in range(sides):
            angle = 2 * math.pi * side / sides
            points.append(tuple(center[i] + radius * (math.cos(angle)*u[i] + math.sin(angle)*v[i])
                                for i in range(3)))
    points.extend((tuple(a), tuple(b)))
    bottom, top = 2*sides, 2*sides+1
    triangles = []
    for side in range(sides):
        next_side = (side+1) % sides
        triangles.extend(((side, next_side, sides+next_side),
                          (side, sides+next_side, sides+side),
                          (bottom, next_side, side),
                          (top, sides+side, sides+next_side)))
    add_mesh(points, triangles)


def add_ellipsoid(center: tuple[float, float, float],
                  radii: tuple[float, float, float], sides: int = 12,
                  rings: int = 6) -> None:
    points = [(center[0], center[1], center[2]+radii[2])]
    for ring in range(1, rings):
        phi = math.pi * ring / rings
        for side in range(sides):
            theta = 2 * math.pi * side / sides
            points.append((center[0]+radii[0]*math.sin(phi)*math.cos(theta),
                           center[1]+radii[1]*math.sin(phi)*math.sin(theta),
                           center[2]+radii[2]*math.cos(phi)))
    points.append((center[0], center[1], center[2]-radii[2]))
    south = len(points)-1
    triangles = []
    for side in range(sides):
        next_side = (side+1) % sides
        triangles.append((0, 1+next_side, 1+side))
        for ring in range(rings-2):
            a = 1+ring*sides+side
            b = 1+ring*sides+next_side
            c = 1+(ring+1)*sides+side
            d = 1+(ring+1)*sides+next_side
            triangles.extend(((a, b, d), (a, d, c)))
        last = 1+(rings-2)*sides
        triangles.append((south, last+side, last+next_side))
    add_mesh(points, triangles)


def main() -> None:
    # Distinct pieces intentionally leave visible gaps between arms and torso
    # and between the two legs when seen from the front.
    add_tapered_box(0.82, 1.42, 0.30, 0.42, 0.22, 0.25)  # torso
    add_tapered_box(0.72, 0.88, 0.32, 0.30, 0.22, 0.22)  # pelvis
    add_cylinder_between((0.0, 0.0, 1.40), (0.0, 0.0, 1.52), 0.065)  # neck
    add_ellipsoid((0.0, 0.0, 1.63), (0.15, 0.13, 0.18))  # head

    for sign in (-1, 1):
        add_cylinder_between((sign*0.25, 0.0, 1.35),
                             (sign*0.42, 0.0, 0.88), 0.075)  # arm
        add_ellipsoid((sign*0.42, 0.0, 0.85), (0.065, 0.065, 0.08), 10, 5)  # hand
        add_cylinder_between((sign*0.10, 0.0, 0.75),
                             (sign*0.14, 0.0, 0.12), 0.09)  # leg
        add_tapered_box(0.0, 0.13, 0.19, 0.18, 0.30, 0.23,
                        x=sign*0.15, y=0.05)  # foot

    output = Path(__file__).with_name("low_poly_person.obj")
    with output.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# Original low-poly human proxy; units: meters; CC0.\n")
        stream.write("# Generated by generate_low_poly_person.py.\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in faces:
            stream.write(f"f {a} {b} {c}\n")
    print(f"{output}: {len(vertices)} vertices, {len(faces)} triangles")


if __name__ == "__main__":
    main()
