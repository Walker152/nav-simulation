#!/usr/bin/env python3
"""Rebuild this house's local SDF, ramp meshes and floor maps (stdlib only)."""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
COLORS = {
    "wall": "0.86 0.88 0.85 1", "floor": "0.68 0.55 0.39 1",
    "upper": "0.76 0.65 0.48 1", "wood": "0.38 0.21 0.10 1",
    "white": "0.93 0.92 0.86 1", "fabric": "0.22 0.46 0.50 1",
    "bed": "0.40 0.55 0.72 1", "metal": "0.16 0.20 0.23 1",
    "ramp": "0.43 0.49 0.52 1",
}


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def numbers(values):
    return " ".join(f"{v:.8g}" for v in values)


def write_xml(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def make_house():
    root = ET.Element("sdf", version="1.7")
    model = element(root, "model", name="home_indoor")
    element(model, "static", "true")
    boxes = []

    def body(name, pose, geometry, color):
        link = element(model, "link", name=name)
        element(link, "pose", numbers(pose))
        collision = element(link, "collision", name=f"{name}_collision")
        collision.append(deepcopy(geometry))
        ode = element(element(element(collision, "surface"), "friction"), "ode")
        element(ode, "mu", 1.0)
        element(ode, "mu2", 1.0)
        visual = element(link, "visual", name=f"{name}_visual")
        visual.append(deepcopy(geometry))
        material = element(visual, "material")
        element(material, "ambient", COLORS[color])
        element(material, "diffuse", COLORS[color])

    def box(name, xyz, size, color="wall", roll=0):
        geometry = ET.Element("geometry")
        element(element(geometry, "box"), "size", numbers(size))
        body(name, (*xyz, roll, 0, 0), geometry, color)
        boxes.append((name, xyz, size, color, roll))

    def partition(name, x, y0, y1, z, door_y, width):
        # Framing is OUTSIDE the requested clear opening; no door threshold.
        for side, a, b in (("south", y0, door_y-width/2-0.08),
                           ("north", door_y+width/2+0.08, y1)):
            box(f"{name}_{side}", (x, (a+b)/2, z+1.2), (0.18, b-a, 2.4))
        box(f"{name}_lintel", (x, door_y, z+2.30), (0.18, width+0.16, 0.20))
        for side in (-1, 1):
            box(f"{name}_jamb_{side}", (x, door_y+side*(width/2+0.04), z+1.05),
                (0.24, 0.08, 2.1), "wood")
        box(f"{name}_frame_top", (x, door_y, z+2.15), (0.24, width+0.16, 0.1), "wood")

    def table(name, x, y, z, sx=1.6, sy=0.9, height=0.76):
        box(f"{name}_top", (x, y, z+height-0.04), (sx, sy, 0.08), "wood")
        for ix in (-1, 1):
            for iy in (-1, 1):
                box(f"{name}_leg_{ix}_{iy}", (x+ix*(sx/2-0.09), y+iy*(sy/2-0.09),
                    z+(height-0.08)/2), (0.08, 0.08, height-0.08), "metal")

    def chair(name, x, y, z):
        box(f"{name}_seat", (x, y, z+0.44), (0.46, 0.46, 0.10), "fabric")
        box(f"{name}_back", (x, y+0.20, z+0.70), (0.46, 0.08, 0.5), "fabric")
        for ix in (-1, 1):
            for iy in (-1, 1):
                box(f"{name}_leg_{ix}_{iy}", (x+ix*0.17, y+iy*0.17, z+0.2),
                    (0.05, 0.05, 0.4), "wood")

    def bed(name, x, y, z):
        box(f"{name}_base", (x, y, z+0.22), (1.8, 2.2, 0.44), "wood")
        box(f"{name}_mattress", (x, y, z+0.51), (1.76, 2.16, 0.18), "bed")
        box(f"{name}_head", (x, y+1.07, z+0.6), (1.9, 0.12, 1.2), "wood")
        for dx in (-0.45, 0.45):
            box(f"{name}_pillow_{dx}", (x+dx, y+0.75, z+0.65), (0.65, 0.40, 0.1), "white")

    def sofa(name, x, y, z):
        box(f"{name}_base", (x, y, z+0.24), (0.95, 2.5, 0.48), "fabric")
        box(f"{name}_back", (x-0.40, y, z+0.64), (0.18, 2.5, 0.80), "fabric")
        for dy in (-1.15, 1.15):
            box(f"{name}_arm_{dy}", (x, y+dy, z+0.48), (0.95, 0.20, 0.40), "fabric")

    box("floor_ground", (6.6, 5.4, -0.1), (13.6, 11.2, 0.2), "floor")
    box("floor_upper", (5, 4.5, 2.5), (10, 9, 0.2), "upper")
    for floor, z in enumerate((0.0, 2.6)):
        p = f"f{floor}"
        box(f"{p}_west", (0, 4.5, z+1.2), (0.18, 9, 2.4))
        box(f"{p}_north", (5, 9, z+1.2), (10, 0.18, 2.4))
        box(f"{p}_south", (5, 0, z+1.2), (10, 0.18, 2.4))
        partition(f"{p}_ramp_door", 10, 0, 9, z, 0.8, 1.1)
        for x in (4.2, 5.8):
            for j, door_y in enumerate((2.25, 6.75)):
                partition(f"{p}_room_{x}_{j}", x, j*4.5, (j+1)*4.5, z,
                          door_y, 0.9 if x == 4.2 else 1.0)
        for x in (2.1, 7.9):
            box(f"{p}_cross_wall_{x}", (x, 4.5, z+1.2), (4.2, 0.18, 2.4))
        box(f"{p}_hall_console", (5, 8.65, z+0.45), (1.0, 0.45, 0.9), "wood")

    sofa("living_sofa", 0.8, 2.1, 0)
    table("coffee", 2.35, 2.15, 0, 0.65, 1.1, 0.42)
    box("tv_stand", (2.1, 4.1, 0.3), (1.8, 0.5, 0.6), "wood")
    box("television", (2.1, 4.12, 1.1), (1.4, 0.12, 0.8), "metal")
    bed("guest_bed", 1.3, 6.8, 0)
    box("guest_wardrobe", (3.3, 8.45, 1), (1.2, 0.7, 2), "wood")
    table("dining", 7.65, 2.75, 0, 1.3, 0.8)
    for i, xy in enumerate(((6.65, 2.75), (8.65, 2.75), (7.65, 1.85), (7.65, 3.65))):
        chair(f"dining_chair_{i}", *xy, 0)
    box("kitchen_counter_north", (7.6, 8.45, 0.46), (2.9, 0.85, 0.92), "white")
    box("kitchen_counter_east", (9.45, 7.8, 0.46), (0.85, 1.7, 0.92), "white")
    box("fridge", (9.4, 5.35, 0.95), (0.9, 0.85, 1.9), "white")
    box("sink", (7.5, 8.45, 0.93), (0.65, 0.55, 0.04), "metal")
    bed("master_bed", 1.3, 2.1, 2.6)
    box("master_wardrobe", (2.1, 4.05, 3.6), (2.7, 0.65, 2), "white")
    bed("second_bed", 1.3, 6.8, 2.6)
    table("second_desk", 3.2, 8.35, 2.6, 1.3, 0.7)
    table("study_desk", 7.6, 3.8, 2.6, 1.8, 0.7)
    chair("study_chair", 7.6, 2.8, 2.6)
    box("bookcase", (9.5, 2.8, 3.6), (0.55, 1.8, 2), "wood")
    sofa("lounge_sofa", 8.8, 6.4, 2.6)
    table("lounge_coffee", 7.25, 6.45, 2.6, 0.65, 1.1, 0.42)
    box("lounge_cabinet", (8, 8.55, 3.05), (2.4, 0.55, 0.9), "white")

    # The top surfaces exactly meet z=0 / 1.3 / 2.6, without a box-edge lip.
    (HERE / "meshes").mkdir(exist_ok=True)
    for name, x0, z0, z1 in (("ramp_lower", 10.2, 0, 1.3),
                              ("ramp_upper", 11.8, 2.6, 1.3)):
        vertices = [(x, y, z-d) for d in (0, 0.16)
                    for x, y, z in ((x0, 1.6, z0), (x0+1.2, 1.6, z0),
                                    (x0+1.2, 9.1, z1), (x0, 9.1, z1))]
        faces = ((0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
                 (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
                 (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0))
        lines = [f"solid {name}"]
        for face in faces:
            a, b, c = [vertices[i] for i in face]
            u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            length = math.sqrt(sum(q*q for q in n))
            lines += [f"  facet normal {numbers(q/length for q in n)}", "    outer loop"]
            lines += [f"      vertex {numbers(vertices[i])}" for i in face]
            lines += ["    endloop", "  endfacet"]
        lines += [f"endsolid {name}"]
        (HERE / "meshes" / f"{name}.stl").write_text("\n".join(lines)+"\n")
        geometry = ET.Element("geometry")
        element(element(geometry, "mesh"), "uri", f"model://home_indoor/meshes/{name}.stl")
        body(name, (0, 0, 0, 0, 0, 0), geometry, "ramp")
        angle = math.atan2(z1-z0, 7.5)
        for i, x in enumerate((x0-0.05, x0+1.25)):
            box(f"{name}_guard_{i}", (x, 5.35, (z0+z1)/2+0.48),
                (0.1, math.hypot(7.5, z1-z0), 0.85), "metal", angle)
    box("landing_middle", (11.6, 9.9, 1.2), (2.8, 1.6, 0.2), "ramp")
    box("landing_upper", (11.6, 0.8, 2.5), (3.2, 1.6, 0.2), "upper")
    box("middle_end_guard", (11.6, 10.75, 1.8), (3, 0.1, 1), "metal")
    for x in (10.15, 13.05):
        box(f"middle_side_guard_{x}", (x, 9.9, 1.8), (0.1, 1.6, 1), "metal")
    box("upper_east_guard", (13.1, 0.8, 3.1), (0.1, 1.6, 1), "metal")
    box("upper_south_guard", (11.6, 0.05, 3.1), (3.2, 0.1, 1), "metal")
    # Close the unused half of the upper landing edge above the lower flight.
    box("upper_void_guard", (10.85, 1.55, 3.1), (1.7, 0.1, 1), "metal")
    write_xml(root, HERE / "model.sdf")
    return boxes


def write_maps_and_plan(boxes):
    # Union of obstacle footprints intersecting robot height 5–120 cm.
    # Ramps are unknown: a planar map cannot encode their changing elevation.
    width, height, resolution = 268, 220, 0.05
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1160" height="530" viewBox="0 0 1160 530">',
           '<rect width="1160" height="530" fill="#faf8f2"/>',
           '<g font-family="sans-serif" fill="#253039">']
    rooms = (("Living", "Guest bedroom", "Dining", "Kitchen"),
             ("Main bedroom", "Bedroom", "Study", "Lounge"))
    for floor, z in enumerate((0.0, 2.6)):
        stem = "home_indoor" + ("_upper" if floor else "")
        obstacles = [(xyz, size) for _, xyz, size, _, roll in boxes if roll == 0
                     and xyz[2]+size[2]/2 > z+0.05 and xyz[2]-size[2]/2 < z+1.2]
        pixels = bytearray()
        for row in range(height):
            y = (height-row-0.5)*resolution
            for column in range(width):
                x = (column+0.5)*resolution
                value = 254 if ((x < 10 and y < 9) or (10 <= x < 13.2 and y < 1.6)) else 205
                if any(abs(x-c[0]) <= s[0]/2 and abs(y-c[1]) <= s[1]/2 for c, s in obstacles):
                    value = 0
                pixels.append(value)
        (PACKAGE / "maps" / f"{stem}.pgm").write_bytes(
            f"P5\n{width} {height}\n255\n".encode()+pixels)
        (PACKAGE / "maps" / f"{stem}.yaml").write_text(
            f"image: {stem}.pgm\nmode: trinary\nresolution: {resolution}\n"
            "origin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        ox, oy, scale = 35+floor*580, 475, 31
        svg.append(f'<text x="{ox}" y="35" font-size="22">{floor+1}F · z = {z:.1f} m</text>')
        svg.append(f'<rect x="{ox}" y="{oy-10.8*scale}" width="{13.2*scale}" height="{10.8*scale}" fill="#e8dfca"/>')
        for name, c, s, color, roll in boxes:
            if roll or c[2]+s[2]/2 <= z+0.05 or c[2]-s[2]/2 >= z+1.2:
                continue
            rgb = [round(float(v)*255) for v in COLORS[color].split()[:3]]
            svg.append(f'<rect x="{ox+(c[0]-s[0]/2)*scale:.2f}" y="{oy-(c[1]+s[1]/2)*scale:.2f}" '
                       f'width="{s[0]*scale:.2f}" height="{s[1]*scale:.2f}" fill="rgb{tuple(rgb)}"/>')
        for i, label in enumerate(rooms[floor]):
            x, y = (2.1 if i < 2 else 7.9), (0.6 if i%2 == 0 else 5.1)
            svg.append(f'<text x="{ox+x*scale}" y="{oy-y*scale}" font-size="12" text-anchor="middle">{label}</text>')
        for x, label in ((10.8, "0 → 1.3 m"), (12.4, "2.6 ← 1.3 m")):
            svg.append(f'<rect x="{ox+(x-0.6)*scale}" y="{oy-9.1*scale}" width="{1.2*scale}" height="{7.5*scale}" fill="#a6b4b9" stroke="#4b5f66"/>')
            svg.append(f'<text x="{ox+x*scale}" y="{oy-5.8*scale}" font-size="11" text-anchor="middle" transform="rotate(-90 {ox+x*scale} {oy-5.8*scale})">{label}</text>')
        svg.append(f'<text x="{ox}" y="510" font-size="13">10 × 9 m home + switchback ramp · doors 0.9 / 1.0 / 1.1 m</text>')
        if floor == 0:
            svg.append(f'<circle cx="{ox+5*scale}" cy="{oy-1.2*scale}" r="8" fill="#ce5836"/>')
    svg += ["</g></svg>"]
    (HERE / "floor_plan.svg").write_text("\n".join(svg)+"\n")


def main():
    boxes = make_house()
    write_maps_and_plan(boxes)
    config = ET.Element("model")
    element(config, "name", "Home indoor - two floors and ramp")
    element(config, "version", "1.0")
    element(config, "sdf", "model.sdf", version="1.7")
    author = element(config, "author")
    element(author, "name", "Naturewill contributors")
    element(config, "description", "Original two-storey furnished home with a 9.83 degree switchback ramp.")
    write_xml(config, HERE / "model.config")
    # Reuse the established Fortress plugin contract, without copying another arena.
    root = ET.parse(PACKAGE / "resource/worlds/rmuc_2026_world.sdf").getroot()
    world = root.find("world")
    world.find("include/uri").text = "model://home_indoor"
    world.find("include/name").text = "home_indoor"
    physics = element(world, "physics", name="1ms", type="ignored")
    element(physics, "max_step_size", 0.001)
    element(physics, "real_time_factor", 1.0)
    scene = element(world, "scene")
    element(scene, "ambient", "0.65 0.65 0.65 1")
    element(scene, "background", "0.85 0.89 0.92 1")
    write_xml(root, PACKAGE / "resource/worlds/home_indoor_world.sdf")
    print(f"Generated home_indoor: {len(boxes)} boxes, 2 ramp meshes, 2 floor maps.")


if __name__ == "__main__":
    main()
