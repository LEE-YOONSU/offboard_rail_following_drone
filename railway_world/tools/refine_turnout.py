#!/usr/bin/env python3
"""Make the turnout transition continuous with the detailed main track."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path


MODEL = Path(__file__).resolve().parents[1] / "models/railway_turnout/model.sdf"
GAUGE_HALF = 0.7175


def pose_values(element):
    return [float(value) for value in element.findtext("pose").split()]


def sleeper_visual(index, x, branch_y):
    centre_y = branch_y * 0.5
    length = 2.40 + abs(branch_y)
    visual = ET.Element("visual", {"name": f"transition_sleeper_{index:02d}"})
    ET.SubElement(visual, "pose").text = f"{x:.3f} {centre_y:.4f} 0.235 0 0 0"
    geometry = ET.SubElement(visual, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    ET.SubElement(mesh, "uri").text = "model://pc_sleeper/meshes/pc_sleeper_concrete.obj"
    ET.SubElement(mesh, "scale").text = f"1 {length / 2.4:.6f} 1"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.47 0.47 0.45 1"
    ET.SubElement(material, "diffuse").text = "0.68 0.68 0.65 1"
    pbr = ET.SubElement(material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    ET.SubElement(metal, "roughness").text = "0.92"
    ET.SubElement(metal, "metalness").text = "0"
    return visual


def branch_offset(x):
    control = [
        (-10.0, 0.0), (-7.0, -0.040), (-1.0, -0.215), (5.0, -0.625),
        (11.0, -1.325), (17.0, -2.300), (23.0, -3.500),
    ]
    for (x0, y0), (x1, y1) in zip(control, control[1:]):
        if x <= x1:
            ratio = (x - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    return control[-1][1]


def main():
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(MODEL, parser=parser)
    root = tree.getroot()
    model = root.find("model")
    link = model.find("link")

    ballast_centres = {}
    for child in link:
        name = child.get("name", "")
        if name.startswith("ballast_") and name[8:].isdigit():
            ballast_centres[int(name[8:])] = pose_values(child)

    # Remove the rectangular ballast blocks that cut across the existing main bed.
    for child in list(link):
        name = child.get("name", "")
        if name in {f"ballast_{i}" for i in range(5)} | {f"ballast_collision_{i}" for i in range(5)}:
            link.remove(child)
        elif name == "ballast_transition" or name.startswith("transition_sleeper_"):
            link.remove(child)

    transition = ET.Element("visual", {"name": "ballast_transition"})
    geometry = ET.SubElement(transition, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    ET.SubElement(mesh, "uri").text = "model://railway_turnout/meshes/ballast_transition.obj"
    material = ET.SubElement(transition, "material")
    ET.SubElement(material, "ambient").text = "0.38 0.38 0.36 1"
    ET.SubElement(material, "diffuse").text = "0.60 0.59 0.56 1"
    pbr = ET.SubElement(material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    ET.SubElement(metal, "albedo_map").text = "model://track_12m/materials/textures/ballast_albedo.png"
    ET.SubElement(metal, "roughness").text = "0.98"
    ET.SubElement(metal, "metalness").text = "0"
    link.insert(0, transition)

    # Correct the right-hand rail, which was accidentally duplicated at the left rail position.
    for index, centre in ballast_centres.items():
        cx, cy, _, _, _, yaw = centre
        right_x = cx + GAUGE_HALF * math.sin(yaw)
        right_y = cy - GAUGE_HALF * math.cos(yaw)
        for prefix in ("rail_r_collision_", "rail_r_foot_", "rail_r_web_", "rail_r_head_"):
            element = link.find(f"*[@name='{prefix}{index}']")
            if element is None:
                continue
            pose = pose_values(element)
            pose[0], pose[1] = right_x, right_y
            element.find("pose").text = " ".join(f"{value:.6f}" for value in pose)

    # Replace the siding's simplified three-box rails with the same 50N profile
    # mesh used by every main-line module. Collision boxes remain lightweight.
    rail_segments = []
    for element in list(link):
        name = element.get("name", "")
        if not name.startswith("rail_l_head_"):
            continue
        index = int(name.rsplit("_", 1)[1])
        left_pose = pose_values(element)
        yaw = left_pose[5]
        centre_x = left_pose[0] + GAUGE_HALF * math.sin(yaw)
        centre_y = left_pose[1] - GAUGE_HALF * math.cos(yaw)
        length = float(element.findtext("./geometry/box/size").split()[0])
        rail_segments.append((index, centre_x, centre_y, yaw, length))

    for element in list(link):
        name = element.get("name", "")
        if name.startswith(("rail_l_foot_", "rail_l_web_", "rail_l_head_", "rail_r_foot_", "rail_r_web_", "rail_r_head_")):
            link.remove(element)

    for index, centre_x, centre_y, yaw, length in rail_segments:
        for side, offset in (("left", GAUGE_HALF), ("right", -GAUGE_HALF)):
            x = centre_x - offset * math.sin(yaw)
            y = centre_y + offset * math.cos(yaw)
            visual = ET.Element("visual", {"name": f"branch_rail_{side}_{index}"})
            ET.SubElement(visual, "pose").text = f"{x:.6f} {y:.6f} 0.459 0 0 {yaw:.6f}"
            geometry = ET.SubElement(visual, "geometry")
            mesh = ET.SubElement(geometry, "mesh")
            ET.SubElement(mesh, "uri").text = "model://track_12m/meshes/rail_profile_12m.obj"
            ET.SubElement(mesh, "scale").text = f"{length / 12.0:.7f} 1 1"
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = "0.13 0.09 0.065 1"
            ET.SubElement(material, "diffuse").text = "0.31 0.22 0.16 1"
            ET.SubElement(material, "specular").text = "0.30 0.25 0.20 1"
            pbr = ET.SubElement(material, "pbr")
            metal = ET.SubElement(pbr, "metal")
            ET.SubElement(metal, "roughness").text = "0.48"
            ET.SubElement(metal, "metalness").text = "0.72"
            link.append(visual)

    # Match the polished running surface and gauge-face wear used on the main rail.
    for element in list(link):
        name = element.get("name", "")
        if name.startswith(("branch_running_band_", "branch_gauge_wear_")):
            link.remove(element)
    for rail in list(link):
        name = rail.get("name", "")
        if not name.startswith("branch_rail_"):
            continue
        side = "left" if "_left_" in name else "right"
        index = name.rsplit("_", 1)[1]
        pose = pose_values(rail)
        length = float(rail.findtext("./geometry/mesh/scale").split()[0]) * 12.0
        inward = -1.0 if side == "left" else 1.0

        band = ET.Element("visual", {"name": f"branch_running_band_{side}_{index}"})
        offset = inward * 0.0038
        bx = pose[0] - offset * math.sin(pose[5])
        by = pose[1] + offset * math.cos(pose[5])
        ET.SubElement(band, "pose").text = f"{bx:.6f} {by:.6f} 0.6155 0 0 {pose[5]:.6f}"
        geometry = ET.SubElement(band, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = f"{length:.5f} 0.058 0.006"
        material = ET.SubElement(band, "material")
        ET.SubElement(material, "ambient").text = "0.42 0.43 0.43 1"
        ET.SubElement(material, "diffuse").text = "0.66 0.67 0.67 1"
        ET.SubElement(material, "specular").text = "0.92 0.92 0.92 1"
        pbr = ET.SubElement(material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        ET.SubElement(metal, "roughness").text = "0.18"
        ET.SubElement(metal, "metalness").text = "0.92"
        link.append(band)

        wear = ET.Element("visual", {"name": f"branch_gauge_wear_{side}_{index}"})
        offset = inward * 0.034
        wx = pose[0] - offset * math.sin(pose[5])
        wy = pose[1] + offset * math.cos(pose[5])
        ET.SubElement(wear, "pose").text = f"{wx:.6f} {wy:.6f} 0.586 0 0 {pose[5]:.6f}"
        geometry = ET.SubElement(wear, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = f"{length:.5f} 0.004 0.034"
        material = ET.SubElement(wear, "material")
        ET.SubElement(material, "diffuse").text = "0.47 0.48 0.48 1"
        ET.SubElement(material, "specular").text = "0.72 0.72 0.72 1"
        link.append(wear)

    # Use the exact main-line trapezoidal ballast asset on the separated siding.
    # The custom wedge remains only where both beds merge through the switch.
    for element in link:
        name = element.get("name", "")
        if not (name.startswith("ballast_") and name[8:].isdigit()):
            continue
        size = element.findtext("./geometry/box/size")
        if size is not None:
            length = float(size.split()[0])
            geometry = element.find("geometry")
            geometry.clear()
            mesh = ET.SubElement(geometry, "mesh")
            ET.SubElement(mesh, "uri").text = "model://track_12m/meshes/ballast_bed.obj"
            ET.SubElement(mesh, "scale").text = f"{length / 12.0:.7f} 1 1"
            pose = pose_values(element)
            pose[2] = 0.0
            element.find("pose").text = " ".join(f"{value:.6f}" for value in pose)
        material = element.find("material")
        ambient = material.find("ambient")
        if ambient is None:
            ambient = ET.SubElement(material, "ambient")
        ambient.text = "0.38 0.38 0.36 1"
        material.find("diffuse").text = "0.60 0.59 0.56 1"

    # Replace misaligned duplicate sleepers with bearers aligned to the 600 mm main-track rhythm.
    for include in list(model.findall("include")):
        name = include.findtext("name", "")
        if name.startswith("branch_sleeper_") and int(name.rsplit("_", 1)[1]) <= 50:
            model.remove(include)
    insert_at = 1
    for index, step in enumerate(range(0, 34)):
        x = -5.7 + step * 0.6
        link.insert(insert_at + index, sleeper_visual(index, x, branch_offset(x)))

    # Once the track centres are sufficiently separated, change gradually to
    # independent 2.4 m branch sleepers instead of one implausibly long bearer.
    for index in range(41, 51):
        include = ET.Element("include")
        ET.SubElement(include, "uri").text = "model://pc_sleeper"
        ET.SubElement(include, "name").text = f"branch_sleeper_{index:03d}"
        x = 14.6 + (index - 41) * 0.6
        y = -1.86 - (index - 41) * 0.11
        ET.SubElement(include, "pose").text = f"{x:.4f} {y:.4f} 0.235 0 0 -0.181320"
        model.append(include)

    # Match all remaining branch sleepers to the main-track vertical datum.
    for include in model.findall("include"):
        if include.findtext("name", "").startswith("branch_sleeper_"):
            pose = [float(value) for value in include.findtext("pose").split()]
            pose[2] = 0.235
            include.find("pose").text = " ".join(f"{value:.6f}" for value in pose)

    # Use the same weathered steel palette as the detailed main-line rail.
    for element in link:
        name = element.get("name", "")
        if not name.startswith(("rail_l_", "rail_r_", "point_blade", "check_rail")):
            continue
        material = element.find("material")
        if material is None:
            continue
        ambient = material.find("ambient")
        diffuse = material.find("diffuse")
        if ambient is not None:
            ambient.text = "0.13 0.09 0.065 1"
        if diffuse is not None:
            diffuse.text = "0.31 0.22 0.16 1"
        roughness = material.find("./pbr/metal/roughness")
        metalness = material.find("./pbr/metal/metalness")
        if roughness is not None:
            roughness.text = "0.48"
        if metalness is not None:
            metalness.text = "0.72"

    ET.indent(tree, space="  ")
    tree.write(MODEL, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
