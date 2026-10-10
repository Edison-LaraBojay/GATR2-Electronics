#!/usr/bin/env python3
"""Convert the archived Override field.xml into spectaGATR field JSON.

One-time data conversion; the viewer loads only the committed JSON.

    python spectaGATR/fields/convert_override.py field.xml [override.json]

The XML input must be supplied separately; it is not part of a fresh checkout.
Get it from Git history:

    git show 10c568e:pi/naviGATR/config/override/field.xml > field.xml
Output defaults to spectaGATR/fields/override.json. Display geometry only: perimeter, wall
features, floor tape and goal bodies. Tag mounts, collision boxes and
planning data are not carried over.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "override.json"


def num(el, name, default=0.0):
    v = el.get(name)
    return float(v) if v is not None else default


def convert(xml_path):
    root = ET.parse(xml_path).getroot()
    dims = root.find("Dimensions")
    if dims is None:
        raise SystemExit("field.xml has no Dimensions element")
    out = {
        "schema": "spectagatr-field/1",
        "name": "override",
        "title": "V5RC Override 2026-2027 (nominal)",
        "source": "converted from pi/naviGATR/config/override/field.xml at commit 10c568e "
                  "(revision %s) by convert_override.py; %s" % (root.get("revision", "?"),
                                                              dims.get("source", "")),
        "dimensions": {
            "inside_x_m": num(dims, "inside_x_m"),
            "inside_y_m": num(dims, "inside_y_m"),
            "wall_height_m": num(dims, "wall_height_m"),
            "wall_thickness_m": num(dims, "wall_thickness_m"),
            "tile_m": num(dims, "tile_m"),
        },
        "walls": {
            "x_min": {"label": "red wall", "color": "#ff8a8a"},
            "x_max": {"label": "blue wall", "color": "#8ad0ff"},
            "y_min": {"label": "audience", "color": "#d8dde6"},
        },
        "objects": [],
    }
    objects = out["objects"]
    for f in root.findall("Feature"):
        kind = f.get("kind")
        obj = {
            "id": f.get("id"),
            "kind": kind,
            "x_m": num(f, "x_m"),
            "y_m": num(f, "y_m"),
        }
        if kind == "box":
            obj["z_m"] = num(f, "z_m")
        elif kind != "tape":
            raise SystemExit("unknown Feature kind %r" % kind)
        obj["size_x_m"] = num(f, "size_x_m")
        obj["size_y_m"] = num(f, "size_y_m")
        if kind == "box":
            obj["size_z_m"] = num(f, "size_z_m")
        obj["yaw_deg"] = num(f, "yaw_deg")
        obj["color"] = f.get("color", "#999999")
        objects.append(obj)
    for lm in root.findall("Landmark"):
        pose = lm.find("NominalPose")
        vis = lm.find("Visual")
        if pose is None or vis is None or vis.get("shape") != "octagonal_prism":
            raise SystemExit("landmark %s: expected NominalPose and an octagonal_prism Visual"
                             % lm.get("id"))
        objects.append({
            "id": lm.get("id"),
            "kind": "octagon",
            "x_m": num(pose, "x_m"),
            "y_m": num(pose, "y_m"),
            "yaw_deg": num(pose, "heading_deg"),
            "height_m": num(vis, "height_m"),
            "base_across_flats_m": num(vis, "base_across_flats_m"),
            "top_across_flats_m": num(vis, "top_across_flats_m"),
            "color": vis.get("color", "#888888"),
        })
    return out


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="separately supplied legacy field XML")
    parser.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT,
                        help="output JSON (default: fields/override.json)")
    args = parser.parse_args(argv[1:])
    src, dst = args.source, args.output
    if not src.is_file():
        parser.error("field XML does not exist: %s" % src)
    doc = convert(src)
    text = json.dumps(doc, indent=2) + "\n"
    with dst.open("w", encoding="ascii", newline="\n") as output:
        output.write(text)
    print("wrote %s (%d objects)" % (dst, len(doc["objects"])))


if __name__ == "__main__":
    main(sys.argv)
