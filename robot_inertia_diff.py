"""Compare URDF mass properties after frame changes and conversion."""

import math
import xml.etree.ElementTree as ET


def matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)
    ]


def parse(text):
    if "<!DOCTYPE" in text.upper() or "<!ENTITY" in text.upper():
        raise ValueError("DTD/entities unsupported")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as error:
        raise ValueError(str(error)) from error
    if root.tag != "robot":
        raise ValueError("expected URDF robot root")
    links = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        if name in links:
            raise ValueError("duplicate link: " + name)
        inertial = link.find("inertial")
        if inertial is None:
            links[name] = None
            continue
        mass_node = inertial.find("mass")
        tensor_node = inertial.find("inertia")
        if mass_node is None or tensor_node is None:
            raise ValueError("incomplete inertial: " + name)
        mass = float(mass_node.attrib["value"])
        a = tensor_node.attrib
        xx, yy, zz, xy, xz, yz = [
            float(a[k]) for k in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
        ]
        tensor = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
        origin = inertial.find("origin")
        xyz = (
            [0.0, 0.0, 0.0]
            if origin is None
            else list(map(float, origin.get("xyz", "0 0 0").split()))
        )
        rpy = (
            [0.0, 0.0, 0.0]
            if origin is None
            else list(map(float, origin.get("rpy", "0 0 0").split()))
        )
        if (
            len(xyz) != 3
            or len(rpy) != 3
            or not all(
                math.isfinite(x) for x in [mass, xx, yy, zz, xy, xz, yz] + xyz + rpy
            )
        ):
            raise ValueError("invalid inertial numeric value")
        r, p, y = rpy
        cr, sr, cp, sp, cy, sy = (
            math.cos(r),
            math.sin(r),
            math.cos(p),
            math.sin(p),
            math.cos(y),
            math.sin(y),
        )
        rotation = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
        rotated = matmul(matmul(rotation, tensor), list(map(list, zip(*rotation))))
        norm = sum(v * v for v in xyz)
        at_origin = [
            [
                rotated[i][j] + mass * ((norm if i == j else 0) - xyz[i] * xyz[j])
                for j in range(3)
            ]
            for i in range(3)
        ]
        det = (
            xx * (yy * zz - yz * yz)
            - xy * (xy * zz - yz * xz)
            + xz * (xy * yz - yy * xz)
        )
        issues = []
        if mass <= 0:
            issues.append("nonpositive_mass")
        if not (xx > 0 and xx * yy - xy * xy > 0 and det > 0):
            issues.append("tensor_not_positive_definite")
        links[name] = {
            "mass": mass,
            "com": xyz,
            "inertia_at_link_origin": at_origin,
            "issues": issues,
        }
    return links


def analyze(data):
    before, after = parse(data["before"]), parse(data["after"])
    tol = float(data.get("tolerance", 1e-9))
    if not math.isfinite(tol) or tol < 0:
        raise ValueError("invalid tolerance")
    changes = []
    for name in sorted(before.keys() | after.keys()):
        if name not in before or name not in after:
            changes.append(
                {"link": name, "kind": "added" if name in after else "removed"}
            )
            continue
        a, b = before[name], after[name]
        if a is None or b is None:
            if a != b:
                changes.append({"link": name, "kind": "inertial_presence_changed"})
            continue
        mass = abs(a["mass"] - b["mass"])
        com = max(abs(x - y) for x, y in zip(a["com"], b["com"]))
        tensor = max(
            abs(a["inertia_at_link_origin"][i][j] - b["inertia_at_link_origin"][i][j])
            for i in range(3)
            for j in range(3)
        )
        if max(mass, com, tensor) > tol:
            changes.append(
                {
                    "link": name,
                    "kind": "physical_change",
                    "mass_delta": mass,
                    "max_com_delta": com,
                    "max_tensor_delta": tensor,
                }
            )
    issues = [
        {"side": side, "link": name, "issues": v["issues"]}
        for side, links in [("before", before), ("after", after)]
        for name, v in sorted(links.items())
        if v and v["issues"]
    ]
    return {
        "equivalent_within_tolerance": not changes,
        "changes": changes,
        "physical_warnings": issues,
    }


def main():
    """Analyze a JSON envelope or explicitly selected native files."""
    import argparse
    import json
    import sys
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", help="JSON input path, or - for stdin")
    parser.add_argument("--before", help="Read before directly from a local file")
    parser.add_argument("--after", help="Read after directly from a local file")
    parser.add_argument("--tolerance", type=float, default=1e-9)
    args = parser.parse_args()
    native = bool(args.before or args.after)
    if native and (args.input or not (args.before and args.after)):
        parser.error("Supply all native file flags, without a positional JSON input")
    if not native and not args.input:
        parser.error("Supply a JSON input or native file flags")
    try:
        if native:
            data = {
                "before": Path(args.before).read_text(),
                "after": Path(args.after).read_text(),
                "tolerance": args.tolerance,
            }
        else:
            raw = (
                sys.stdin.read()
                if args.input == "-"
                else Path(args.input).read_text(encoding="utf-8")
            )
            data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError("input must be a JSON object")
        result = analyze(data)
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        print(f"Invalid input: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
