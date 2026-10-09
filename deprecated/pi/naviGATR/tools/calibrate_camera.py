#!/usr/bin/env python3
"""Capture Navigatr inspection JPEGs and calibrate a pinhole camera.

Requires Python 3.9+, numpy, and opencv-python-headless. Run --help without
installing dependencies. Calibration follows OpenCV's chessboard workflow:
https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

Use the same camera, lens/focus, crop, and capture mode as live detection.
Inspection.preview_max_width must be >= Capture.width_px and preview_quality
must be 100. Query-string width/quality parameters do not configure the server.
The endpoint exports unannotated JPEGs, not lossless sensor images. No resizing,
undistortion, or annotation is performed by this tool.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET


MIN_VIEWS = 10
MAX_IMAGE_BYTES = 32 * 1024 * 1024


def positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return value


def positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def dependencies():
    try:
        import cv2
        import numpy
    except ImportError as error:
        raise ValueError(
            "Install dependencies in your Python environment: "
            "python -m pip install opencv-python-headless numpy"
        ) from error
    return cv2, numpy


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def image_data(data, expected_size, label, cv, np):
    image = cv.imdecode(np.frombuffer(data, dtype=np.uint8), cv.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"{label}: unreadable image")
    height, width = image.shape
    if (width, height) != expected_size:
        raise ValueError(
            f"{label}: image is {width}x{height}; expected "
            f"{expected_size[0]}x{expected_size[1]}. Check capture and inspection "
            "preview settings; do not resize calibration images."
        )
    return image, hashlib.sha256(image.tobytes()).hexdigest()


def fetch_jpeg(url, timeout):
    if urllib.parse.urlsplit(url).scheme not in ("http", "https"):
        raise ValueError("capture URL must use http:// or https://")
    request = urllib.request.Request(url, headers={"Cache-Control": "no-cache"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        if response.headers.get_content_type() != "image/jpeg":
            raise ValueError("camera endpoint did not return image/jpeg")
        data = response.read(MAX_IMAGE_BYTES + 1)
    if len(data) > MAX_IMAGE_BYTES:
        raise ValueError("camera image exceeds 32 MiB")
    return data


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def capture(args, cv, np):
    # A new directory prevents accidental replacement or mixing camera modes.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "source_url": args.url,
        "expected_size_px": [args.width, args.height],
        "started_at_utc": utc_now(),
        "images": [],
    }
    manifest_path = args.output_dir / "capture_manifest.json"
    write_json(manifest_path, manifest)
    seen = set()
    print("Move a flat chessboard through different positions, distances, and tilts.")
    print("Keep the whole board visible; include the image edges. Save 15-25 clear views.")
    print("Press Enter to save the current frame, or q then Enter to finish.")
    while True:
        try:
            command = input(f"Capture {len(seen) + 1}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if command in ("q", "quit"):
            break
        if command:
            print("Use Enter to capture or q to finish.")
            continue
        try:
            data = fetch_jpeg(args.url, args.timeout)
            _, pixel_hash = image_data(
                data, (args.width, args.height), args.url, cv, np
            )
        except (OSError, ValueError, urllib.error.URLError) as error:
            print(f"Not saved: {error}", file=sys.stderr)
            continue
        if pixel_hash in seen:
            print("Not saved: identical image already captured (possibly a stale frame).")
            continue
        name = f"frame_{len(seen) + 1:04d}.jpg"
        with (args.output_dir / name).open("xb") as destination:
            destination.write(data)
        seen.add(pixel_hash)
        manifest["images"].append({
            "file": name,
            "received_at_utc": utc_now(),
            "sha256": hashlib.sha256(data).hexdigest(),
            "pixels_sha256": pixel_hash,
        })
        write_json(manifest_path, manifest)
        print(f"Saved {args.output_dir / name}")
    print(f"Saved {len(seen)} distinct images; calibration needs {MIN_VIEWS} usable views.")


def board_points(cols, rows, square_size, np):
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return points


def detect_views(paths, size, board_size, cv, np):
    image_points, records, skipped = [], [], []
    seen = set()
    for path in paths:
        data = path.read_bytes()
        gray, pixel_hash = image_data(data, size, str(path), cv, np)
        record = {"file": str(path.resolve()), "sha256": hashlib.sha256(data).hexdigest()}
        if pixel_hash in seen:
            skipped.append(dict(record, reason="duplicate image"))
            continue
        seen.add(pixel_hash)
        found, corners = cv.findChessboardCorners(
            gray, board_size, cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE
        )
        if not found:
            skipped.append(dict(record, reason="complete chessboard not detected"))
            continue
        corners = cv.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_MAX_ITER, 50, 0.001),
        )
        image_points.append(corners)
        records.append(record)
    if len(image_points) < MIN_VIEWS:
        raise ValueError(
            f"Only {len(image_points)} distinct usable chessboard views "
            f"({len(skipped)} skipped); need at least {MIN_VIEWS}. "
            "Check INNER-corner counts, board visibility, and sharpness."
        )
    return image_points, records, skipped


def solve_calibration(image_points, object_points, size, cv, np):
    if len(image_points) < MIN_VIEWS:
        raise ValueError(f"Need at least {MIN_VIEWS} usable views")
    objects = [object_points for _ in image_points]
    rms, matrix, distortion, rotations, translations = cv.calibrateCamera(
        objects, image_points, size, None, None,
        flags=0,
        criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_MAX_ITER, 100, 1e-10),
    )
    distortion = distortion.reshape(-1)
    if (distortion.size != 5 or not math.isfinite(rms) or rms < 0
            or not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0):
        raise ValueError("Calibration produced invalid parameters; recapture diverse views")
    per_view = []
    for points, rotation, translation in zip(image_points, rotations, translations):
        projected, _ = cv.projectPoints(object_points, rotation, translation, matrix, distortion)
        residual = points.reshape(-1, 2) - projected.reshape(-1, 2)
        per_view.append(float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))
    return matrix, distortion, float(rms), per_view


def calibration_xml(calibration_id, frame_id, size, matrix, distortion, rms):
    root = ET.Element("Calibration", calibration_id=calibration_id)
    values = {
        "model": "brown_conrady",
        "calibrated_width_px": str(size[0]), "calibrated_height_px": str(size[1]),
        "fx_px": format(matrix[0, 0], ".17g"), "fy_px": format(matrix[1, 1], ".17g"),
        "cx_px": format(matrix[0, 2], ".17g"), "cy_px": format(matrix[1, 2], ".17g"),
    }
    values.update({name: format(value, ".17g") for name, value in zip(
        ("k1", "k2", "p1", "p2", "k3"), distortion
    )})
    values["rms_reprojection_px"] = format(rms, ".17g")
    ET.SubElement(root, "Intrinsics", values)
    ET.SubElement(root, "Extrinsic", frame_id=frame_id)
    ET.indent(root, space="    ")
    return ET.tostring(root, encoding="unicode") + "\n"


def calibrate(args, cv, np):
    if args.output_dir.exists():
        raise ValueError(f"Output directory already exists: {args.output_dir}")
    if not args.images.is_dir():
        raise ValueError(f"Image directory does not exist: {args.images}")
    if args.cols < 2 or args.rows < 2:
        raise ValueError("Chessboard needs at least 2 INNER corners in each direction")
    paths = sorted(path for path in args.images.iterdir()
                   if path.is_file() and path.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not paths:
        raise ValueError(f"No JPEG/PNG images found in {args.images}")
    size = (args.width, args.height)
    views, records, skipped = detect_views(paths, size, (args.cols, args.rows), cv, np)
    points = board_points(args.cols, args.rows, args.square_size_m, np)
    matrix, distortion, rms, per_view = solve_calibration(views, points, size, cv, np)
    for record, error in zip(records, per_view):
        record["rms_reprojection_px"] = error
    report = {
        "calibration_id": args.calibration_id,
        "created_at_utc": utc_now(),
        "opencv_version": cv.__version__,
        "model": "brown_conrady",
        "image_size_px": list(size),
        "board": {"inner_cols": args.cols, "inner_rows": args.rows,
                  "square_size_m": args.square_size_m},
        "camera_matrix": matrix.tolist(),
        "distortion_order": ["k1", "k2", "p1", "p2", "k3"],
        "distortion": distortion.tolist(),
        "rms_reprojection_px": rms,
        "used_images": records,
        "skipped_images": skipped,
        "extrinsic_frame_reference": args.frame_id,
        "mounting_estimated": False,
        "notes": [
            "View diversity must be checked manually; distinct images alone are insufficient.",
            "Use unchanged camera, focus, crop, and capture resolution.",
            "Extrinsic references a separately measured robot mounting frame.",
        ],
    }
    manifest_path = args.images / "capture_manifest.json"
    if manifest_path.is_file():
        report["capture_manifest"] = json.loads(manifest_path.read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write_json(args.output_dir / "calibration.json", report)
    (args.output_dir / "calibration.xml").write_text(
        calibration_xml(args.calibration_id, args.frame_id, size, matrix, distortion, rms),
        encoding="utf-8",
    )
    print(f"Used {len(views)} views; skipped {len(skipped)}. RMS: {rms:.4f} px")
    print(f"Wrote {args.output_dir / 'calibration.json'} and calibration.xml")
    print("Review per-view errors and capture diversity. Mounting must be measured separately.")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="Save user-triggered inspection JPEGs")
    capture_parser.add_argument("--url", required=True, help="HTTP /api/frame.jpg?camera=... URL")
    capture_parser.add_argument("--timeout", type=positive_float, default=10.0,
                                help="HTTP timeout in seconds (default: 10)")
    calibrate_parser = commands.add_parser("calibrate", help="Fit intrinsics from chessboard images")
    calibrate_parser.add_argument("--images", type=Path, required=True, help="JPEG/PNG directory")
    calibrate_parser.add_argument("--cols", type=positive_int, required=True, help="INNER corner columns")
    calibrate_parser.add_argument("--rows", type=positive_int, required=True, help="INNER corner rows")
    calibrate_parser.add_argument("--square-size-m", type=positive_float, required=True,
                                  help="Measured chessboard square edge in metres")
    calibrate_parser.add_argument("--calibration-id", required=True, help="Name for this calibration run")
    calibrate_parser.add_argument("--frame-id", required=True, help="Existing robot camera mount frame ID")
    for command in (capture_parser, calibrate_parser):
        command.add_argument("--width", type=positive_int, required=True, help="Actual capture width in pixels")
        command.add_argument("--height", type=positive_int, required=True, help="Actual capture height in pixels")
        command.add_argument("--output-dir", type=Path, required=True, help="NEW directory; never overwritten")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        cv, np = dependencies()
        if args.command == "capture":
            capture(args, cv, np)
        else:
            calibrate(args, cv, np)
    except (ValueError, OSError, cv.error if "cv" in locals() else ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
