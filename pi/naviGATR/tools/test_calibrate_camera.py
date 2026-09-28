"""Host checks for calibration math, image capture, and exported XML.

Run with the same OpenCV/numpy environment as calibrate_camera.py:
    python -m unittest discover -s tools -p test_calibrate_camera.py
No camera, Navigatr process, or Pi is required.
"""

import contextlib
import http.server
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import cv2 as cv
import numpy as np

import calibrate_camera as tool


SIZE = (1280, 960)
K = np.array([[840.0, 0.0, 640.0], [0.0, 820.0, 480.0], [0.0, 0.0, 1.0]])


def board_pose(index):
    rotation = np.array([0.30 * np.sin(index * 1.1),
                         0.40 * np.cos(index * 0.8),
                         0.15 * np.sin(index * 0.4)])
    translation = np.array([-0.1 + 0.10 * np.sin(index * 0.6),
                            -0.07 + 0.09 * np.cos(index * 0.7),
                            0.50 + 0.025 * (index % 6)])
    return rotation, translation


def render_boards(directory, count=16):
    # Ten by seven squares -> nine by six interior corners. World origin is
    # the first interior corner, independently of the tool's point builder.
    square = 80
    board = np.full((7 * square, 10 * square), 255, np.uint8)
    for row in range(7):
        for col in range(10):
            if (row + col) % 2 == 0:
                board[row*square:(row+1)*square, col*square:(col+1)*square] = 0
    source = np.float32([[0, 0], [800, 0], [800, 560], [0, 560]])
    corners = np.float32([[-0.025, -0.025, 0], [0.225, -0.025, 0],
                          [0.225, 0.150, 0], [-0.025, 0.150, 0]])
    for i in range(count):
        rotation, translation = board_pose(i)
        projected, _ = cv.projectPoints(corners, rotation, translation, K, np.zeros(5))
        homography = cv.getPerspectiveTransform(source, projected.reshape(4, 2))
        image = cv.warpPerspective(board, homography, SIZE, borderValue=255)
        assert cv.imwrite(str(directory / f"view_{i:02d}.png"), image)


class CalibrationTests(unittest.TestCase):
    def test_recovers_known_camera_from_projected_measurements(self):
        points = np.array([[col * 0.025, row * 0.025, 0.0]
                           for row in range(6) for col in range(9)], np.float32)
        distortion = np.array([-0.10, 0.03, 0.001, -0.002, 0.002])
        views = []
        for i in range(20):
            rotation, translation = board_pose(i)
            image_points, _ = cv.projectPoints(points, rotation, translation, K, distortion)
            views.append(image_points)
        matrix, fitted, rms, per_view = tool.solve_calibration(views, points, SIZE, cv, np)
        np.testing.assert_allclose(matrix, K, atol=0.02, rtol=0)
        np.testing.assert_allclose(fitted, distortion, atol=0.002, rtol=0)
        self.assertLess(rms, 0.002)
        self.assertEqual(len(per_view), len(views))

    def test_full_image_to_xml_workflow_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            images = root / "images"
            images.mkdir()
            render_boards(images)
            output = root / "result"
            args = ["calibrate", "--images", str(images), "--width", "1280",
                    "--height", "960", "--cols", "9", "--rows", "6",
                    "--square-size-m", "0.025", "--calibration-id", "synthetic-test",
                    "--frame-id", "front_camera_engineering", "--output-dir", str(output)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(args), 0)
            report = json.loads((output / "calibration.json").read_text())
            self.assertGreaterEqual(len(report["used_images"]), 10)
            self.assertFalse(report["mounting_estimated"])
            self.assertEqual(report["distortion_order"], ["k1", "k2", "p1", "p2", "k3"])
            fitted = np.array(report["camera_matrix"])
            self.assertLess(abs(fitted[0, 0] - K[0, 0]) / K[0, 0], 0.02)
            calibration = ET.parse(output / "calibration.xml").getroot()
            self.assertEqual(calibration.tag, "Calibration")
            intrinsics = calibration.find("Intrinsics")
            self.assertEqual(intrinsics.get("model"), "brown_conrady")
            self.assertEqual(intrinsics.get("calibrated_width_px"), "1280")
            self.assertEqual(float(intrinsics.get("fx_px")), report["camera_matrix"][0][0])
            for name, value in zip(report["distortion_order"], report["distortion"]):
                self.assertEqual(float(intrinsics.get(name)), value)
            self.assertEqual(calibration.find("Extrinsic").get("frame_id"), "front_camera_engineering")
            original = (output / "calibration.xml").read_bytes()
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(tool.main(args), 1)
            self.assertEqual((output / "calibration.xml").read_bytes(), original)

    def test_rejects_resized_or_unreadable_images(self):
        _, encoded = cv.imencode(".jpg", np.zeros((480, 640), np.uint8))
        with self.assertRaisesRegex(ValueError, "expected 1280x960"):
            tool.image_data(encoded.tobytes(), SIZE, "preview", cv, np)
        with self.assertRaisesRegex(ValueError, "unreadable"):
            tool.image_data(b"not an image", SIZE, "broken", cv, np)

    def test_rejects_insufficient_measurements(self):
        with self.assertRaisesRegex(ValueError, "at least 10"):
            tool.solve_calibration([], np.zeros((54, 3), np.float32), SIZE, cv, np)

    def test_http_capture_preserves_original_and_skips_duplicate(self):
        _, encoded = cv.imencode(".jpg", np.full((960, 1280), 127, np.uint8))
        jpeg = encoded.tobytes()

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.end_headers()
                self.wfile.write(jpeg)

            def log_message(self, *args):
                pass

        with http.server.HTTPServer(("127.0.0.1", 0), Handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                with tempfile.TemporaryDirectory() as temporary:
                    output = Path(temporary) / "capture"
                    args = ["capture", "--url", f"http://127.0.0.1:{server.server_port}/api/frame.jpg?camera=front_camera",
                            "--width", "1280", "--height", "960", "--output-dir", str(output)]
                    with patch("builtins.input", side_effect=["", "", "q"]), contextlib.redirect_stdout(io.StringIO()):
                        self.assertEqual(tool.main(args), 0)
                    self.assertEqual(len(list(output.glob("*.jpg"))), 1)
                    self.assertEqual((output / "frame_0001.jpg").read_bytes(), jpeg)
                    manifest = json.loads((output / "capture_manifest.json").read_text())
                    self.assertEqual(len(manifest["images"]), 1)
            finally:
                server.shutdown()
                thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
