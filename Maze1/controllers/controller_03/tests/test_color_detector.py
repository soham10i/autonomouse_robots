import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from perception.color_detector import ColorDetector, bgr_to_hsv


def _blue_img(w, h, c0, c1):
    bgr = np.zeros((h, w, 3), dtype=np.uint8)
    bgr[10:34, c0:c1] = (255, 0, 0)        # blue blob (BGR) -> H~120
    return bgr


class TestColorDetectorEdgeClip(unittest.TestCase):
    def setUp(self):
        self.det = ColorDetector(64, 48, 1.04)
        self.depth = np.full((48, 64), 1.0, dtype=np.float32)

    def _detect(self, img):
        return self.det._detect_one(bgr_to_hsv(img), self.depth,
                                    *S.HSV_BLUE, S.PILLAR_MIN_PIXELS)

    def test_centered_blob_is_not_side_clipped(self):
        det = self._detect(_blue_img(64, 48, 28, 40))     # well inside the frame
        self.assertIsNotNone(det)
        self.assertFalse(det["side_clipped"])

    def test_left_edge_blob_is_side_clipped(self):
        det = self._detect(_blue_img(64, 48, 0, 12))      # runs off the left edge
        self.assertIsNotNone(det)
        self.assertTrue(det["side_clipped"])

    def test_right_edge_blob_is_side_clipped(self):
        det = self._detect(_blue_img(64, 48, 52, 64))     # runs off the right edge
        self.assertIsNotNone(det)
        self.assertTrue(det["side_clipped"])


if __name__ == "__main__":
    unittest.main()
