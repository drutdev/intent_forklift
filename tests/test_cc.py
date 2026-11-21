import unittest

import numpy as np

from pallet_engagement.cc import connectors, endpoint, integrate, Segment, turn


class CCChecks(unittest.TestCase):
    def test_elementary_turn_stays_on_cc_circle(self):
        limit, sharpness = .45, .3
        q = endpoint((0, 0, 0, 0), [Segment(limit / sharpness, 0, sharpness)])
        center = q[:2] + np.array([-np.sin(q[2]), np.cos(q[2])]) / limit
        radius = np.linalg.norm(center)
        mu = np.arctan2(center[0], center[1])
        for angle in (.001, .1, .674, .676, 1.2):
            parts = turn(angle, 0, limit, sharpness)
            end = endpoint((0, 0, 0, 0), parts)
            self.assertAlmostEqual(np.linalg.norm(end[:2] - center), radius, places=10)
            self.assertAlmostEqual(end[2], angle)
            self.assertAlmostEqual(end[3], 0)
            for p in parts:
                self.assertLessEqual(abs(p.sharpness), sharpness + 1e-12)
                self.assertLessEqual(abs(p.curvature), limit + 1e-12)
            # Reflected construction has the same length and mirrored endpoint.
            mirror = endpoint((0, 0, 0, 0), turn(-angle, 0, limit, sharpness))
            np.testing.assert_allclose(mirror, end * [1, -1, -1, -1], atol=1e-10)
        short = turn(.1, 0, limit, sharpness)
        self.assertLess(max(abs(p.curvature) for p in short), limit)
        self.assertLess(abs(short[0].sharpness), sharpness)
        self.assertGreater(2 * radius * np.sin(mu), 0)

    def test_feedback_curvature_and_rejected_boundary(self):
        parts = turn(.3, .12, .45, .3)
        pose = np.array([0., 0., .2, .12])
        for p in parts:
            self.assertAlmostEqual(p.curvature, pose[3])
            pose = integrate(pose, p, p.length)
        np.testing.assert_allclose(pose[2:], [.5, 0], atol=1e-12)
        opposite = endpoint((0, 0, 0, .12), turn(-.3, .12, .45, .3))
        np.testing.assert_allclose(opposite[2:], [-.3, 0], atol=1e-12)
        self.assertIsNone(turn(.001, .12, .45, .3))

    def test_csc_closure_and_mirror(self):
        start = (-6., -1., .35, 0.)
        paths = list(connectors(start, -1., (-2.8, -1.7), .45, .3))
        self.assertTrue(paths)
        reflected = list(connectors(np.array(start) * [1, -1, -1, -1], -1., (-2.8, -1.7), .45, .3))
        rows, join, family = paths[0]
        self.assertIn(family, ('LSL', 'LSR', 'RSL', 'RSR', 'SL', 'SR'))
        np.testing.assert_allclose(rows[0], start)
        np.testing.assert_allclose(rows[join, 1:], 0, atol=1e-8)
        np.testing.assert_allclose(rows[-1], [-1, 0, 0, 0], atol=1e-8)
        self.assertLessEqual(abs(rows[:, 3]).max(), .45 + 1e-10)
        self.assertTrue(any(np.allclose(r, rows * [1, -1, -1, -1]) for r, j, f in reflected if r.shape == rows.shape))
