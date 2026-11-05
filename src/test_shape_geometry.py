#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-10-28
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Shape Drawing
#
# test_shape_geometry.py
# unittest tests for shape_geometry.py (no ROS needed).
# Run: cd src && python3 -m unittest -v test_shape_geometry
# ---------------------------------------------------------
import math
import os
import unittest

import shape_geometry as sg

SHAPES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'shapes')
ALL_SHAPES = ('square', 'rectangle', 'triangle', 'circle', 'star', 'figure8')

# Defaults from the task: 1.5 x 1.5 m area, 0.15 m margin, 0.15 m footprint radius
W = H = 1.5
M = R = 0.15


def load(name):
    return sg.load_shape(name, SHAPES_DIR)


class TestFit(unittest.TestCase):

    def test_default_usable_box(self):
        box = sg.usable_box(W, H, M, R)
        for got, want in zip(box, (0.30, 0.30, 1.20, 1.20)):
            self.assertAlmostEqual(got, want, places=9)

    def test_square_autofits_to_0_9(self):
        spec = load('square')
        fit = sg.fit_shape(spec, W, H, M, R)
        self.assertTrue(fit.ok, fit.reason)
        self.assertAlmostEqual(fit.scale, 0.9, places=6)
        wps = sg.plan(spec, W, H, M, R)
        corners = [(round(w.x, 6), round(w.y, 6)) for w in wps]
        self.assertEqual(corners, [(0.3, 0.3), (1.2, 0.3), (1.2, 1.2), (0.3, 1.2), (0.3, 0.3)])
        self.assertTrue(all(w.stop for w in wps))
        self.assertAlmostEqual(sg.path_length(wps), 3.6, places=6)

    def test_size_1_refused(self):
        fit = sg.fit_shape(load('square'), W, H, M, R, size=1.0)
        self.assertFalse(fit.ok)
        self.assertIn('1.00', fit.reason)
        self.assertIn('0.90', fit.reason)
        with self.assertRaises(ValueError):
            sg.plan(load('square'), W, H, M, R, size=1.0)

    def test_size_explicit_fits(self):
        fit = sg.fit_shape(load('square'), W, H, M, R, size=0.5)
        self.assertTrue(fit.ok, fit.reason)
        self.assertAlmostEqual(fit.scale, 0.5, places=6)

    def test_small_area_refused(self):
        # 0.5 x 0.5: usable box is 0.5 - 2*0.3 = -0.1 m
        fit = sg.fit_shape(load('square'), 0.5, 0.5, M, R)
        self.assertFalse(fit.ok)
        self.assertIn('-0.10', fit.reason)

    def test_below_min_size_refused(self):
        # 0.8 x 0.8: usable box 0.2 m, so the square fits at 0.2 m, below min_size 0.25
        fit = sg.fit_shape(load('square'), 0.8, 0.8, M, R, min_size=0.25)
        self.assertFalse(fit.ok)
        self.assertIn('min_size 0.25', fit.reason)

    def test_start_outside_usable_refused(self):
        fit = sg.fit_shape(load('square'), W, H, M, R, start=(0.1, 0.75))
        self.assertFalse(fit.ok)
        self.assertIn('(0.10, 0.75)', fit.reason)

    def test_all_shapes_inside_usable_box(self):
        xmin, ymin, xmax, ymax = sg.usable_box(W, H, M, R)
        eps = 1e-9
        for name in ALL_SHAPES:
            wps = sg.plan(load(name), W, H, M, R)
            self.assertGreater(len(wps), 1, name)
            for w in wps:
                self.assertTrue(xmin - eps <= w.x <= xmax + eps and ymin - eps <= w.y <= ymax + eps,
                                '%s point (%.4f, %.4f) outside usable box' % (name, w.x, w.y))
            # closed shapes end where they start
            self.assertAlmostEqual(wps[0].x, wps[-1].x, places=6, msg=name)
            self.assertAlmostEqual(wps[0].y, wps[-1].y, places=6, msg=name)
            self.assertTrue(wps[-1].stop, name)

    def test_circle_chords(self):
        max_chord = 0.05
        wps = sg.plan(load('circle'), W, H, M, R, max_chord=max_chord)
        chords = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(wps, wps[1:])]
        self.assertTrue(all(c <= max_chord + 1e-12 for c in chords), max(chords))
        # one full revolution: interior points are pass points, the end is a stop
        self.assertTrue(wps[-1].stop)
        self.assertFalse(any(w.stop for w in wps[1:-1]))
        # fitted to the full 0.9 m box (measured on a fine densification so arc extremes count)
        bx = sg.bounding_box(sg.plan(load('circle'), W, H, M, R, max_chord=0.001))
        self.assertAlmostEqual(bx[2] - bx[0], 0.9, places=3)
        self.assertAlmostEqual(bx[3] - bx[1], 0.9, places=3)

    def test_figure8_aspect(self):
        bx = sg.spec_bbox(load('figure8'))
        w, h = bx[2] - bx[0], bx[3] - bx[1]
        self.assertAlmostEqual(h / w, 2.0, places=4)
        pb = sg.bounding_box(sg.plan(load('figure8'), W, H, M, R, max_chord=0.001))
        self.assertAlmostEqual(pb[3] - pb[1], 0.9, places=3)
        self.assertAlmostEqual(pb[2] - pb[0], 0.45, places=3)

    def test_star_vertices(self):
        spec = load('star')
        self.assertEqual(len(spec['segments']), 10)
        self.assertEqual(spec['start'], [0.0, 1.0])
        for i, seg in enumerate(spec['segments'][:-1]):
            r = math.hypot(*seg['to'])
            self.assertAlmostEqual(r, 0.382 if i % 2 == 0 else 1.0, places=4)


class TestFrames(unittest.TestCase):

    def test_area_to_odom_roundtrip(self):
        wps = [sg.Waypoint(0.3, 0.3, True), sg.Waypoint(1.2, 0.75, False)]
        start = (0.75, 0.75)
        odom0 = (2.0, -1.0, math.pi / 2)
        out = sg.area_to_odom(wps, start, odom0)
        # (0.3,0.3) is 0.45 back and 0.45 right of start; robot faces +y in odom
        self.assertAlmostEqual(out[0].x, 2.0 + 0.45)
        self.assertAlmostEqual(out[0].y, -1.0 - 0.45)
        self.assertTrue(out[0].stop)
        self.assertFalse(out[1].stop)
        for w, o in zip(wps, out):
            ax, ay = sg.odom_to_area(o.x, o.y, start, odom0)
            self.assertAlmostEqual(ax, w.x)
            self.assertAlmostEqual(ay, w.y)

    def test_yaw_from_quaternion(self):
        for yaw in (0.0, 0.5, math.pi / 2, -2.0, 3.1):
            q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            self.assertAlmostEqual(sg.yaw_from_quaternion(*q), yaw)


class TestController(unittest.TestCase):

    def test_body_velocity_yaw0(self):
        vx, vy = sg.body_velocity(0.02, 0.04, 0.0, kp=1.5, v_max=0.2, v_min=0.03, tol=0.01)
        self.assertAlmostEqual(vx, 0.03)
        self.assertAlmostEqual(vy, 0.06)

    def test_body_velocity_yaw_pi_2(self):
        # robot faces world +y: a world +x error is a strafe to the right (-y body)
        vx, vy = sg.body_velocity(0.1, 0.0, math.pi / 2, kp=1.0, v_max=0.2, v_min=0.03, tol=0.01)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, -0.1)
        # world +y error is straight ahead
        vx, vy = sg.body_velocity(0.0, 0.1, math.pi / 2, kp=1.0, v_max=0.2, v_min=0.03, tol=0.01)
        self.assertAlmostEqual(vx, 0.1)
        self.assertAlmostEqual(vy, 0.0)

    def test_body_velocity_clamp_keeps_direction(self):
        vx, vy = sg.body_velocity(3.0, 4.0, 0.0, kp=1.5, v_max=0.2, v_min=0.03, tol=0.01)
        self.assertAlmostEqual(math.hypot(vx, vy), 0.2)
        self.assertAlmostEqual(vy / vx, 4.0 / 3.0)

    def test_body_velocity_v_min(self):
        vx, vy = sg.body_velocity(0.0, 0.015, 0.0, kp=1.5, v_max=0.2, v_min=0.03, tol=0.01)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, 0.03)
        # within tol: no deadband boost
        vx, vy = sg.body_velocity(0.005, 0.0, 0.0, kp=1.5, v_max=0.2, v_min=0.03, tol=0.01)
        self.assertAlmostEqual(vx, 0.0075)
        # zero error: zero command
        self.assertEqual(sg.body_velocity(0.0, 0.0, 0.0, 1.5, 0.2, 0.03, 0.01), (0.0, 0.0))

    def test_heading_rate_wraps(self):
        # target just past +pi, robot just past -pi: shortest error is -0.2, not 2pi - 0.2
        w = sg.heading_rate(math.pi - 0.1, -math.pi + 0.1, kyaw=2.0, w_max=1.0)
        self.assertAlmostEqual(w, -0.4)
        w = sg.heading_rate(-math.pi + 0.1, math.pi - 0.1, kyaw=2.0, w_max=1.0)
        self.assertAlmostEqual(w, 0.4)
        self.assertAlmostEqual(sg.heading_rate(1.5, 0.0, kyaw=2.0, w_max=1.0), 1.0)
        self.assertAlmostEqual(sg.heading_rate(-1.5, 0.0, kyaw=2.0, w_max=1.0), -1.0)


class TestMetrics(unittest.TestCase):

    def test_closure_error(self):
        actual = [(0.0, 0.0), (1.0, 0.0), (0.03, 0.04)]
        self.assertAlmostEqual(sg.closure_error(actual, (0.0, 0.0)), 0.05)

    def test_cross_track_errors(self):
        planned = [(0, 0), (1, 0), (1, 1)]
        actual = [(0.5, 0.1), (1.2, 0.5), (-0.3, -0.4), (1.0, 1.5), (0.9, 0.05)]
        errs = sg.cross_track_errors(actual, planned)
        for got, want in zip(errs, [0.1, 0.2, 0.5, 0.5, 0.05]):
            self.assertAlmostEqual(got, want)


if __name__ == '__main__':
    unittest.main()
