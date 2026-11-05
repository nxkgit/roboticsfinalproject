#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-10-28
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Shape Drawing
#
# shape_geometry.py
# Pure-Python geometry for the shape drawer (stdlib + yaml, no ROS).
# Loads normalized shape specs, fits them into the usable part of a
# W x H area, densifies arcs into chords, converts between the area
# frame and the odom frame, and holds the controller / error math so
# all of it can be unit tested without ROS.
#
# Area frame A: origin at the lower-left corner, +x along W, +y along H.
# The robot starts at (start_x, start_y) in A, facing +x.
# ---------------------------------------------------------
import math
import os
from collections import namedtuple

import yaml

Waypoint = namedtuple('Waypoint', 'x y stop')          # meters; stop=True => settle here
FitResult = namedtuple('FitResult', 'ok scale offset reason')

SEGMENT_TYPES = ('line', 'arc')


# ---------------------------------------------------------
# Loading
# ---------------------------------------------------------
def load_shape(name_or_path, shapes_dir):
    """Load a shape spec from a YAML path, or by name from shapes_dir."""
    path = name_or_path
    if not os.path.isfile(path):
        path = os.path.join(shapes_dir, name_or_path)
        if not path.endswith('.yaml'):
            path += '.yaml'
    if not os.path.isfile(path):
        raise ValueError('shape "%s" not found (looked in %s)' % (name_or_path, shapes_dir))
    with open(path) as f:
        spec = yaml.safe_load(f)
    if not isinstance(spec, dict) or 'start' not in spec or not spec.get('segments'):
        raise ValueError('shape file %s needs "start" and a non-empty "segments" list' % path)
    spec['start'] = [float(v) for v in spec['start']]
    for i, seg in enumerate(spec['segments']):
        if seg.get('type') not in SEGMENT_TYPES:
            raise ValueError('%s segment %d: unknown type %r' % (path, i, seg.get('type')))
        if seg['type'] == 'line' and 'to' not in seg:
            raise ValueError('%s segment %d: line needs "to"' % (path, i))
        if seg['type'] == 'arc' and ('center' not in seg or 'sweep_deg' not in seg):
            raise ValueError('%s segment %d: arc needs "center" and "sweep_deg"' % (path, i))
    spec.setdefault('name', os.path.splitext(os.path.basename(path))[0])
    return spec


# ---------------------------------------------------------
# Densify / transform
# ---------------------------------------------------------
def densify(spec, max_chord):
    """Expand a spec into waypoints; arcs become chords no longer than max_chord.

    The start point and the end of every segment are stop points; interior
    arc points are pass points. Lines are not subdivided.
    """
    x, y = spec['start']
    wps = [Waypoint(x, y, True)]
    for seg in spec['segments']:
        if seg['type'] == 'line':
            x, y = float(seg['to'][0]), float(seg['to'][1])
            wps.append(Waypoint(x, y, True))
        else:
            cx, cy = float(seg['center'][0]), float(seg['center'][1])
            r = math.hypot(x - cx, y - cy)
            a0 = math.atan2(y - cy, x - cx)
            sweep = math.radians(float(seg['sweep_deg']))
            # chord = 2 r sin(dθ/2) <= r dθ, so arc length / n bounds the chord
            n = max(1, int(math.ceil(r * abs(sweep) / max_chord)))
            for i in range(1, n + 1):
                a = a0 + sweep * i / n
                wps.append(Waypoint(cx + r * math.cos(a), cy + r * math.sin(a), i == n))
            x, y = wps[-1].x, wps[-1].y
    return wps


def transform_spec(spec, scale, offset):
    """Return a copy of spec scaled uniformly about the origin, then shifted by offset."""
    ox, oy = offset

    def tf(p):
        return [scale * float(p[0]) + ox, scale * float(p[1]) + oy]

    out = dict(spec)
    out['start'] = tf(spec['start'])
    out['segments'] = []
    for seg in spec['segments']:
        seg = dict(seg)
        if seg['type'] == 'line':
            seg['to'] = tf(seg['to'])
        else:
            seg['center'] = tf(seg['center'])
        out['segments'].append(seg)
    return out


def _extent(spec):
    """Rough size of a spec (max |coordinate| plus arc radii), used to pick a fine chord."""
    x, y = spec['start']
    ext = max(abs(x), abs(y))
    for seg in spec['segments']:
        if seg['type'] == 'line':
            x, y = seg['to']
        else:
            cx, cy = seg['center']
            r = math.hypot(x - cx, y - cy)
            ext = max(ext, abs(cx) + r, abs(cy) + r)
            a = math.atan2(y - cy, x - cx) + math.radians(seg['sweep_deg'])
            x, y = cx + r * math.cos(a), cy + r * math.sin(a)
        ext = max(ext, abs(x), abs(y))
    return ext


def bounding_box(points):
    """(xmin, ymin, xmax, ymax) of an iterable of (x, y, ...) points."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def spec_bbox(spec):
    """Bounding box from a fine densification, so arc extremes count."""
    fine = max(_extent(spec), 1e-9) / 2000.0
    return bounding_box(densify(spec, fine))


# ---------------------------------------------------------
# Fit check / plan
# ---------------------------------------------------------
def usable_box(W, H, margin, footprint_r):
    """(xmin, ymin, xmax, ymax) the robot center may use inside a W x H area."""
    d = margin + footprint_r
    return (d, d, W - d, H - d)


def fit_shape(spec, W, H, margin, footprint_r, size=0.0, min_size=0.20, start=None):
    """Scale and center spec in the usable box, or refuse with a reason that has the numbers.

    size == 0: largest uniform scale that fits.
    size > 0: longest bounding-box side becomes `size` meters.
    start: robot start (x, y) in the area frame; defaults to the area center.
    """
    xmin, ymin, xmax, ymax = usable_box(W, H, margin, footprint_r)
    uw, uh = xmax - xmin, ymax - ymin
    area_txt = 'area %.2f x %.2f m, margin %.2f m, footprint_r %.2f m' % (W, H, margin, footprint_r)
    if uw <= 0 or uh <= 0:
        return FitResult(False, 0.0, (0.0, 0.0),
                         'usable box is %.2f x %.2f m (must be > 0) for %s' % (uw, uh, area_txt))

    bx0, by0, bx1, by1 = spec_bbox(spec)
    bw, bh = bx1 - bx0, by1 - by0
    longest = max(bw, bh)
    if longest <= 0:
        return FitResult(False, 0.0, (0.0, 0.0), 'shape "%s" has zero size' % spec.get('name'))

    s_max = min(uw / bw if bw > 0 else float('inf'), uh / bh if bh > 0 else float('inf'))
    if size > 0:
        scale = size / longest
        if scale > s_max * (1 + 1e-9):
            return FitResult(False, 0.0, (0.0, 0.0),
                             'requested size %.2f m makes shape %.2f x %.2f m, which exceeds the '
                             'usable box %.2f x %.2f m (largest size that fits: %.2f m; %s)'
                             % (size, scale * bw, scale * bh, uw, uh, s_max * longest, area_txt))
    else:
        scale = s_max

    if scale * longest < min_size:
        return FitResult(False, 0.0, (0.0, 0.0),
                         'fitted shape is only %.2f x %.2f m (longest side %.2f m < min_size %.2f m) '
                         'in usable box %.2f x %.2f m (%s)'
                         % (scale * bw, scale * bh, scale * longest, min_size, uw, uh, area_txt))

    sx, sy = start if start is not None else (W / 2.0, H / 2.0)
    eps = 1e-9
    if not (xmin - eps <= sx <= xmax + eps and ymin - eps <= sy <= ymax + eps):
        return FitResult(False, 0.0, (0.0, 0.0),
                         'start (%.2f, %.2f) is outside the usable box x %.2f..%.2f, y %.2f..%.2f m (%s)'
                         % (sx, sy, xmin, xmax, ymin, ymax, area_txt))

    offset = ((xmin + xmax) / 2.0 - scale * (bx0 + bx1) / 2.0,
              (ymin + ymax) / 2.0 - scale * (by0 + by1) / 2.0)
    return FitResult(True, scale, offset,
                     'ok: scale %.3f, shape %.2f x %.2f m in usable box %.2f x %.2f m'
                     % (scale, scale * bw, scale * bh, uw, uh))


def plan(spec, W, H, margin, footprint_r, size=0.0, max_chord=0.05, start=None):
    """Fit, scale and densify spec; return waypoints in the area frame. Raises ValueError if refused."""
    fit = fit_shape(spec, W, H, margin, footprint_r, size=size, start=start)
    if not fit.ok:
        raise ValueError(fit.reason)
    return densify(transform_spec(spec, fit.scale, fit.offset), max_chord)


def path_length(points, start_xy=None):
    """Polyline length through points, optionally starting from start_xy."""
    pts = ([tuple(start_xy)] if start_xy is not None else []) + [(p[0], p[1]) for p in points]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def estimate_time(wps, start_xy, speed, settle=0.3):
    """Rough run time: path length at `speed` plus a settle pause at each stop point."""
    return path_length(wps, start_xy) / speed + settle * sum(1 for w in wps if w.stop)


# ---------------------------------------------------------
# Frames
# ---------------------------------------------------------
def wrap_angle(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Yaw (rotation about z) of a quaternion, via atan2."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def area_to_odom(wps, start_xy, odom0):
    """Map area-frame waypoints into odom, given the robot's start in A and its odom pose (x0, y0, yaw0)."""
    x0, y0, yaw0 = odom0
    c, s = math.cos(yaw0), math.sin(yaw0)
    out = []
    for w in wps:
        dx, dy = w.x - start_xy[0], w.y - start_xy[1]
        out.append(Waypoint(x0 + c * dx - s * dy, y0 + s * dx + c * dy, w.stop))
    return out


def odom_to_area(x, y, start_xy, odom0):
    """Inverse of area_to_odom for a single point; returns (x, y) in the area frame."""
    x0, y0, yaw0 = odom0
    c, s = math.cos(yaw0), math.sin(yaw0)
    dx, dy = x - x0, y - y0
    return (start_xy[0] + c * dx + s * dy, start_xy[1] - s * dx + c * dy)


def inside_box(x, y, box):
    """True if (x, y) lies inside box = (xmin, ymin, xmax, ymax)."""
    return box[0] <= x <= box[2] and box[1] <= y <= box[3]


# ---------------------------------------------------------
# Controller math
# ---------------------------------------------------------
def body_velocity(ex, ey, yaw, kp, v_max, v_min, tol):
    """World-frame position error -> body-frame (vx, vy).

    P command, norm clamped to v_max (direction kept), raised to v_min while
    the error is above tol (motor deadband), then rotated by -yaw.
    """
    wx, wy = kp * ex, kp * ey
    v = math.hypot(wx, wy)
    if v > v_max:
        wx, wy = wx * v_max / v, wy * v_max / v
    elif v > 0 and math.hypot(ex, ey) > tol and v < v_min:
        v_floor = min(v_min, v_max)
        wx, wy = wx * v_floor / v, wy * v_floor / v
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * wx + s * wy, -s * wx + c * wy)


def heading_rate(yaw_target, yaw, kyaw, w_max):
    """P yaw rate toward yaw_target; error wrapped to [-pi, pi], output clamped to +-w_max."""
    w = kyaw * wrap_angle(yaw_target - yaw)
    return max(-w_max, min(w_max, w))


# ---------------------------------------------------------
# Error metrics
# ---------------------------------------------------------
def closure_error(actual_xy, planned_start_xy):
    """Distance from the last actual point to the planned start (closed shapes end where they began)."""
    x, y = actual_xy[-1][0], actual_xy[-1][1]
    return math.hypot(x - planned_start_xy[0], y - planned_start_xy[1])


def _point_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def cross_track_errors(actual_xy, planned_polyline):
    """Distance from each actual point to the nearest segment of the planned polyline."""
    pl = [(p[0], p[1]) for p in planned_polyline]
    if len(pl) == 1:
        pl = pl * 2
    segs = list(zip(pl, pl[1:]))
    return [min(_point_segment_dist(p[0], p[1], a[0], a[1], b[0], b[1]) for a, b in segs)
            for p in actual_xy]
